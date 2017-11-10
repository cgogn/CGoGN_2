/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France       *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Web site: http://cgogn.unistra.fr/                                           *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/

#ifndef CGOGN_IO_FORMATS_CG_H_
#define CGOGN_IO_FORMATS_CG_H_

#include <set>

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/graph_import.h>
#include <cgogn/io/graph_export.h>

#include <iomanip>

namespace cgogn 
{

namespace io 
{

///
/// Tagliasacchi 2012
///
template <typename VEC3>
class CgGraphImport : public GraphFileImport
{

public:
	using Self = CgGraphImport<VEC3>;
	using Inherit = GraphFileImport;
    using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
    template <typename T>
    using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline CgGraphImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CgGraphImport);
	virtual ~CgGraphImport() override {}

protected:
    virtual bool import_file_impl(const std::string& filename) override
    {
        std::ifstream fp(filename.c_str(), std::ios::in);

        std::string line;
        line.reserve(512);

		io::getline_safe(fp, line);
		if (line.rfind("# D") == std::string::npos)
		{
				cgogn_log_error("CgGraphImport::import_file_impl") << "File \"" << filename << "\" is not a valid cg file.";
				return false;
		}

		// read header
		std::replace(line.begin(), line.end(), ':', ' ');
		std::stringstream oss(line);
		std::string tag;
		uint32 value;
		oss >> tag;
		oss >> tag;
		oss >> value; // dimension unused for now
		oss >> tag;
		oss >> value;
		const uint32 nb_vertices = value;
		oss >> tag;
		oss >> value;
		const uint32 nb_edges = value;

		this->reserve(nb_vertices);

		ChunkArray<VEC3>* position = this->template add_vertex_attribute<VEC3>("position");

		// read vertices
        std::vector<uint32> vertices_id;
		vertices_id.reserve(nb_vertices);

        for (uint32 i = 0; i < nb_vertices; ++i)
        {
            io::getline_safe(fp, line);
            std::stringstream oss(line);

            std::string tag;
            oss >> tag;

            if (tag == std::string("v"))
            {
                float64 x, y, z;
                oss >> x;
                oss >> y;
                oss >> z;

                VEC3 pos{Scalar(x), Scalar(y), Scalar(z)};

                uint32 vertex_id = this->vertex_attributes_.template insert_lines<1>();
                (*position)[vertex_id] = pos;

                vertices_id.push_back(vertex_id);
            }
		}

		// read edges
        for (uint32 i = 0; i < nb_edges; ++i)
        {
            io::getline_safe(fp, line);
            std::stringstream oss(line);

            std::string tag;
            oss >> tag;

            if (tag == std::string("e"))
            {
                uint32  a, b;
                oss >> a;
                oss >> b;

                this->edges_nb_vertices_.push_back(2);
				this->edges_vertex_indices_.push_back(a-1);
				this->edges_vertex_indices_.push_back(b-1);
            }
        }

    	return true;
    }
};

template <typename MAP>
class CgGraphExport : public GraphExport<MAP>
{
public:
    using Inherit = GraphExport<MAP>;
    using Self = CgGraphExport<MAP>;
    using Map = typename Inherit::Map;
    using Vertex = typename Inherit::Vertex;
    using Edge = typename Inherit::Edge;
    using ChunkArrayGen = typename Inherit::ChunkArrayGen;
    template <typename T>
    using VertexAttribute = typename Inherit::template VertexAttribute<T>;
    using ChunkArrayContainer = typename Inherit::ChunkArrayContainer;

protected:
    virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& ) override
	{
		// Header
		output << "# D:3 NV:" << map.template nb_cells<Vertex::ORBIT>() << " NE:" << map.template nb_cells<Edge::ORBIT>() << std::endl;

		// Vertices
		map.foreach_cell([&] (Vertex v)
		{
			output << "v ";
			this->position_attribute(Vertex::ORBIT)->export_element(map.embedding(v), output, false, false);
			output << std::endl;
		}, *(this->cell_cache_));

		// Edges (index from 1)
		map.foreach_cell([&] (Edge e)
		{
			std::pair<Vertex, Vertex> vs = map.vertices(e);
			output << "e " << (this->vindices_[vs.first]+1u) << " " << (this->vindices_[vs.second]+1u) << std::endl;
		}, *(this->cell_cache_));
    }
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_FORMATS_CG_CPP_))
extern template class CGOGN_IO_API CgGraphImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API CgGraphImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API CgGraphImport<geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API CgGraphImport<geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API CgGraphExport<UndirectedGraph>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_FORMATS_CG_CPP_))


} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_FORMATS_CSKEL_H_

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

#ifndef CGOGN_IO_SOBJ_IO_H_
#define CGOGN_IO_SOBJ_IO_H_

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
///
///
template <typename VEC3>
class ObjGraphImport : public GraphFileImport<VEC3>
{

public:
	using Self = ObjGraphImport<VEC3>;
	using Inherit = GraphFileImport<VEC3>;
    using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
    template <typename T>
    using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline ObjGraphImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ObjGraphImport);
	virtual ~ObjGraphImport() override {}

protected:
    virtual bool import_file_impl(const std::string& filename) override
    {
        std::ifstream fp(filename.c_str(), std::ios::in);


		ChunkArray<VEC3>* position = this->position_attribute();
		ChunkArray<Scalar>* radius = this->radius_attribute();

		std::string line, tag;

		do
		{
			fp >> tag;
			getline_safe(fp, line);
		} while (tag != std::string("v"));

		// lecture des sommets
		std::vector<uint32> vertices_id;
		vertices_id.reserve(102400);

		uint32 i = 0;
		do
		{
			if (tag == std::string("v"))
			{
				std::stringstream oss(line);

				float64 x, y, z;
				oss >> x;
				oss >> y;
				oss >> z;

				VEC3 pos{Scalar(x), Scalar(y), Scalar(z)};

				uint32 vertex_id = this->insert_line_vertex_container();
				(*position)[vertex_id] = pos;
				(*radius)[vertex_id] = Scalar(0.001);

				vertices_id.push_back(vertex_id);
				i++;
			}

			fp >> tag;
			getline_safe(fp, line);
		} while (!fp.eof());

		fp.clear();
		fp.seekg(0, std::ios::beg);

		this->edges_nb_vertices_.reserve(vertices_id.size() * 2);

		do
		{
			fp >> tag;
			getline_safe(fp, line);
		} while (tag != std::string("l"));

		do
		{
			if (tag == std::string("l"))
			{
				std::stringstream oss(line);

				uint32  a, b;
				oss >> a;
				oss >> b;

				this->edges_nb_vertices_.push_back(2);
				this->edges_vertex_indices_.push_back(a);
				this->edges_vertex_indices_.push_back(b);
			}

			fp >> tag;
			getline_safe(fp, line);
		} while (!fp.eof());

		return true;
    }
};

template <typename MAP>
class ObjGraphExport : public GraphExport<MAP>
{
public:
    using Inherit = GraphExport<MAP>;
	using Self = ObjGraphExport<MAP>;
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
        const ChunkArrayGen* radius_attribute(nullptr);

        for(const ChunkArrayGen* vatt: this->vertex_attributes())
		{
            if(to_lower(vatt->name()) == "radius" || to_lower(vatt->name()) == "radii")
                radius_attribute = vatt;
		}

        // set precision for float output
        output << std::setprecision(12);

		map.foreach_cell([&] (Vertex v)
		{
			output << "v ";
			this->position_attribute(Vertex::ORBIT)->export_element(map.embedding(v), output, false, false);
			output << std::endl;
		}, *(this->cell_cache_));

		map.foreach_cell([&] (Edge e)
		{
			std::pair<Vertex, Vertex> vs = map.vertices(e);
			output << "l " << (this->vindices_[vs.first]+1u) << " " << (this->vindices_[vs.second]+1u) << std::endl;
		}, *(this->cell_cache_));
    }
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_SOBJ_IO_CPP_))
extern template class CGOGN_IO_API ObjGraphImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API ObjGraphImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API ObjGraphImport<geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API ObjGraphImport<geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API ObjGraphExport<UndirectedGraph>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_SOBJ_IO_CPP_))


} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_SOBJ_IO_H_

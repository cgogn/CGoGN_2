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

#ifndef CGOGN_IO_FORMATS_SKEL_H_
#define CGOGN_IO_FORMATS_SKEL_H_

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
/// livesu TVCG 2012
///
template <typename VEC3>
class SkelGraphImport : public GraphFileImport
{

public:
	using Self = SkelGraphImport<VEC3>;
	using Inherit = GraphFileImport;
    using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
    template <typename T>
    using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline SkelGraphImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(SkelGraphImport);
	virtual ~SkelGraphImport() override {}

protected:
    virtual bool import_file_impl(const std::string& filename) override
    {
        std::ifstream fp(filename.c_str(), std::ios::in);

        std::string line;
        line.reserve(512);

        // read SKEL header
        io::getline_safe(fp, line);
        if (line.rfind("ID") == std::string::npos)
        {
				cgogn_log_error("SkelGraphImport::import_file_impl") << "File \"" << filename << "\" is not a valid skel file.";
                return false;
        }

		std::set<std::pair<uint32,uint32>> unique_edges;

        // read number of vertices
        io::getline_safe(fp, line);
        const uint32 nb_vertices = uint32((std::stoul(line)));
        this->reserve(nb_vertices);

		ChunkArray<VEC3>* position = this->template add_vertex_attribute<VEC3>("position");
		ChunkArray<Scalar>* radius = this->template add_vertex_attribute<Scalar>("radius");

        std::vector<uint32> vertices_id;
        vertices_id.reserve(nb_vertices);

        std::vector<uint32> table;
        table.reserve(64);

        for (uint32 i = 0; i < nb_vertices; ++i)
        {
            io::getline_safe(fp, line);
            std::stringstream oss(line);

            uint32 id, nb_neighboors;
            float64 Cx, Cy, Cz, radi;

            oss >> id;
            oss >> Cx;
            oss >> Cy;
            oss >> Cz;
            oss >> radi;

            VEC3 pos{Scalar(Cx), Scalar(Cy), Scalar(Cz)};
            const uint32 vertex_id = this->insert_line_vertex_container();

            (*position)[vertex_id] = pos;
            (*radius)[vertex_id] = Scalar(radi);

            vertices_id.push_back(vertex_id);

            //neighborhood connectivity
            oss >> nb_neighboors;
            table.clear();
			uint32 index;
            for(uint32 j = 0u ; j < nb_neighboors ; ++j)
            {                
                oss >> index;				
                table.push_back(index);
            }

			for(uint32 j = 0u; j < nb_neighboors ; ++j)
				unique_edges.emplace(std::make_pair(std::min(id,table[j]), std::max(id, table[j])));
        }

		for(auto& it : unique_edges)
		{
			this->edges_nb_vertices_.push_back(2);
			this->edges_vertex_indices_.push_back(it.first);
			this->edges_vertex_indices_.push_back(it.second);
		}

        return true;
    }
};

template <typename MAP>
class SkelGraphExport : public GraphExport<MAP>
{
public:
    using Inherit = GraphExport<MAP>;
    using Self = SkelGraphExport<MAP>;
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
        output << "ID Cx Cy Cz RADIUS #NEIGHBORS NEIGHBORS_LIST" << std::endl;
        output << map.template nb_cells<Vertex::ORBIT>() << std::endl;

		std::vector<uint32> neighbors;
		neighbors.reserve(32);
		map.foreach_cell([&] (Vertex v)
		{
//			//output << map.embedding(v) << " ";
			output << this->vindices_[v] << " ";
			this->position_attribute(Vertex::ORBIT)->export_element(map.embedding(v), output, false, false);
			//output << " ";
			radius_attribute->export_element(map.embedding(v), output, false, false);
			neighbors.clear();
			map.foreach_adjacent_vertex_through_edge(v, [&] (Vertex va)
			{
				neighbors.push_back(this->vindices_[va]);
			});

			output << " " << neighbors.size();
			for(auto& n : neighbors)
				output << " " << n;

			output << std::endl;

		}, *(this->cell_cache_));
    }
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_FORMATS_SKELL_CPP_))
extern template class CGOGN_IO_API SkelGraphImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API SkelGraphImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API SkelGraphImport<geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API SkelGraphImport<geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API SkelGraphExport<UndirectedGraph>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_FORMATS_SKELL_CPP_))


} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_FORMATS_SKEL_H_

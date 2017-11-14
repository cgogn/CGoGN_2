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

#ifndef CGOGN_IO_FORMATS_CSKEL_H_
#define CGOGN_IO_FORMATS_CSKEL_H_

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
/// Dey and Sun 2006 (.skc)
///
/// OUTPUT FILES
///The output of the program is written in a file starting
///with "outfile_prefix" where the extracted curve-skeleton
///is stored. The format of the file is as follow.

///#vertices #edges
///x1 y1 z1
///x2 y2 z2
///...

///index1_of_edge1 index2_of_edge1
///geod
///eccentricity
///#touching_points index1 index2 index3

///index1_of_edge2 index2_of_edge2
///geod
///eccentricity
///#touching_points index1 index2 index3
///...

///First, the number of vertices and skeleton edges.
///Second, the coordinates of the vertices.
///Third, the information associated with each skeleton
///edge containing four parts. They are the indices of
///two endpoints, the length of the geodesic circle,
///the eccentricity value and the indices of the vertices
///of the dual Delaunay triangle in the original triangle
///mesh.
///
/// \todo import/export of the edge properties
template <typename VEC3>
class CskelGraphImport : public GraphFileImport
{

public:
	using Self = CskelGraphImport<VEC3>;
	using Inherit = GraphFileImport;
    using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
    template <typename T>
    using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline CskelGraphImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CskelGraphImport);
	virtual ~CskelGraphImport() override {}

protected:
    virtual bool import_file_impl(const std::string& filename) override
    {
 		std::ifstream fp(filename.c_str(), std::ios::in);

		ChunkArray<VEC3>* position = this->template add_vertex_attribute<VEC3>("position");
		ChunkArray<Scalar>* radius = this->template add_vertex_attribute<Scalar>("radius");

        std::string line;
        line.reserve(512);

        // reading number of points and number of edges
        uint32 nb_vertices = 0u, nb_edges = 0u;
        {
        	getline_safe(fp, line);
        	std::istringstream iss(line);
        	iss >> nb_vertices;
        	iss >> nb_edges;
        }
        this->reserve(nb_edges);

        // read points
        for(uint32 i = 0u; i < nb_vertices; ++i)
		{
			getline_safe(fp, line);
			std::istringstream oss(line);

			float64 Cx, Cy, Cz;
			oss >> Cx;
			oss >> Cy;
			oss >> Cz;

			VEC3 pos{Scalar(Cx), Scalar(Cy), Scalar(Cz)};
			const uint32 vertex_id = this->insert_line_vertex_container();

			(*position)[vertex_id] = pos;
			(*radius)[vertex_id] = Scalar(0.1);
        }

        // read connectivity
		for(uint32 i = 0u; i < nb_edges; ++i)
		{

			getline_safe(fp, line);
			int32 pA, pB;
			std::istringstream iss(line);
			iss >> pA;
			iss >> pB;

			this->edges_nb_vertices_.push_back(2);
			this->edges_vertex_indices_.push_back(pA);
			this->edges_vertex_indices_.push_back(pB);

			getline_safe(fp, line);
			getline_safe(fp, line);
			getline_safe(fp, line);
			getline_safe(fp, line);

		}

    	return true;
    }
};

template <typename MAP>
class CskelGraphExport : public GraphExport<MAP>
{
public:
    using Inherit = GraphExport<MAP>;
    using Self = CskelGraphExport<MAP>;
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
		output << map.template nb_cells<Vertex::ORBIT>() << " " << map.template nb_cells<Edge::ORBIT>() << std::endl;

		// Vertices
		map.foreach_cell([&] (Vertex v)
		{
			this->position_attribute(Vertex::ORBIT)->export_element(map.embedding(v), output, false, false);
			output << std::endl;
		}, *(this->cell_cache_));


		// Edges
		map.foreach_cell([&] (Edge e)
		{
			std::pair<Vertex, Vertex> vs = map.vertices(e);
			output << (this->vindices_[vs.first]) << " " << (this->vindices_[vs.second]) << std::endl;
			output << "0" << std::endl;
			output << "0" << std::endl;
			output << "0" << std::endl;
			output << "0" << std::endl;

		}, *(this->cell_cache_));
    }
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_FORMATS_CSKELL_CPP_))
extern template class CGOGN_IO_API CskelGraphImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API CskelGraphImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API CskelGraphImport<geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API CskelGraphImport<geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API CskelGraphExport<UndirectedGraph>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_FORMATS_CSKELL_CPP_))


} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_FORMATS_CSKEL_H_

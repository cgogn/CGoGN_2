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

#ifndef GEOMETRY_ALGOS_PICKING_H_
#define GEOMETRY_ALGOS_PICKING_H_

#include <core/utils/precision.h>
#include <core/basic/cell.h>

#include <geometry/algos/area.h>
#include <geometry/functions/basics.h>
#include <geometry/functions/intersection.h>
#include <geometry/functions/distance.h>
#include <geometry/types/geometry_traits.h>

namespace cgogn
{

namespace geometry
{

//template <typename VEC3, typename MAP>
//inline void picking_vertices(MAP& map, const typename MAP::template VertexAttributeHandler<VEC3>& position, const VEC3& A, const VEC3& B, typename VEC3::Scalar dist, typename std::vector<typename MAP::Vertex> selected)
//{
//	VEC3 AB = B - A ;
//	cgogn_message_assert(AB.squaredNorm()>0.0,"line must be defined by 2 different points");
//	AB.normalize();

//	selected.clear();
//	map.foreach_cell([&] (typename MAP::Vertex v)
//	{
//		if(squared_distance_normalized_line_point<VEC3>(A,AB,position[v])< dist*dist)
//			selected.push_back(v);
//	});
//}

template <typename VEC3, typename MAP>
inline void picking_face(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position, const VEC3& A, const VEC3& B, typename std::vector<typename MAP::Face> selected)
{
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;
	VEC3 AB = B - A ;
	cgogn_message_assert(AB.squaredNorm()>0.0,"line must be defined by 2 different points");
	AB.normalize();

	selected.clear();
	std::vector<unsigned int> ear_indices;
	ear_indices.reserve(256);

	m.foreach_cell([&] (Face f)
	{
		if (m.has_degree(f,3))
		{
			const VEC3& p1 = position[Vertex(f.dart)];
			const VEC3& p2 = position[Vertex(m.phi1(f.dart))];
			const VEC3& p3 = position[Vertex(m.phi1(m.phi1(f.dart)))];
			if (intersection_ray_triangle<VEC3>(A,AB,p1,p2,p3))
				selected.push_back(f);
		}
		else
		{
			cgogn::geometry::compute_ear_triangulation<VEC3>(m,f,position,ear_indices);
			for(unsigned int i=0; i<ear_indices.size(); i+=3)
			{
				const VEC3& p1 = position[ear_indices[i]];
				const VEC3& p2 = position[ear_indices[i+1]];
				const VEC3& p3 = position[ear_indices[i+2]];
				if (intersection_ray_triangle<VEC3>(A,AB,p1,p2,p3))
				{
					selected.push_back(f);
					i = ear_indices.size();
				}
			}
		}
	});
}


} // namespace geometry

} // namespace cgogn

#endif // GEOMETRY_ALGOS_PICKING_H_

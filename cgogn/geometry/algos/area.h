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

#ifndef GEOMETRY_ALGOS_AREA_H_
#define GEOMETRY_ALGOS_AREA_H_

#include <geometry/functions/area.h>
#include <geometry/algos/centroid.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3_T, typename MAP>
inline typename VEC3_T::Scalar triangle_area(const MAP& map, typename MAP::Face f, const typename MAP::template VertexAttribute<VEC3_T>& position)
{
	using Vertex = typename MAP::Vertex;
	return triangle_area<VEC3_T>(
		position[Vertex(f.dart)],
		position[Vertex(map.phi1(f.dart))],
		position[Vertex(map.phi_1(f.dart))]
	);
}

template <typename VEC3_T, typename MAP>
inline typename VEC3_T::Scalar convex_face_area(const MAP& map, typename MAP::Face f, const typename MAP::template VertexAttribute<VEC3_T>& position)
{
	using Vertex = typename MAP::Vertex;
	if (map.codegree(f) == 3)
		return triangle_area<VEC3_T>(map, f, position);
	else
	{
		typename VEC3_T::Scalar area{0};
		VEC3_T center = centroid<VEC3_T>(map, f, position);
		map.foreach_incident_edge(f, [&] (typename MAP::Edge e)
		{
			area += triangle_area<VEC3_T>(center, position[Vertex(e.dart)], position[Vertex(map.phi1(e.dart))]);
		});
		return area;
	}
}

} // namespace geometry

} // namespace cgogn

#endif // GEOMETRY_ALGOS_AREA_H_

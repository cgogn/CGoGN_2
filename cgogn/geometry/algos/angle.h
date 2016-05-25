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

#ifndef CGOGN_GEOMETRY_ALGOS_ANGLE_H_
#define CGOGN_GEOMETRY_ALGOS_ANGLE_H_

#include <cmath>

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/functions/basics.h>
#include <cgogn/geometry/algos/normal.h>

#include <cgogn/core/utils/masks.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3, typename MAP>
inline typename VEC3::Scalar angle_between_face_normals(
	const MAP& map,
	const typename MAP::Edge e,
	const typename MAP::template VertexAttribute<VEC3>& position)
{
	using Scalar = typename VEC3::Scalar;
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;

	if(map.is_incident_to_boundary(e))
		return Scalar(0);

	std::pair<Vertex, Vertex> v = map.vertices(e);
	const VEC3 n1 = face_normal<VEC3, MAP>(map, Face(v.first.dart), position);
	const VEC3 n2 = face_normal<VEC3, MAP>(map, Face(v.second.dart), position);

	return angle(n1, n2);
}

template <typename VEC3, typename MAP, typename MASK>
inline void compute_angle_between_face_normals(
	const MAP& map,
	const MASK& mask,
	const typename MAP::template VertexAttribute<VEC3>& position,
	typename MAP::template Attribute<typename VEC3::Scalar, Orbit::PHI2>& angles)
{
	map.parallel_foreach_cell([&] (Cell<Orbit::PHI2> e, uint32)
	{
		angles[e] = angle_between_face_normals<VEC3, MAP>(map, e, position);
	},
	mask);
}

template <typename VEC3, typename MAP>
inline void compute_angle_between_face_normals(
	const MAP& map,
	const typename MAP::template VertexAttribute<VEC3>& position,
	typename MAP::template Attribute<typename VEC3::Scalar, Orbit::PHI2>& angles)
{
	compute_angle_between_face_normals<VEC3>(map, CellFilters(), position, angles);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_ANGLE_H_

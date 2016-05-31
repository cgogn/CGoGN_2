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

/**
 * compute and return the angle formed by the normals of the two faces incident to the given edge
 */
template <typename VEC3, typename MAP>
inline typename vector_traits<VEC3>::Scalar angle_between_face_normals(
	const MAP& map,
	const Cell<Orbit::PHI2> e,
	const typename MAP::template VertexAttribute<VEC3>& position
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex2 = Cell<Orbit::PHI21>;
	using Face2 = Cell<Orbit::PHI1>;

	if (map.is_incident_to_boundary(e))
		return Scalar(0);

	const Dart d = e.dart;
	const Dart d2 = map.phi2(d);

	const VEC3 n1 = normal<VEC3>(map, Face2(d), position);
	const VEC3 n2 = normal<VEC3>(map, Face2(d2), position);

	VEC3 edge = position[Vertex2(d2)] - position[Vertex2(d)];
	edge.normalize();
	Scalar s = edge.dot(n1.cross(n2));
	Scalar c = n1.dot(n2);
	Scalar a(0);

	// the following trick is useful to avoid NaNs (due to floating point errors)
	if (c > 0.5) a = asin(s);
	else
	{
		if(c < -1) c = -1;
		if (s >= 0) a = acos(c);
		else a = -acos(c);
	}
	if(a != a)
		cgogn_log_warning("angle_between_face_normals") << "NaN computed";

	return a;
}

template <typename VEC3, typename MAP, typename MASK>
inline void compute_angle_between_face_normals(
	const MAP& map,
	const MASK& mask,
	const typename MAP::template VertexAttribute<VEC3>& position,
	typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI2>& edge_angle
)
{
	map.parallel_foreach_cell([&] (Cell<Orbit::PHI2> e, uint32)
	{
		edge_angle[e] = angle_between_face_normals<VEC3>(map, e, position);
	},
	mask);
}

template <typename VEC3, typename MAP>
inline void compute_angle_between_face_normals(
	const MAP& map,
	const typename MAP::template VertexAttribute<VEC3>& position,
	typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI2>& edge_angle
)
{
	compute_angle_between_face_normals<VEC3>(map, CellFilters(), position, edge_angle);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_ANGLE_H_

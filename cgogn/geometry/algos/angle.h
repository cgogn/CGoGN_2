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
#include <cgogn/core/cmap/cmap2.h>

namespace cgogn
{

namespace geometry
{

/**
* @brief angle at vertex in a face
* @param map
* @param v
* @param position vertex attribute of position
* @return
*/
template <typename MAP, typename VERTEX_ATTR>
inline ScalarOf<InsideTypeOf<VERTEX_ATTR>> angle(const MAP& map, const Cell<Orbit::DART> v, const VERTEX_ATTR& position)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value, "position must be a vertex attribute");

	using VEC3 = InsideTypeOf<VERTEX_ATTR>;
	using Vertex = typename MAP::Vertex;

	const VEC3& p = position[Vertex(v.dart)];
	VEC3 v1 = position[Vertex(map.phi1(v.dart))] - p;
	VEC3 v2 = position[Vertex(map.phi_1(v.dart))] - p;
	return angle(v1, v2);
}


/**
 * @brief compute and return the angle formed by the normals of the two faces incident to the given edge
 * @param map
 * @param e edge
 * @param position vertex attribute of position position attribute
 * @return
 */
template <typename MAP, typename VERTEX_ATTR>
inline ScalarOf<InsideTypeOf<VERTEX_ATTR>> angle_between_face_normals(
	const MAP& map,
	const Cell<Orbit::PHI2> e,
	const VERTEX_ATTR& position)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value, "position must be a vertex attribute");

	using VEC3 = InsideTypeOf<VERTEX_ATTR>;
	using Scalar = ScalarOf<VEC3>;
	using Vertex2 = Cell<Orbit::PHI21>;
	using Face2 = Cell<Orbit::PHI1>;

	if (map.is_incident_to_boundary(e))
		return Scalar(0);

	const Dart d = e.dart;
	const Dart d2 = map.phi2(d);

    const VEC3 n1 = normal(map, Face2(d), position);
    const VEC3 n2 = normal(map, Face2(d2), position);

	VEC3 edge = position[Vertex2(d2)] - position[Vertex2(d)];
	edge.normalize();
	Scalar s = edge.dot(n1.cross(n2));
	Scalar c = n1.dot(n2);
	Scalar a(0);

	// the following trick is useful to avoid NaNs (due to floating point errors)
	if (c > Scalar(0.5)) a = std::asin(s);
	else
	{
		if(c < -1) c = -1;
		if (s >= 0) a = std::acos(c);
		else a = -std::acos(c);
	}
	if (a != a)
		cgogn_log_warning("angle_between_face_normals") << "NaN computed";

	return a;
}


/**
 * @brief compute angle between face normals incident to each edge of a part of the map
 * @param map
 * @param mask
 * @param position vertex attribute of position
 * @param edge_angle attribute to store angles
 */
template <typename MAP, typename MASK, typename VERTEX_ATTR>
inline void compute_angle_between_face_normals(
	const MAP& map,
	const MASK& mask,
	const VERTEX_ATTR& position,
	Attribute<ScalarOf<InsideTypeOf<VERTEX_ATTR>>, Orbit::PHI2>& edge_angle
)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value, "position must be a vertex attribute");

	map.parallel_foreach_cell([&] (Cell<Orbit::PHI2> e)
	{
		edge_angle[e] = angle_between_face_normals(map, e, position);
	},
	mask);
}

/**
 * @brief compute angle between face normals incident to each edge of the map
 * @param map
 * @param position vertex attribute of position
 * @param edge_angle edge attribute to store angles
 */
template <typename MAP, typename VERTEX_ATTR>
inline void compute_angle_between_face_normals(
	const MAP& map,
	const VERTEX_ATTR& position,
	Attribute<ScalarOf<InsideTypeOf<VERTEX_ATTR>>, Orbit::PHI2>& edge_angle
)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value, "position must be a vertex attribute");

	compute_angle_between_face_normals(map, AllCellsFilter(), position, edge_angle);
}



#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_EXTERNAL_TEMPLATES_CPP_))
extern template CGOGN_GEOMETRY_API float32 angle_between_face_normals(const CMap2&, const Cell<Orbit::PHI2>, const CMap2::VertexAttribute<Eigen::Vector3f>&);
extern template CGOGN_GEOMETRY_API float64 angle_between_face_normals(const CMap2&, const Cell<Orbit::PHI2>, const CMap2::VertexAttribute<Eigen::Vector3d>&);
extern template CGOGN_GEOMETRY_API void compute_angle_between_face_normals(const CMap2&, const CMap2::VertexAttribute<Eigen::Vector3f>&, Attribute<float32, Orbit::PHI2>&);
extern template CGOGN_GEOMETRY_API void compute_angle_between_face_normals(const CMap2&, const CMap2::VertexAttribute<Eigen::Vector3d>&, Attribute<float64, Orbit::PHI2>&);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_EXTERNAL_TEMPLATES_CPP_))

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_ANGLE_H_


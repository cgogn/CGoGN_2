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

#ifndef CGOGN_GEOMETRY_ALGOS_NORMAL_H_
#define CGOGN_GEOMETRY_ALGOS_NORMAL_H_

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/basic/cell.h>

#include <cgogn/geometry/algos/area.h>
#include <cgogn/geometry/functions/basics.h>
#include <cgogn/geometry/functions/normal.h>
#include <cgogn/geometry/types/geometry_traits.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3, typename MAP>
inline VEC3 triangle_normal(const MAP& map, Cell<Orbit::PHI1> f, const typename MAP::template VertexAttribute<VEC3>& position)
{
	using Vertex = typename MAP::Vertex;
	VEC3 n = triangle_normal<VEC3>(
		position[Vertex(f.dart)],
		position[Vertex(map.phi1(f.dart))],
		position[Vertex(map.phi_1(f.dart))]
	);
	normalize_safe(n);
	return n;
}

template <typename VEC3, typename MAP>
inline VEC3 newell_normal(const MAP& map, Cell<Orbit::PHI1> f, const typename MAP::template VertexAttribute<VEC3>& position)
{
	using Vertex = typename MAP::Vertex;
	using Scalar = typename cgogn::geometry::vector_traits<VEC3>::Scalar;
	VEC3 n{Scalar(0), Scalar(0), Scalar(0)};
	map.foreach_incident_vertex(f, [&] (Cell<Orbit::PHI21> v)
	{
		const VEC3& p = position[Vertex(v.dart)];
		const VEC3& q = position[Vertex(map.phi1(v.dart))];
		n[0] += (p[1] - q[1]) * (p[2] + q[2]);
		n[1] += (p[2] - q[2]) * (p[0] + q[0]);
		n[2] += (p[0] - q[0]) * (p[1] + q[1]);
	});
	normalize_safe(n);
	return n;
}

template <typename VEC3, typename MAP>
inline VEC3 face_normal(const MAP& map, Cell<Orbit::PHI1> f, const typename MAP::template VertexAttribute<VEC3>& position)
{
	if (map.has_codegree(f, 3))
		return triangle_normal<VEC3>(map, f, position);
	else
		return newell_normal<VEC3>(map, f, position);
}

template <typename VEC3, typename MAP>
inline VEC3 vertex_normal(const MAP& map, Cell<Orbit::PHI21> v, const typename MAP::template VertexAttribute<VEC3>& position)
{
	using Vertex = typename MAP::Vertex;
	using Scalar = typename VEC3::Scalar;

	VEC3 n{Scalar{0}, Scalar{0}, Scalar{0}};
	const VEC3& p = position[Vertex(v.dart)];
	map.foreach_incident_face(v, [&] (Cell<Orbit::PHI1> f)
	{
		VEC3 facen = face_normal<VEC3>(map, f, position);
		const VEC3& p1 = position[Vertex(map.phi1(f.dart))];
		const VEC3& p2 = position[Vertex(map.phi_1(f.dart))];
		const Scalar l = (p1-p).squaredNorm() * (p2-p).squaredNorm();
		if (l != Scalar(0))
			facen *= convex_face_area<VEC3>(map, f, position) / l;
		n += facen;
	});
	normalize_safe(n);
	return n;
}

template <typename VEC3, typename MAP>
inline VEC3 vertex_normal(const MAP& map, Cell<Orbit::PHI21> v, const typename MAP::template VertexAttribute<VEC3>& position, const typename MAP::template Attribute<VEC3, Orbit::PHI1>& fnormal)
{
	using Vertex = typename MAP::Vertex;
	using Scalar = typename VEC3::Scalar;

	VEC3 n{Scalar{0}, Scalar{0} ,Scalar{0}};
	const VEC3& p = position[Vertex(v.dart)];
	map.foreach_incident_face(v, [&] (Cell<Orbit::PHI1> f)
	{
		VEC3 facen = fnormal[f];
		const VEC3& p1 = position[Vertex(map.phi1(f.dart))];
		const VEC3& p2 = position[Vertex(map.phi_1(f.dart))];
		const Scalar l = (p1-p).squaredNorm() * (p2-p).squaredNorm();
		if (l != Scalar(0))
			facen *= convex_face_area<VEC3>(map, f, position) / l;
		n += facen;
	});
	normalize_safe(n);
	return n;
}

template <typename VEC3, typename MAP>
inline void compute_normal_faces(const MAP& map, const typename MAP::template VertexAttribute<VEC3>& position, typename MAP::template Attribute<VEC3, Orbit::PHI1>& normal)
{
	map.parallel_foreach_cell([&] (Cell<Orbit::PHI1> f, uint32)
	{
		normal[f] = face_normal<VEC3>(map, f, position);
	});
}

template <typename VEC3, typename MAP>
inline void compute_normal_vertices(const MAP& map, const typename MAP::template VertexAttribute<VEC3>& position, typename MAP::template Attribute<VEC3, Orbit::PHI21>& normal)
{
	map.parallel_foreach_cell([&] (Cell<Orbit::PHI21> v, uint32)
	{
		normal[v] = vertex_normal<VEC3>(map, v, position);
	});
}

template <typename VEC3, typename MAP>
inline void compute_normal_vertices(const MAP& map, const typename MAP::template VertexAttribute<VEC3>& position, const typename MAP::template Attribute<VEC3, Orbit::PHI1>& fnormal, typename MAP::template Attribute<VEC3, Orbit::PHI21>& normal)
{
	map.parallel_foreach_cell([&] (Cell<Orbit::PHI21> v, uint32)
	{
		normal[v] = vertex_normal<VEC3>(map, v, position, fnormal);
	});
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_NORMAL_H_

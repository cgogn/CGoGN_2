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

#ifndef GEOMETRY_ALGOS_NORMAL_H_
#define GEOMETRY_ALGOS_NORMAL_H_

#include <core/basic/cell.h>
#include <geometry/algos/area.h>
#include <geometry/functions/normal.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3_T, typename MAP>
inline VEC3_T triangle_normal(MAP& map, Cell<Orbit::PHI1> f, const typename MAP::template VertexAttributeHandler<VEC3_T>& position)
{
	VEC3_T n = triangle_normal<VEC3_T>(
		position[f.dart],
		position[map.phi1(f.dart)],
		position[map.phi_1(f.dart)]
	);
	n.normalize();
	return n;
}

template <typename VEC3_T, typename MAP>
inline VEC3_T newell_normal(MAP& map, Cell<Orbit::PHI1> f, const typename MAP::template VertexAttributeHandler<VEC3_T>& position)
{
	VEC3_T n{0,0,0};
	map.foreach_incident_vertex(f, [&] (Cell<Orbit::PHI21> v)
	{
		const VEC3_T& p = position[v.dart];
		const VEC3_T& q = position[map.phi1(v.dart)];
		n[0] += (p[1] - q[1]) * (p[2] + q[2]);
		n[1] += (p[2] - q[2]) * (p[0] + q[0]);
		n[2] += (p[0] - q[0]) * (p[1] + q[1]);
	});
	n.normalize();
	return n;
}

template <typename VEC3_T, typename MAP>
inline VEC3_T face_normal(MAP& map, Cell<Orbit::PHI1> f, const typename MAP::template VertexAttributeHandler<VEC3_T>& position)
{
	if (map.degree(f) == 3)
		return triangle_normal<VEC3_T>(map, f, position);
	else
		return newell_normal<VEC3_T>(map, f, position);
}

template <typename VEC3_T, typename MAP>
inline VEC3_T vertex_normal(MAP& map, Cell<Orbit::PHI21> v, const typename MAP::template VertexAttributeHandler<VEC3_T>& position)
{
	VEC3_T n{0,0,0};
	const VEC3_T& p = position[v.dart];
	map.foreach_incident_face(v, [&] (Cell<Orbit::PHI1> f)
	{
		VEC3_T facen = face_normal<VEC3_T>(map, f, position);
		const VEC3_T& p1 = position[map.phi1(f.dart)];
		const VEC3_T& p2 = position[map.phi_1(f.dart)];
		typename VEC3_T::Scalar l = (p1-p).squaredNorm() * (p2-p).squaredNorm();
		facen *= convex_face_area<VEC3_T>(map, f, position) / l;
		n += facen;
	});
	n.normalize();
	return n;
}

template <typename VEC3_T, typename MAP>
inline VEC3_T vertex_normal(MAP& map, Cell<Orbit::PHI21> v, const typename MAP::template VertexAttributeHandler<VEC3_T>& position, const typename MAP::template AttributeHandler<VEC3_T, Orbit::PHI1>& fnormal)
{
	VEC3_T n{0,0,0};
	const VEC3_T& p = position[v.dart];
	map.foreach_incident_face(v, [&] (Cell<Orbit::PHI1> f)
	{
		VEC3_T facen = fnormal[f];
		const VEC3_T& p1 = position[map.phi1(f.dart)];
		const VEC3_T& p2 = position[map.phi_1(f.dart)];
		typename VEC3_T::Scalar l = (p1-p).squaredNorm() * (p2-p).squaredNorm();
		facen *= convex_face_area<VEC3_T>(map, f, position) / l;
		n += facen;
	});
	n.normalize();
	return n;
}

template <typename VEC3_T, typename MAP>
inline void compute_normal_faces(MAP& map, const typename MAP::template VertexAttributeHandler<VEC3_T>& position, typename MAP::template AttributeHandler<VEC3_T, Orbit::PHI1>& normal)
{
	map.template foreach_cell<Orbit::PHI1>([&] (Cell<Orbit::PHI1> f)
	{
		normal[f] = face_normal<VEC3_T>(map, f, position);
	});
}

template <typename VEC3_T, typename MAP>
inline void compute_normal_vertices(MAP& map, const typename MAP::template VertexAttributeHandler<VEC3_T>& position, typename MAP::template AttributeHandler<VEC3_T, Orbit::PHI21>& normal)
{
	map.template foreach_cell<Orbit::PHI21>([&] (Cell<Orbit::PHI21> v)
	{
		normal[v] = vertex_normal<VEC3_T>(map, v, position);
	});
}

template <typename VEC3_T, typename MAP>
inline void compute_normal_vertices(MAP& map, const typename MAP::template VertexAttributeHandler<VEC3_T>& position, const typename MAP::template AttributeHandler<VEC3_T, Orbit::PHI1>& fnormal, typename MAP::template AttributeHandler<VEC3_T, Orbit::PHI21>& normal)
{
	map.template foreach_cell<Orbit::PHI21>([&] (Cell<Orbit::PHI21> v)
	{
		normal[v] = vertex_normal<VEC3_T>(map, v, position, fnormal);
	});
}

} // namespace geometry

} // namespace cgogn

#endif // GEOMETRY_ALGOS_NORMAL_H_

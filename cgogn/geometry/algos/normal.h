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
#include <cgogn/core/cmap/attribute.h>
#include <cgogn/core/utils/masks.h>

#include <cgogn/geometry/algos/area.h>
#include <cgogn/geometry/functions/basics.h>
#include <cgogn/geometry/functions/normal.h>
#include <cgogn/geometry/types/geometry_traits.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3, typename MAP>
inline VEC3 normal(
	const MAP& map,
	Cell<Orbit::PHI1> f,
	const typename MAP::template VertexAttribute<VEC3>& position
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;

	if (map.codegree(f) == 3)
	{
		VEC3 n = normal(
			position[Vertex(f.dart)],
			position[Vertex(map.phi1(f.dart))],
			position[Vertex(map.phi_1(f.dart))]
		);
		normalize_safe(n);
		return n;
	}
	else
	{
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
}

template <typename VEC3, typename MAP>
inline VEC3 normal(
	const MAP& map,
	Cell<Orbit::PHI21> v,
	const typename MAP::template VertexAttribute<VEC3>& position
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;

	VEC3 n{Scalar{0}, Scalar{0}, Scalar{0}};
	const VEC3& p = position[Vertex(v.dart)];
	map.foreach_incident_face(v, [&] (Cell<Orbit::PHI1> f)
	{
		VEC3 face_normal = normal<VEC3>(map, f, position);
		const VEC3& p1 = position[Vertex(map.phi1(f.dart))];
		const VEC3& p2 = position[Vertex(map.phi_1(f.dart))];
		const Scalar l = (p1-p).squaredNorm() * (p2-p).squaredNorm();
		if (l != Scalar(0))
			face_normal *= convex_area<VEC3>(map, f, position) / l;
		n += face_normal;
	});
	normalize_safe(n);
	return n;
}

template <typename VEC3, typename MAP>
inline VEC3 normal(
	const MAP& map,
	Cell<Orbit::PHI21> v,
	const typename MAP::template VertexAttribute<VEC3>& position,
	const Attribute<VEC3, Orbit::PHI1>& face_normal
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;

	VEC3 n{Scalar{0}, Scalar{0}, Scalar{0}};
	const VEC3& p = position[Vertex(v.dart)];
	map.foreach_incident_face(v, [&] (Cell<Orbit::PHI1> f)
	{
		VEC3 facen = face_normal[f];
		const VEC3& p1 = position[Vertex(map.phi1(f.dart))];
		const VEC3& p2 = position[Vertex(map.phi_1(f.dart))];
		const Scalar l = (p1-p).squaredNorm() * (p2-p).squaredNorm();
		if (l != Scalar(0))
			facen *= convex_area<VEC3>(map, f, position) / l;
		n += facen;
	});
	normalize_safe(n);
	return n;
}

template <typename VEC3, typename MAP, typename MASK>
inline void compute_normal(
	const MAP& map,
	const MASK& mask,
	const typename MAP::template VertexAttribute<VEC3>& position,
	Attribute<VEC3, Orbit::PHI1>& face_normal
)
{
	map.parallel_foreach_cell([&] (Cell<Orbit::PHI1> f, uint32)
	{
		face_normal[f] = normal<VEC3>(map, f, position);
	},
	mask);
}

template <typename VEC3, typename MAP>
inline void compute_normal(
	const MAP& map,
	const typename MAP::template VertexAttribute<VEC3>& position,
	Attribute<VEC3, Orbit::PHI1>& face_normal
)
{
	compute_normal<VEC3>(map, AllCellsFilter(), position, face_normal);
}

template <typename VEC3, typename MAP, typename MASK>
inline void compute_normal(
	const MAP& map,
	const MASK& mask,
	const typename MAP::template VertexAttribute<VEC3>& position,
	Attribute<VEC3, Orbit::PHI21>& vertex_normal
)
{
	map.parallel_foreach_cell([&] (Cell<Orbit::PHI21> v, uint32)
	{
		vertex_normal[v] = normal<VEC3>(map, v, position);
	},
	mask);
}

template <typename VEC3, typename MAP>
inline void compute_normal(
	const MAP& map,
	const typename MAP::template VertexAttribute<VEC3>& position,
	Attribute<VEC3, Orbit::PHI21>& vertex_normal
)
{
	compute_normal<VEC3>(map, AllCellsFilter(), position, vertex_normal);
}

template <typename VEC3, typename MAP, typename MASK>
inline void compute_normal(
	const MAP& map,
	const MASK& mask,
	const typename MAP::template VertexAttribute<VEC3>& position,
	const Attribute<VEC3, Orbit::PHI1>& face_normal,
	Attribute<VEC3, Orbit::PHI21>& vertex_normal
)
{
	map.parallel_foreach_cell([&] (Cell<Orbit::PHI21> v, uint32)
	{
		vertex_normal[v] = normal<VEC3>(map, v, position, face_normal);
	},
	mask);
}

template <typename VEC3, typename MAP>
inline void compute_normal(
	const MAP& map,
	const typename MAP::template VertexAttribute<VEC3>& position,
	const Attribute<VEC3, Orbit::PHI1>& face_normal,
	Attribute<VEC3, Orbit::PHI21>& vertex_normal
)
{
	compute_normal<VEC3>(map, AllCellsFilter(), position, face_normal, vertex_normal);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_NORMAL_H_

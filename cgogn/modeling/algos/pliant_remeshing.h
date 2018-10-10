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

#ifndef CGOGN_MODELING_ALGOS_PLIANT_REMESHING_H_
#define CGOGN_MODELING_ALGOS_PLIANT_REMESHING_H_

#include <cgogn/modeling/dll.h>

#include <cgogn/geometry/functions/basics.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/algos/length.h>
#include <cgogn/geometry/algos/centroid.h>
#include <cgogn/geometry/algos/normal.h>

#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/core/utils/masks.h>

namespace cgogn
{

namespace modeling
{

template <typename VERTEX_ATTR>
void pliant_remeshing(
	CMap2& map,
	VERTEX_ATTR& position
)
{
	static_assert(is_orbit_of<VERTEX_ATTR, CMap2::Vertex::ORBIT>::value,"position must be a vertex attribute");

	using VEC3 = InsideTypeOf<VERTEX_ATTR>;
	using Scalar = geometry::ScalarOf<VEC3>;

	using Vertex = typename CMap2::Vertex;
	using Edge = typename CMap2::Edge;

	Scalar mean_edge_length = geometry::mean_edge_length(map, position);

	const Scalar squared_min_edge_length = Scalar(0.5625) * mean_edge_length * mean_edge_length; // 0.5625 = 0.75^2
	const Scalar squared_max_edge_length = Scalar(1.5625) * mean_edge_length * mean_edge_length; // 1.5625 = 1.25^2

	CMap2::CellCache cache(map);

//	for (uint32 i = 0; i < 3; ++i)
//	{

	// cut long edges (and adjacent faces)
	auto long_edges_selection = [&] (Edge e) {
		std::pair<Vertex, Vertex> v = map.vertices(e);
		return (position[v.first] - position[v.second]).squaredNorm() > squared_max_edge_length;
	};

	cache.template build<Edge>(long_edges_selection);
	while (cache.template size<Edge>() > 0)
	{
		map.foreach_cell([&] (Edge e)
		{
			Dart e2 = map.phi2(e.dart);
			Vertex nv = map.cut_edge(e);
			position[nv] = Scalar(0.5) * (position[Vertex(e.dart)] + position[Vertex(e2)]);
			map.cut_face(map.phi1(e.dart), map.phi_1(e.dart));
			if (!map.is_boundary(e2))
				map.cut_face(map.phi1(e2), map.phi_1(e2));
		},
		cache);
		cache.template build<Edge>(long_edges_selection);
	}

	// collapse short edges
	auto short_edges_selection = [&] (Edge e)
	{
		std::pair<Vertex, Vertex> v = map.vertices(e);
		bool collapse = (position[v.first] - position[v.second]).squaredNorm() < squared_min_edge_length;
		if (collapse)
		{
			const VEC3& p = position[v.first];
			map.foreach_adjacent_vertex_through_edge(v.second, [&] (Vertex vv)
			{
				const VEC3& vec = p - position[vv];
				if (vec.squaredNorm() > squared_max_edge_length)
					collapse = false;
			});
			if (collapse)
			{
				if (!map.edge_can_collapse(e))
					collapse = false;
			}
		}
		return collapse;
	};

	cache.template build<Edge>(short_edges_selection);
	while (cache.template size<Edge>() > 0)
	{
		map.foreach_cell([&] (Edge e)
		{
			std::pair<Vertex,Vertex> v = map.vertices(e);
			bool collapse = true;
			const VEC3& p = position[v.first];
			map.foreach_adjacent_vertex_through_edge(v.second, [&] (Vertex vv)
			{
				const VEC3& vec = p - position[vv];
				if (vec.squaredNorm() > squared_max_edge_length)
					collapse = false;
			});
			if (collapse)
			{
				if (map.edge_can_collapse(e))
				{
					Vertex cv = map.collapse_edge(e);
					position[cv] = p;
				}
			}
		},
		cache);
		cache.template build<Edge>(short_edges_selection);
	}

	// equalize valences with edge flips
	typename CMap2::DartMarker dm(map);
	map.foreach_cell(
		[&] (Edge e)
		{
			map.flip_edge(e); // flip edge
			const Dart d = e.dart;
			const Dart d2 = map.phi2(d);
			dm.mark_orbit(Edge(map.phi1(d)));
			dm.mark_orbit(Edge(map.phi_1(d))); // mark adjacent
			dm.mark_orbit(Edge(map.phi1(d2))); // edges
			dm.mark_orbit(Edge(map.phi_1(d2)));
		},
		// this filter only keeps edges that are not marked
		// and whose incident vertices' degree meet some requirements
		[&] (Edge e) -> bool
		{
			if (dm.is_marked(e.dart))
				return false;
			std::pair<Vertex,Vertex> v = map.vertices(e);
			const uint32 w = map.degree(v.first);
			const uint32 x = map.degree(v.second);
			const uint32 y = map.degree(Vertex(map.phi1(map.phi1(v.first.dart))));
			const uint32 z = map.degree(Vertex(map.phi1(map.phi1(v.second.dart))));
			int32 flip = 0;
			flip += w > 6 ? 1 : (w < 6 ? -1 : 0);
			flip += x > 6 ? 1 : (x < 6 ? -1 : 0);
			flip += y < 6 ? 1 : (y > 6 ? -1 : 0);
			flip += z < 6 ? 1 : (z > 6 ? -1 : 0);
			return flip > 1;
		}
	);

	// tangential relaxation
	map.foreach_cell([&] (Vertex v)
	{
		VEC3 c(0,0,0);
		uint32 count = 0;
		map.foreach_adjacent_vertex_through_edge(v, [&] (Vertex av)
		{
			c += position[av];
			++count;
		});
		c /= Scalar(count);
		VEC3 n = geometry::normal(map, v, position);
		position[v] = c + n.dot(position[v] - c) * n;
	});

//	}
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_EXTERNAL_TEMPLATES_CPP_))
extern template CGOGN_MODELING_API void pliant_remeshing(CMap2&, CMap2::VertexAttribute<Eigen::Vector3f>&);
extern template CGOGN_MODELING_API void pliant_remeshing(CMap2&, CMap2::VertexAttribute<Eigen::Vector3d>&);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_EXTERNAL_TEMPLATES_CPP_))

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_PLIANT_REMESHING_H_

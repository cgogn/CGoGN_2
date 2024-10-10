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

#ifndef CGOGN_MODELING_ALGOS_CATMULL_CLARK_H_
#define CGOGN_MODELING_ALGOS_CATMULL_CLARK_H_

#include <vector>

#include <cgogn/modeling/cgogn_modeling_export.h>
#include <cgogn/core/basic/dart_marker.h>
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/geometry/algos/centroid.h>
#include <cgogn/geometry/types/geometry_traits.h>

namespace cgogn
{

namespace modeling
{

template <typename MAP>
typename MAP::Vertex quadrangule_face(MAP& map, typename MAP::Face f)
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;

	Dart d1 = map.phi1(f.dart);
	Dart d2 = map.phi1(map.phi1(d1));

	map.cut_face(d1, d2);

	map.cut_edge(Edge(map.phi_1(d1)));

	Dart x = map.phi2(map.phi_1(d1));
	Dart dd = map.template phi<1111>(x);
	while (dd != x)
	{
		Dart next = map.template phi<11>(dd);
		map.cut_face(dd, map.phi1(x));
		dd = next ;
	}

	return Vertex(map.phi2(x));	// Return a dart of the central vertex
}

template < typename MAP, typename VERTEX_ATTR>
void catmull_clark(MAP& map, VERTEX_ATTR& position)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value,"position must be a vertex attribute");

	using VEC3 = InsideTypeOf<VERTEX_ATTR>;
	using Scalar = geometry::ScalarOf<VEC3>;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	DartMarker<MAP> initial_edge_marker(map);

	CellCache<MAP> initial_cache(map);
	initial_cache.template build<Vertex>();
	initial_cache.template build<Edge>();
	initial_cache.template build<Face>();

	map.foreach_cell([&] (Edge e) {	initial_edge_marker.mark_orbit(e); }, initial_cache);

	map.foreach_cell([&] (Edge e)
	{
		std::pair<Vertex,Vertex> ve = map.vertices(e);
		Vertex middle = map.cut_edge(e);
		position[middle] = (position[ve.first] + position[ve.second]) / Scalar(2);
	}
	, initial_cache);

	map.foreach_cell([&] (Face f)
	{
		Face ff = f;
		if (!initial_edge_marker.is_marked(f.dart))
			ff = Face(map.phi1(f.dart));

		initial_edge_marker.unmark_orbit(ff);

		VEC3 center = geometry::centroid(map, ff, position);
		Vertex vc = quadrangule_face(map, ff);
		position[vc] = center;
	}
	, initial_cache);

	map.foreach_cell([&] (Edge e)
	{
		Dart e2 = map.phi2(e.dart);
		if (!map.is_boundary(e.dart) && !map.is_boundary(e2))
		{
			Vertex f1(map.template phi<11>(e.dart));
			Vertex f2(map.phi_1(e2));
			Vertex m(map.phi1(e.dart));
			position[m] = (Scalar(2) * position[m] + position[f1] + position[f2]) / Scalar(4);
		}
	}
	, initial_cache);

	map.foreach_cell([&] (Vertex v)
	{
		VEC3 sum_face; // Sum_F
		sum_face.setZero();
		VEC3 sum_edge;// Sum_E
		sum_edge.setZero();

		int nb_f = 0;
		int nb_e = 0;
		Dart bound;
		map.foreach_incident_edge(v, [&] (Edge e)
		{
			nb_e++;
			sum_edge += position[Vertex(map.phi1(e.dart))];
			if (!map.is_boundary(e.dart))
			{
				nb_f++;
				sum_face += position[Vertex(map.template phi<11>(e.dart))];
			}
			else
				bound = e.dart;
		});

		if (nb_f > nb_e) // boundary case
		{
			Vertex e1(map.phi2(bound));
			Vertex e2(map.phi_1(bound));
			position[v] = Scalar(3.0/8.0) * position[v] + Scalar(1.0/4.0) * (position[e1] + position[e2]);
		}
		else
		{
			VEC3 delta = position[v] * Scalar(-3*nb_f);
			delta += sum_face + Scalar(2) * sum_edge;
			delta /= Scalar(nb_f * nb_f);
			position[v] += delta;
		}
	}
	, initial_cache);
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_EXTERNAL_TEMPLATES_CPP_))
extern template CGOGN_MODELING_EXPORT CMap2::Vertex quadrangule_face<CMap2>(CMap2&, CMap2::Face);
extern template CGOGN_MODELING_EXPORT CMap3::Vertex quadrangule_face<CMap3>(CMap3&, CMap3::Face);
extern template CGOGN_MODELING_EXPORT void catmull_clark(CMap2&, CMap2::VertexAttribute<Eigen::Vector3f>&);
extern template CGOGN_MODELING_EXPORT void catmull_clark(CMap2&, CMap2::VertexAttribute<Eigen::Vector3d>&);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_EXTERNAL_TEMPLATES_CPP_))

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_CATMULL_CLARK_H_

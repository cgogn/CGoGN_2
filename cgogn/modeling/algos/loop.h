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

#ifndef CGOGN_MODELING_ALGOS_LOOP_H_
#define CGOGN_MODELING_ALGOS_LOOP_H_

#include <vector>

#include <cgogn/modeling/dll.h>
#include <cgogn/core/basic/dart_marker.h>
#include <cgogn/core/utils/masks.h>
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/geometry/types/geometry_traits.h>

namespace cgogn
{

namespace modeling
{

template <typename VEC3, typename MAP>
void loop(MAP& map, typename MAP::template VertexAttribute<VEC3>& position)
{
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	typename MAP::template VertexAttribute<VEC3> position2 = map.template add_attribute<VEC3, Vertex>("position_tempo_loop");

	DartMarker<MAP> initial_edge_marker(map);

	CellCache<MAP> initial_cache(map);
	initial_cache.template build<Vertex>();
	initial_cache.template build<Edge>();
	initial_cache.template build<Face>();

	map.foreach_cell([&] (Edge e) {	initial_edge_marker.mark_orbit(e); }, initial_cache);

	// cut edges
	map.foreach_cell([&] (Edge e) { map.cut_edge(e); }, initial_cache);

	// compute position of new edge points
	map.foreach_cell([&] (Edge e)
	{
		Vertex v1(e.dart);
		Vertex v2(map.template phi<12>(e.dart));
		Vertex nve(map.phi1(e.dart));
		Vertex vr(map.template phi<2111>(e.dart));
		Vertex vl(map.phi_1(map.phi_1(e.dart)));

		position2[nve] = Scalar(3.0/8.0) * (position[v1] + position[v2]) + Scalar(1.0/8.0) * (position[vr] + position[vl]);
	}
	, initial_cache);

	// compute new position of old vertices
	map.foreach_cell([&] (Vertex v)
	{
		VEC3 sum_edge;// Sum_E
		sum_edge.setZero();

		int nb_e = 0;
		Dart bound;
		map.foreach_incident_edge(v, [&] (Edge e)
		{
			nb_e++;
			sum_edge += position[Vertex(map.template phi<12>(e.dart))];
			if (map.is_boundary(e.dart))
				bound = e.dart;
		});

		if (!bound.is_nil()) // boundary case
		{
			Vertex e1(map.phi2(bound));
			Vertex e2(map.phi_1(bound));
			position2[v] = Scalar(3.0/4.0) * position[v] + Scalar(1.0/8.0) * (position[e1]+position[e2]);
		}
		else
		{
			float64 beta = 3.0 / 16.0;
			if (nb_e > 3)
				beta = 3.0 / (8.0 * nb_e);
			position2[v] = Scalar(beta)*sum_edge + Scalar(1.0-beta*nb_e) * position[v];
		}
	},
	initial_cache);

	// add edges inside faces
	map.foreach_cell([&] (Face f)
	{
		Dart d0 = f.dart;
		if (initial_edge_marker.is_marked(d0))
			d0 = map.phi1(d0);

		Dart d1 = map.template phi<11>(d0);
		map.cut_face(d0, d1);

		Dart d2 = map.template phi<11>(d1);
		map.cut_face(d1, d2);

		Dart d3 = map.template phi<11>(d2);
		map.cut_face(d2, d3);
	},
	initial_cache);

	map.swap_attributes(position, position2);
	map.remove_attribute(position2);
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_LOOP_CPP_))
extern template CGOGN_MODELING_API void loop<Eigen::Vector3f, CMap2>(CMap2&, CMap2::VertexAttribute<Eigen::Vector3f>&);
extern template CGOGN_MODELING_API void loop<Eigen::Vector3d, CMap2>(CMap2&, CMap2::VertexAttribute<Eigen::Vector3d>&);
extern template CGOGN_MODELING_API void loop<Eigen::Vector3f, CMap3>(CMap3&, CMap3::VertexAttribute<Eigen::Vector3f>&);
extern template CGOGN_MODELING_API void loop<Eigen::Vector3d, CMap3>(CMap3&, CMap3::VertexAttribute<Eigen::Vector3d>&);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_LOOP_CPP_))

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_LOOP_H_

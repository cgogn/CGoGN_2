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

#include <cgogn/modeling/dll.h>
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

template <typename VEC3, typename MAP>
void catmull_clark(MAP& map, typename MAP::template VertexAttribute<VEC3>& position)
{
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	DartMarker<MAP> initial_edge_marker(map);

	std::vector<Edge>* initial_edges = cgogn::dart_buffers()->cell_buffer<Edge>();
	map.foreach_cell([&] (Edge e)
	{
		initial_edges->push_back(e);
		initial_edge_marker.mark_orbit(e);
	});

	std::vector<Vertex>* initial_vertices = cgogn::dart_buffers()->cell_buffer<Vertex>();
	map.foreach_cell([&] (Vertex v)
	{
		initial_vertices->push_back(v);
	});

	for (Edge e : *initial_edges)
	{
		std::pair<Vertex,Vertex> ve = map.vertices(e);
		Vertex middle = map.cut_edge(e);
		position[middle] = (position[ve.first] + position[ve.second]) / Scalar(2);
	}

	map.foreach_cell([&] (Face f)
	{
		Face ff = f;
		if (!initial_edge_marker.is_marked(f.dart))
			ff = Face(map.phi1(f.dart));

		initial_edge_marker.unmark_orbit(ff);

		VEC3 center = geometry::centroid<VEC3>(map, ff, position);
		Vertex vc = quadrangule_face(map, ff);
		position[vc] = center;
	});

	for (Edge e : *initial_edges)
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

	for (Vertex v : *initial_vertices)
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

	cgogn::dart_buffers()->release_cell_buffer(initial_edges);
	cgogn::dart_buffers()->release_cell_buffer(initial_vertices);
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_CATMULL_CLARK_CPP_))
extern template CGOGN_MODELING_API CMap2::Vertex quadrangule_face<CMap2>(CMap2&, CMap2::Face);
extern template CGOGN_MODELING_API CMap3::Vertex quadrangule_face<CMap3>(CMap3&, CMap3::Face);
extern template CGOGN_MODELING_API void catmull_clark<Eigen::Vector3f, CMap2>(CMap2&, CMap2::VertexAttribute<Eigen::Vector3f>&);
extern template CGOGN_MODELING_API void catmull_clark<Eigen::Vector3d, CMap2>(CMap2&, CMap2::VertexAttribute<Eigen::Vector3d>&);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_CATMULL_CLARK_CPP_))

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_CATMULL_CLARK_H_

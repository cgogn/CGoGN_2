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

#ifndef CGOGN_GEOMETRY_ALGOS_PICKING_H_
#define CGOGN_GEOMETRY_ALGOS_PICKING_H_

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/basic/cell.h>
#include <cgogn/core/basic/dart_marker.h>

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/algos/area.h>
#include <cgogn/geometry/functions/basics.h>
#include <cgogn/geometry/functions/intersection.h>
#include <cgogn/geometry/functions/distance.h>
#include <cgogn/geometry/algos/ear_triangulation.h>

#include <tuple>

namespace cgogn
{

namespace geometry
{

template <typename VEC3, typename MAP>
inline void picking_internal_face(
	const MAP& m,
	const typename MAP::template VertexAttribute<VEC3>& position,
	const VEC3& A,
	const VEC3& B,
	typename std::vector<std::tuple<typename MAP::Face, VEC3, typename vector_traits<VEC3>::Scalar>>& selected
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;
	using Triplet = typename std::tuple<Face, VEC3, Scalar>;

	VEC3 AB = B - A ;
	cgogn_message_assert(AB.squaredNorm() > 0.0, "line must be defined by 2 different points");
	AB.normalize();

	// thread data
	uint32 nb_threads = thread_pool()->nb_workers();
	std::vector<std::vector<Triplet>> selected_th(nb_threads);
	std::vector<std::vector<uint32>> ear_indices_th(nb_threads);

	m.parallel_foreach_cell([&] (Face f)
	{
		uint32 th = current_thread_index();
		VEC3 inter;
		if (m.codegree(f) == 3)
		{
			const VEC3& p1 = position[Vertex(f.dart)];
			const VEC3& p2 = position[Vertex(m.phi1(f.dart))];
			const VEC3& p3 = position[Vertex(m.phi1(m.phi1(f.dart)))];
			if (intersection_ray_triangle<VEC3>(A, AB, p1, p2, p3, &inter))
				selected_th[th].push_back(std::make_tuple(f, inter, (inter-A).squaredNorm()));
		}
		else
		{
			std::vector<uint32>& ear_indices = ear_indices_th[th];
			ear_indices.clear();
			append_ear_triangulation<VEC3>(m, f, position, ear_indices);
			for (std::size_t i = 0; i < ear_indices.size(); i += 3)
			{
				const VEC3& p1 = position[ear_indices[i]];
				const VEC3& p2 = position[ear_indices[i+1]];
				const VEC3& p3 = position[ear_indices[i+2]];
				if (intersection_ray_triangle<VEC3>(A, AB, p1, p2, p3, &inter))
				{
					selected_th[th].push_back(std::make_tuple(f, inter, (inter-A).squaredNorm()));
					i = ear_indices.size();
				}
			}
		}
	});

	// merging threads results
	for (uint32 i = 0; i < nb_threads; ++i)
	{
		for (auto x : selected_th[i])
			selected.push_back(x);
	}

	// sorting function
	auto dist_sort = [] (const Triplet& f1, const Triplet& f2) -> bool
	{
		return std::get<2>(f1) < std::get<2>(f2);
	};

	// sorting
	std::sort(selected.begin(), selected.end(), dist_sort);
}

template <typename VEC3, typename MAP>
bool picking(
	const MAP& m,
	const typename MAP::template VertexAttribute<VEC3>& position,
	const VEC3& A,
	const VEC3& B,
	typename std::vector<typename MAP::Face>& selected
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Face = typename MAP::Face;
	using Triplet = typename std::tuple<Face, VEC3, Scalar>;

	std::vector<Triplet> sel;
	picking_internal_face<VEC3>(m, position, A, B, sel);

	selected.clear();
	for (const auto& fs : sel)
		selected.push_back(std::get<0>(fs));

	return !selected.empty();
}

template <typename VEC3, typename MAP>
bool picking(
	const MAP& m,
	const typename MAP::template VertexAttribute<VEC3>& position,
	const VEC3& A,
	const VEC3& B,
	typename std::vector<typename MAP::Vertex>& selected
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;
	using Triplet = typename std::tuple<Face, VEC3, Scalar>;

	std::vector<Triplet> sel;
	picking_internal_face<VEC3>(m, position, A, B, sel);

	DartMarkerStore<MAP> dm(m);
	selected.clear();
	for (const auto& fs : sel)
	{
		Scalar min_d2 = std::numeric_limits<Scalar>::max();
		Vertex closest_vertex;

		Face f = std::get<0>(fs);
		const VEC3& I = std::get<1>(fs);

		m.foreach_incident_vertex(f, [&] (Vertex v)
		{
			Scalar d2 = (position[v] - I).squaredNorm();
			if (d2 < min_d2)
			{
				min_d2 = d2;
				closest_vertex = v;
			}
		});

		if (!dm.is_marked(closest_vertex.dart))
		{
			dm.mark_orbit(closest_vertex);
			selected.push_back(closest_vertex);
		}
	}

	return !selected.empty();
}

template <typename VEC3, typename MAP>
bool picking(
	const MAP& m,
	const typename MAP::template VertexAttribute<VEC3>& position,
	const VEC3& A,
	const VEC3& B,
	typename std::vector<typename MAP::Edge>& selected
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;
	using Triplet = typename std::tuple<Face, VEC3, Scalar>;

	std::vector<Triplet> sel;
	picking_internal_face<VEC3>(m, position, A, B, sel);

	DartMarkerStore<MAP> dm(m);
	selected.clear();
	for (const auto& fs : sel)
	{
		Scalar min_d2 = std::numeric_limits<Scalar>::max();
		Edge closest_edge;

		Face f = std::get<0>(fs);
		const VEC3& I = std::get<1>(fs);

		m.foreach_incident_edge(f, [&] (Edge e)
		{
			const VEC3& p_e1 = position[Vertex(e.dart)];
			const VEC3& p_e2 = position[Vertex(m.phi1(e.dart))];
			Scalar d2 = squared_distance_line_point(p_e1, p_e2, I);
			if (d2 < min_d2)
			{
				min_d2 = d2;
				closest_edge = e;
			}
		});

		if (!dm.is_marked(closest_edge.dart))
		{
			dm.mark_orbit(closest_edge);
			selected.push_back(closest_edge);
		}
	}

	return !selected.empty();
}

template <typename VEC3, typename MAP>
bool picking(
	const MAP& m,
	const typename MAP::template VertexAttribute<VEC3>& position,
	const VEC3& A,
	const VEC3& B,
	typename std::vector<typename MAP::Volume>& selected
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Face = typename MAP::Face;
	using Volume = typename MAP::Volume;
	using Triplet = typename std::tuple<Face, VEC3, Scalar>;

	std::vector<Triplet> sel;
	picking_internal_face<VEC3>(m, position, A, B, sel);

	selected.clear();
	DartMarker<MAP> dm(m);
	for (const auto& fs : sel)
	{
		Face f = std::get<0>(fs);
		m.foreach_incident_volume(f, [&] (Volume vo)
		{
			if ((!dm.is_marked(vo.dart)) && (!m.is_boundary(vo.dart)))
			{
				dm.mark_orbit(vo);
				selected.push_back(vo);
			}
		});
	}

	return !selected.empty();
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_PICKING_H_

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

#ifndef GEOMETRY_ALGOS_PICKING_H_
#define GEOMETRY_ALGOS_PICKING_H_

#include <core/utils/precision.h>
#include <core/basic/cell.h>
#include <core/basic/dart_marker.h>

#include <geometry/algos/area.h>
#include <geometry/functions/basics.h>
#include <geometry/functions/intersection.h>
#include <geometry/functions/distance.h>
#include <geometry/types/geometry_traits.h>

#include <tuple>

namespace cgogn
{

namespace geometry
{

template <typename VEC3, typename MAP>
inline void picking_internal_face(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position, const VEC3& A, const VEC3& B, typename std::vector<std::tuple<typename MAP::Face, VEC3, typename VEC3::Scalar>>& selected )
{
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;
	using Scalar = typename VEC3::Scalar;

	VEC3 AB = B - A ;
	cgogn_message_assert(AB.squaredNorm() > 0.0, "line must be defined by 2 different points");
	AB.normalize();

	// thread data
	using Triplet = typename std::vector<std::tuple<Face, VEC3, typename VEC3::Scalar>>;
	std::vector<Triplet> selected_th(cgogn::get_nb_threads());
	std::vector<std::vector<unsigned int>> ear_indices_th(cgogn::get_nb_threads());

	m.parallel_foreach_cell([&] (Face f, unsigned int th)
	{
		VEC3 inter;
		if (m.has_codegree(f, 3))
		{
			const VEC3& p1 = position[Vertex(f.dart)];
			const VEC3& p2 = position[Vertex(m.phi1(f.dart))];
			const VEC3& p3 = position[Vertex(m.phi1(m.phi1(f.dart)))];
			if (intersection_ray_triangle<VEC3>(A, AB, p1, p2, p3, &inter))
				selected_th[th].push_back(std::make_tuple(f, inter, (inter-A).squaredNorm()));
		}
		else
		{
			std::vector<unsigned int>& ear_indices = ear_indices_th[th];
			ear_indices.clear();
			cgogn::geometry::compute_ear_triangulation<VEC3>(m, f, position, ear_indices);
			for(std::size_t i = 0; i < ear_indices.size(); i += 3)
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

	//merging thread result
	for (unsigned int i = 0; i < cgogn::get_nb_threads(); ++i)
	{
		for (auto x : selected_th[i])
			selected.push_back(x);
	}

	//sorting function
	auto dist_sort = [] (const std::tuple<Face, VEC3,Scalar>& f1, const std::tuple<Face, VEC3,Scalar>& f2) -> bool
	{
		return std::get<2>(f1) < std::get<2>(f2);
	};

	// sorting
	std::sort(selected.begin(), selected.end(), dist_sort);
}

template <typename VEC3, typename MAP>
bool picking_faces(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position, const VEC3& A, const VEC3& B, typename std::vector<typename MAP::Face>& selected)
{
	typename std::vector<std::tuple<typename MAP::Face, VEC3, typename VEC3::Scalar>> sel;
	picking_internal_face<VEC3>(m, position, A, B, sel);

	DartMarkerStore<MAP> dm(m);
	selected.clear();
	for (auto fs : sel)
	{
		selected.push_back(std::get<0>(fs));
	}

	return !selected.empty();
}

template <typename VEC3, typename MAP>
bool picking_vertices(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position, const VEC3& A, const VEC3& B, typename std::vector<typename MAP::Vertex>& selected)
{
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;
	using Scalar = typename VEC3::Scalar;

	typename std::vector<std::tuple<Face, VEC3, typename VEC3::Scalar>> sel;
	picking_internal_face<VEC3>(m, position, A, B, sel);

	DartMarkerStore<MAP> dm(m);
	selected.clear();
	for (auto fs : sel)
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
bool picking_edges(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position, const VEC3& A, const VEC3& B, typename std::vector<typename MAP::Edge>& selected)
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;
	using Scalar = typename VEC3::Scalar;

	typename std::vector<std::tuple<Face, VEC3, typename VEC3::Scalar>> sel;
	picking_internal_face<VEC3>(m, position, A, B, sel);

	DartMarkerStore<MAP> dm(m);
	selected.clear();
	for (auto fs : sel)
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
bool picking_volumes(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position, const VEC3& A, const VEC3& B, typename std::vector<typename MAP::Volume>& selected)
{
	//here used Face2 for selecting the 2 volumes incident to selected faces

	using Face = typename MAP::Face;
	using Volume = typename MAP::Volume;

	typename std::vector<std::tuple<Face, VEC3, typename VEC3::Scalar>> sel;
	picking_internal_face<VEC3>(m, position, A, B, sel);

	selected.clear();
	DartMarker<MAP> dm(m);
	for (auto fs : sel)
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

#endif // GEOMETRY_ALGOS_PICKING_H_

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

#ifndef CGOGN_GEOMETRY_ALGOS_LENGTH_H_
#define CGOGN_GEOMETRY_ALGOS_LENGTH_H_

#include <cgogn/core/basic/cell.h>

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/core/utils/masks.h>
#include <cgogn/core/utils/thread.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3, typename MAP>
inline VEC3 vector_from(
	const MAP& map,
	const Dart d,
	const typename MAP::template VertexAttribute<VEC3>& position
)
{
	using Vertex = typename MAP::Vertex;

	return position[Vertex(map.phi1(d))] - position[Vertex(d)];
}

template <typename VEC3, typename MAP>
inline typename vector_traits<VEC3>::Scalar length(
	const MAP& map,
	const typename MAP::Edge e,
	const typename MAP::template VertexAttribute<VEC3>& position
)
{
	return vector_from<VEC3>(map, e.dart, position).norm();
}

template <typename VEC3, typename MAP, typename MASK>
inline void compute_length(
	const MAP& map,
	const MASK& mask,
	const typename MAP::template VertexAttribute<VEC3>& position,
	typename MAP::template EdgeAttribute<VEC3>& edge_length
)
{
	map.parallel_foreach_cell([&] (typename MAP::Edge e)
	{
		edge_length[e] = length<VEC3>(map, e, position);
	},
	mask);
}

template <typename VEC3, typename MAP>
inline void compute_length(
	const MAP& map,
	const typename MAP::template VertexAttribute<VEC3>& position,
	typename MAP::template EdgeAttribute<VEC3>& edge_length
)
{
	compute_length<VEC3>(map, AllCellsFilter(), position, edge_length);
}

template <typename VEC3, typename MAP, typename MASK>
inline typename vector_traits<VEC3>::Scalar mean_edge_length(
	const MAP& map,
	const MASK& mask,
	const typename MAP::template VertexAttribute<VEC3>& position
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Edge = typename MAP::Edge;

	std::vector<Scalar> edge_length_per_thread(thread_pool()->nb_workers());
	std::vector<uint32> nb_edges_per_thread(thread_pool()->nb_workers());
	for (Scalar& l : edge_length_per_thread) l = 0;
	for (uint32& n : nb_edges_per_thread) n = 0;

	map.parallel_foreach_cell([&] (Edge e)
	{
		uint32 thread_index =cgogn::current_thread_index();
		edge_length_per_thread[thread_index] += ::cgogn::geometry::length<VEC3>(map, e, position);
		nb_edges_per_thread[thread_index]++;
	},
	mask);

	Scalar length = 0;
	uint32 nbe = 0;
	for (Scalar l : edge_length_per_thread) length += l;
	for (uint32 n : nb_edges_per_thread) nbe += n;

	return length / Scalar(nbe);
}

template <typename VEC3, typename MAP>
inline typename vector_traits<VEC3>::Scalar mean_edge_length(
	const MAP& map,
	const typename MAP::template VertexAttribute<VEC3>& position
)
{
	return mean_edge_length<VEC3>(map, AllCellsFilter(), position);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_LENGTH_H_

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

template <typename MAP, typename VA>
inline typename VA::value_type vector_from(
	const MAP& map,
	const Dart d,
	const VA& position
)
{
	static_assert(is_attribute<VA,MAP::Vertex>::value,"position must be a vertex attribute");
	using Vertex = typename MAP::Vertex;
	return position[Vertex(map.phi1(d))] - position[Vertex(d)];
}

template <typename MAP, typename VA>
inline ScalarOf<typename VA::value_type> length(
	const MAP& map,
	const typename MAP::Edge e,
	const VA& position
)
{
	static_assert(is_attribute<VA,MAP::Vertex>::value,"position must be a vertex attribute");
	return vector_from(map, e.dart, position).norm();
}

template <typename MAP, typename MASK, typename VA>
inline void compute_length(
	const MAP& map,
	const MASK& mask,
	const VA& position,
	typename MAP::template EdgeAttribute<typename VA::value_type>& edge_length
)
{
	static_assert(is_attribute<VA,MAP::Vertex>::value,"position must be a vertex attribute");

	map.parallel_foreach_cell([&] (typename MAP::Edge e)
	{
		edge_length[e] = length(map, e, position);
	},
	mask);
}


template <typename MAP, typename VA>
inline void compute_length(
	const MAP& map,
	const VA& position,
	typename MAP::template EdgeAttribute<typename VA::value_type>& edge_length
)
{
	static_assert(is_attribute<VA,MAP::Vertex>::value,"position must be a vertex attribute");

	compute_length(map, AllCellsFilter(), position, edge_length);
}

template <typename MAP, typename MASK, typename VA>
inline ScalarOf<typename VA::value_type> mean_edge_length(
	const MAP& map,
	const MASK& mask,
	const VA& position
)
{
	static_assert(is_attribute<VA,MAP::Vertex>::value,"position must be a vertex attribute");

	using VEC3 = typename VA::value_type;
	using Scalar = ScalarOf<VEC3>;
	using Edge = typename MAP::Edge;

	std::vector<Scalar> edge_length_per_thread(thread_pool()->nb_workers(), 0);
	std::vector<uint32> nb_edges_per_thread(thread_pool()->nb_workers(), 0);

	map.parallel_foreach_cell([&] (Edge e)
	{
		uint32 thread_index = current_thread_index();
		edge_length_per_thread[thread_index] += length(map, e, position);
		++nb_edges_per_thread[thread_index];
	},
	mask);

	Scalar length = 0;
	uint32 nbe = 0;
	for (Scalar l : edge_length_per_thread) length += l;
	for (uint32 n : nb_edges_per_thread) nbe += n;

	return length / Scalar(nbe);
}


template <typename VEC3, typename MAP, typename VA>
inline ScalarOf<VEC3> mean_edge_length(
	const MAP& map,
	const VA& position
)
{
	static_assert(is_attribute<VA,MAP::Vertex>::value,"position must be a vertex attribute");

	return mean_edge_length(map, AllCellsFilter(), position);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_LENGTH_H_

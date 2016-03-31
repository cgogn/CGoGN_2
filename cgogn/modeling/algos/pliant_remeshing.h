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

#ifndef MODELING_ALGOS_PLIANT_REMESHING_H_
#define MODELING_ALGOS_PLIANT_REMESHING_H_

#include <geometry/functions/basics.h>
#include <core/cmap/cmap2.h>
#include <core/utils/masks.h>

namespace cgogn
{

namespace modeling
{

template <typename VEC3, typename MAP_TRAITS>
void pliant_remeshing(
	CMap2<MAP_TRAITS>& map,
	const typename CMap2<MAP_TRAITS>::template VertexAttributeHandler<VEC3>& position
)
{
	using Scalar = typename VEC3::Scalar;
	using Map = CMap2<MAP_TRAITS>;
	using Vertex = typename Map::Vertex;
	using Edge = typename Map::Edge;

	Scalar mean_edge_length = 0;

	CellCache<Edge, Map> edges(map);

	map.foreach_cell([&] (Edge e)
	{
		std::pair<Vertex,Vertex> v = map.vertices(e);
		VEC3 edge = position[v.first] - position[v.second];
		mean_edge_length += edge.norm();
	}, edges);
	mean_edge_length /= edges.size();

	cgogn_log_info("pliant_remeshing") << "mean edge length -> " << mean_edge_length;
}

} // namespace modeling

} // namespace cgogn

#endif // MODELING_ALGOS_PLIANT_REMESHING_H_

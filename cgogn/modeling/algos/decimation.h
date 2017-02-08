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

#ifndef CGOGN_MODELING_ALGOS_DECIMATION_H_
#define CGOGN_MODELING_ALGOS_DECIMATION_H_

#include <cgogn/modeling/dll.h>

#include <cgogn/geometry/functions/basics.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/core/cmap/cmap2.h>

#include <cgogn/modeling/decimation/edge_traversor_map_order.h>
#include <cgogn/modeling/decimation/edge_traversor_edge_length.h>
#include <cgogn/modeling/decimation/edge_traversor_qem.h>

#include <cgogn/modeling/decimation/edge_approximator_mid_edge.h>
#include <cgogn/modeling/decimation/edge_approximator_qem.h>

namespace cgogn
{

namespace modeling
{

template <typename VEC3>
void decimate(
	CMap2& map,
	typename CMap2::template VertexAttribute<VEC3>& position,
	uint32 nb
)
{
	using Vertex = CMap2::Vertex;
	using Edge = CMap2::Edge;

	EdgeApproximator_QEM<CMap2, VEC3> approx(map, position);
	EdgeTraversor_QEM<CMap2, VEC3> trav(map, position, approx);

	uint32 count = 0;
	map.foreach_cell([&] (Edge e) -> bool
	{
		VEC3 newpos = approx(e);

		trav.pre_collapse(e);

		Vertex v = map.collapse_edge(e);
		position[v] = newpos;

		trav.post_collapse();

		++count;
		return count < nb;
	}, trav);
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_DECIMATION_CPP_))
extern template CGOGN_MODELING_API void decimate<Eigen::Vector3f>(CMap2&, CMap2::VertexAttribute<Eigen::Vector3f>&, uint32);
extern template CGOGN_MODELING_API void decimate<Eigen::Vector3d>(CMap2&, CMap2::VertexAttribute<Eigen::Vector3d>&, uint32);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_DECIMATION_CPP_))

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_DECIMATION_H_

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

namespace cgogn
{

namespace geometry
{

template <typename VEC3_T, typename MAP>
inline VEC3_T vector_from(
		const MAP& map,
		const Dart d,
		const typename MAP::template VertexAttribute<VEC3_T>& position)
{
	using Vertex = typename MAP::Vertex;

	VEC3_T vec = position[Vertex(map.phi1(d))] ;
	vec -= position[Vertex(d)] ;
	return vec ;
}

template <typename VEC3_T, typename MAP>
inline typename VEC3_T::Scalar edge_length(
		const MAP& map,
		const typename MAP::Edge e,
		const typename MAP::template VertexAttribute<VEC3_T>& position)
{
	return vector_from<VEC3_T, MAP>(map, e.dart, position).norm();
}

template <typename VEC3_T, typename MAP>
inline typename VEC3_T::Scalar mean_edge_length(
		const MAP& map,
		const typename MAP::template VertexAttribute<VEC3_T>& position)
{
	using Scalar = typename VEC3_T::Scalar;
	using Edge = typename MAP::Edge;

	Scalar length(0);
	uint32 nbe = 0;

	map.foreach_cell([&](Edge e)
	{
		length += edge_length<VEC3_T, MAP>(map, e, position);
		++nbe;
	});

	return length / Scalar(nbe);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_LENGTH_H_

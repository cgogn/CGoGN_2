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

#ifndef GEOMETRY_ALGOS_FILTERING_H_
#define GEOMETRY_ALGOS_FILTERING_H_

#include <geometry/types/geometry_traits.h>

namespace cgogn
{

namespace geometry
{

template <typename T, typename MAP>
void filter_average(
	const MAP& map,
	const typename MAP::template VertexAttributeHandler<T>& attribute_in,
	typename MAP::template VertexAttributeHandler<T>& attribute_out)
{
	using Vertex = typename MAP::Vertex;

	map.foreach_cell([&] (Vertex v)
	{
		T sum;
		set_zero(sum);
		uint32 count = 0;
		map.foreach_adjacent_vertex_through_edge(v, [&] (Vertex av)
		{
			sum += attribute_in[av];
			++count;
		});
		attribute_out[v] = sum / typename vector_traits<T>::Scalar(count);
	});
}

} // namespace geometry

} // namespace cgogn

#endif // GEOMETRY_ALGOS_FILTERING_H_

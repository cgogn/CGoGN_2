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

#ifndef CGOGN_GEOMETRY_ALGOS_FILTERING_H_
#define CGOGN_GEOMETRY_ALGOS_FILTERING_H_

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/core/utils/masks.h>

namespace cgogn
{

namespace geometry
{

template <typename MAP, typename MASK, typename VERTEX_ATTR>
void filter_average(
	const MAP& map,
	const MASK& mask,
	const VERTEX_ATTR& attribute_in,
	VERTEX_ATTR& attribute_out
)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value,"attribute_in & attribute_out must be a vertex attribute");

	using T = InsideTypeOf<VERTEX_ATTR>;
	using Scalar = ScalarOf<T>;
	using Vertex = typename MAP::Vertex;

	map.parallel_foreach_cell([&] (Vertex v)
	{
		T sum;
		set_zero(sum);
		uint32 count = 0;
		map.foreach_adjacent_vertex_through_edge(v, [&] (Vertex av)
		{
			sum += attribute_in[av];
			++count;
		});
		attribute_out[v] = sum / Scalar(count);
	},
	mask);
}

template <typename MAP, typename VERTEX_ATTR>
void filter_average(
	const MAP& map,
	const VERTEX_ATTR& attribute_in,
	VERTEX_ATTR& attribute_out
)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value,"attribute_in & attribute_out must be a vertex attribute");

	filter_average(map, AllCellsFilter(), attribute_in, attribute_out);
}

template <typename MAP, typename MASK, typename VERTEX_ATTR>
void filter_bilateral(
	const MAP& map,
	const MASK& mask,
	const VERTEX_ATTR& position_in,
	VERTEX_ATTR& position_out,
	const VERTEX_ATTR& normal
)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value,"position_in, position_out & normal must be a vertex attribute");

	using VEC3 = InsideTypeOf<VERTEX_ATTR>;
	using Scalar = ScalarOf<VEC3>;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;

	Scalar length_sum = 0;
	Scalar angle_sum = 0;
	uint32 nb_edges = 0;

	map.foreach_cell([&] (Edge e)
	{
		std::pair<Vertex, Vertex> v = map.vertices(e);
		VEC3 edge = position_in[v.first] - position_in[v.second];
		length_sum += edge.norm();
		angle_sum += angle(normal[v.first], normal[v.second]);
		++nb_edges;
	},
	mask);

	Scalar sigmaC = 1.0 * (length_sum / Scalar(nb_edges));
	Scalar sigmaS = 2.5 * (angle_sum / Scalar(nb_edges));

	map.parallel_foreach_cell([&] (Vertex v)
	{
		const VEC3& n = normal[v];

		Scalar sum = 0, normalizer = 0;
		map.foreach_adjacent_vertex_through_edge(v, [&] (Vertex av)
		{
			VEC3 edge = position_in[av] - position_in[v];
			Scalar t = edge.norm();
			Scalar h = n.dot(edge);
			Scalar wcs = std::exp((-1.0 * (t * t) / (2.0 * sigmaC * sigmaC)) + (-1.0 * (h * h) / (2.0 * sigmaS * sigmaS)));
			sum += wcs * h;
			normalizer += wcs;
		});

		position_out[v] = position_in[v] + ((sum / normalizer) * n);
	},
	mask);
}


template <typename MAP, typename VERTEX_ATTR>
void filter_bilateral(
	const MAP& map,
	const VERTEX_ATTR& position_in,
	VERTEX_ATTR& position_out,
	const VERTEX_ATTR& normal
)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value,"position_in, position_out & normal must be a vertex attribute");
	filter_bilateral(map, AllCellsFilter(), position_in, position_out, normal);
}

template <typename MAP, typename MASK, typename VERTEX_ATTR>
void filter_taubin(
	const MAP& map,
	const MASK& mask,
	VERTEX_ATTR& position,
	VERTEX_ATTR& position_tmp)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value,"position & position_tmp must be a vertex attribute");

	using VEC3 = InsideTypeOf<VERTEX_ATTR>;
	using Scalar = ScalarOf<VEC3>;
	using Vertex = typename MAP::Vertex;

	const Scalar lambda = 0.6307;
	const Scalar mu = 0.6732;

	map.parallel_foreach_cell([&] (Vertex v)
	{
		VEC3 avg;
		set_zero(avg);
		uint32 count = 0;
		map.foreach_adjacent_vertex_through_edge(v, [&] (Vertex av)
		{
			avg += position[av];
			++count;
		});
		avg /= Scalar(count);
		const VEC3& p = position[v];
		position_tmp[v] = p + ((avg - p) * lambda);
	},
	mask);

	map.parallel_foreach_cell([&] (Vertex v)
	{
		VEC3 avg;
		set_zero(avg);
		uint32 count = 0;
		map.foreach_adjacent_vertex_through_edge(v, [&] (Vertex av)
		{
			avg += position_tmp[av];
			++count;
		});
		avg /= Scalar(count);
		const VEC3& p = position_tmp[v];
		position[v] = p + ((avg - p) * mu);
	},
	mask);
}

template <typename MAP, typename VERTEX_ATTR>
void filter_taubin(
	const MAP& map,
	VERTEX_ATTR& position,
	VERTEX_ATTR& position_tmp
)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value,"position & position_tmp must be a vertex attribute");
	filter_taubin(map, AllCellsFilter(), position, position_tmp);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_FILTERING_H_

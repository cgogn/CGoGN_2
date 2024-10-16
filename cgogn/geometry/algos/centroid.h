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

#ifndef CGOGN_GEOMETRY_ALGOS_CENTROID_H_
#define CGOGN_GEOMETRY_ALGOS_CENTROID_H_

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/core/basic/cell.h>
#include <cgogn/core/utils/masks.h>

namespace cgogn
{

namespace geometry
{

template <typename CellType, typename MAP, typename VERTEX_ATTR>
inline auto centroid(
	const MAP& map,
	const CellType c,
	const VERTEX_ATTR& attribute
) -> typename std::enable_if<is_cell_type<CellType>::value, InsideTypeOf<VERTEX_ATTR>>::type
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value, "attribute must be a vertex attribute");

	using VEC = InsideTypeOf<VERTEX_ATTR>;
	VEC result;
	set_zero(result);
	uint32 count = 0;
	map.foreach_incident_vertex(c, [&] (typename MAP::Vertex v)
	{
		result += attribute[v];
		++count;
	});
	result /= ScalarOf<VEC>(count);
	return result;
}

template <typename CellType, typename MAP, typename MASK,typename VERTEX_ATTR>
inline void compute_centroid(
	const MAP& map,
	const MASK& mask,
	const VERTEX_ATTR& attribute,
	Attribute<InsideTypeOf<VERTEX_ATTR>, CellType::ORBIT>& cell_centroid
)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value, "attribute must be a vertex attribute");

	map.parallel_foreach_cell([&] (CellType c)
	{
		cell_centroid[c] = centroid(map, c, attribute);
	},
	mask);
}

template <typename CellType, typename MAP, typename VERTEX_ATTR>
inline void compute_centroid(
	const MAP& map,
	const VERTEX_ATTR& attribute,
	Attribute<InsideTypeOf<VERTEX_ATTR>, CellType::ORBIT>& cell_centroid
)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value, "attribute must be a vertex attribute");

	compute_centroid<CellType>(map, AllCellsFilter(), attribute, cell_centroid);
}

template <typename MAP, typename MASK, typename VERTEX_ATTR>
inline auto centroid(
	const MAP& map,
	const MASK& mask,
	const VERTEX_ATTR& attribute
) -> typename std::enable_if<!is_cell_type<MASK>::value, InsideTypeOf<VERTEX_ATTR>>::type
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value, "attribute must be a vertex attribute");

	using VEC = InsideTypeOf<VERTEX_ATTR>;
	std::vector<VEC> sum_per_thread(thread_pool()->nb_workers());
	for (VEC& v :sum_per_thread) { set_zero(v); }
	std::vector<uint32> nb_vertices_per_thread(thread_pool()->nb_workers(), 0);

	map.parallel_foreach_cell([&] (typename MAP::Vertex v)
	{
		uint32 thread_index = current_thread_index();

		sum_per_thread[thread_index] += attribute[v];
		++nb_vertices_per_thread[thread_index];
	},
	mask);

	VEC result;
	set_zero(result);
	uint32 nbv = 0;
	for (VEC& v : sum_per_thread) result += v;
	for (uint32 n : nb_vertices_per_thread) nbv += n;

	return result / ScalarOf<VEC>(nbv);
}

template <typename MAP,typename VERTEX_ATTR>
inline InsideTypeOf<VERTEX_ATTR> centroid(
	const MAP& map,
	const VERTEX_ATTR& attribute
)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value, "attribute must be a vertex attribute");

	return centroid(map, AllCellsFilter(), attribute);
}

template <typename MAP, typename MASK, typename VERTEX_ATTR>
typename MAP::Vertex central_vertex(
	const MAP& map,
	const MASK& mask,
	const VERTEX_ATTR& attribute
)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value, "attribute must be a vertex attribute");

	using VEC = InsideTypeOf<VERTEX_ATTR>;
	using Vertex = typename MAP::Vertex;
	using Scalar = ScalarOf<VEC>;

	VEC center = centroid(map, mask, attribute);

	std::vector<double> min_dist_per_thread(thread_pool()->nb_workers(), std::numeric_limits<Scalar>::max());
	std::vector<Vertex> min_vertex_per_thread(thread_pool()->nb_workers());

	map.parallel_foreach_cell([&] (Vertex v)
	{
		uint32 thread_index = current_thread_index();

		Scalar distance = (attribute[v] - center).squaredNorm();

		if (distance < min_dist_per_thread[thread_index])
		{
			min_dist_per_thread[thread_index] = distance;
			min_vertex_per_thread[thread_index] = v;
		}
	},
	mask);

	uint32 min_pos = std::distance(min_dist_per_thread.begin(), std::min_element(min_dist_per_thread.begin(), min_dist_per_thread.end()));
	return min_vertex_per_thread[min_pos];
}

template <typename MAP, typename VERTEX_ATTR>
typename MAP::Vertex central_vertex(
	const MAP& map,
	const VERTEX_ATTR& attribute
)
{
	static_assert(is_orbit_of<VERTEX_ATTR, MAP::Vertex::ORBIT>::value, "attribute must be a vertex attribute");

	return central_vertex(map, AllCellsFilter(), attribute);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_CENTROID_H_

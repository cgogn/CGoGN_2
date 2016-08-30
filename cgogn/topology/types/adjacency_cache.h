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

#ifndef CGOGN_TOPOLOGY_TYPES_ADJACENCY_CACHE_H_
#define CGOGN_TOPOLOGY_TYPES_ADJACENCY_CACHE_H_

#include <cgogn/topology/dll.h>
#include <cgogn/core/cmap/cmap3.h>

namespace cgogn
{

namespace topology
{


template <typename MAP>
class AdjacencyCache
{
	using Vertex = typename MAP::Vertex;
	using VertexArray = std::vector<Vertex>;
	template<typename T>
	using VertexAttribute = typename MAP::template VertexAttribute<T>;

public:

	inline AdjacencyCache(MAP& map) :
		map_(map)
	{
	}

	inline AdjacencyCache(const AdjacencyCache& other) :
		map_(other.map_),
		adjacency_(other.adjacency_)
	{}

	inline AdjacencyCache(AdjacencyCache&& other) :
		map_(other.map_),
		adjacency_(std::move(other.adjacency_))
	{}

	const AdjacencyCache& operator=(AdjacencyCache&&) = delete;
	const AdjacencyCache& operator=(const AdjacencyCache& ) = delete;

	inline ~AdjacencyCache()
	{
//		map_.remove_attribute(adjacency_);
	}

	void init()
	{
		map_.add_attribute(adjacency_, "__adjacency__");
		map_.foreach_cell([&](Vertex v)
		{
			map_.foreach_adjacent_vertex_through_edge(v, [&](Vertex u)
			{
				adjacency_[v].push_back(u);
			});
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_edge(Vertex v, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		for (Vertex u : adjacency_[v]) f(u);
	}


private:
	MAP& map_;
	VertexAttribute<VertexArray> adjacency_;
};


#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_TOPOLOGY_TYPES_ADJACENCY_CACHE_CPP_))
extern template class CGOGN_TOPLOGY_API AdjacencyCache<CMap2<DefaultMapTraits>>;
extern template class CGOGN_TOPLOGY_API AdjacencyCache<CMap3<DefaultMapTraits>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_TOPOLOGY_TYPES_ADJACENCY_CACHE_CPP_))

} // namespace topology

} // namespace cgogn

#endif // CGOGN_TOPOLOGY_TYPES_ADJACENCY_CACHE_H_

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

#include <cgogn/core/utils/numerics.h>

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

	AdjacencyCache(MAP& map) :
		map_(map)
	{
	}

	~AdjacencyCache()
	{
//		map_.remove_attribute(adjacency_);
	}

	void init()
	{
		adjacency_ = map_.template add_attribute<VertexArray, Vertex::ORBIT>("__adjacency__");
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
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		for (Vertex u : adjacency_[v]) f(u);
	}


private:
	MAP& map_;
	VertexAttribute<VertexArray> adjacency_;
};

} // namespace topology

} // namespace cgogn

#endif // CGOGN_TOPOLOGY_TYPES_ADJACENCY_CACHE_H_

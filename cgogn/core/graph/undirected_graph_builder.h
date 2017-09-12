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

#ifndef CGOGN_CORE_CMAP_UNDIRECTED_GRAPH_BUILDER_H_
#define CGOGN_CORE_CMAP_UNDIRECTED_GRAPH_BUILDER_H_

#include <cgogn/core/cmap/map_base.h>

namespace cgogn
{

template <typename GRAPH>
class UndirectedGraphBuilder_T
{
	static_assert(GRAPH::DIMENSION == 2, "UndirectedGraphBuilder_T works only with 1D Graphs.");

public:
	using Self = UndirectedGraphBuilder_T<GRAPH>;
	using Graph = GRAPH;
	using CDart = typename Graph::CDart;
	using Vertex = typename Graph::Vertex;
	using Edge = typename Graph::Edge;

    template <typename T>
	using ChunkArrayContainer = typename Graph::template ChunkArrayContainer<T>;

	inline UndirectedGraphBuilder_T(Graph& map) : map_(map)
    {}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(UndirectedGraphBuilder_T);

	inline ~UndirectedGraphBuilder_T()
    {}

public:

    template <Orbit ORBIT>
    inline void create_embedding()
    {
        map_.template create_embedding<ORBIT>();
    }

    template <Orbit ORBIT, typename T>
    inline void swap_chunk_array_container(ChunkArrayContainer<T>& cac)
    {
        map_.attributes_[ORBIT].swap(cac);
    }

    template <class CellType>
    inline void set_embedding(Dart d, uint32 emb)
    {
        map_.template set_embedding<CellType>(d, emb);
    }

    template <class CellType, Orbit ORBIT>
    inline void set_orbit_embedding(Cell<ORBIT> c, uint32 emb)
    {
        map_.template set_orbit_embedding<CellType>(c, emb);
    }

    template <class CellType>
    inline void new_orbit_embedding(CellType c)
    {
        map_.new_orbit_embedding(c);
    }

    inline void alpha0_sew(Dart d, Dart e)
    {
        map_.alpha0_sew(d,e);
    }

    inline void alpha0_unsew(Dart d)
    {
        map_.alpha0_unsew(d);
    }

    inline void alpha1_sew(Dart d, Dart e)
    {
        map_.alpha1_sew(d,e);
    }

    inline void alpha1_unsew(Dart d)
    {
        map_.alpha1_unsew(d);
    }

    inline Dart add_edge_topo()
    {
        return map_.add_edge_topo();
    }

	inline Dart add_vertex_topo(uint32 nb_edges)
    {
		return map_.add_vertex_topo(nb_edges);
    }

private:

	Graph& map_;
};

} // namespace cgogn


#endif // CGOGN_CORE_CMAP_UNDIRECTED_GRAPH_BUILDER_H_

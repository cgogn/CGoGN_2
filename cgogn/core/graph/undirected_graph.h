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

#ifndef CGOGN_CORE_GRAPH_UNDIRECTED_GRAPH_H_
#define CGOGN_CORE_GRAPH_UNDIRECTED_GRAPH_H_

#include <cgogn/core/cmap/map_base.h>
#include <cgogn/core/graph/undirected_graph_builder.h>

namespace cgogn
{

template <typename MAP_TYPE>
class UndirectedGraph_T : public MapBase<MAP_TYPE>
{
public:

	static const uint8 DIMENSION = 2;
	static const uint8 PRIM_SIZE = 1;

	using MapType = MAP_TYPE;
	using Inherit = MapBase<MAP_TYPE>;
	using Self = UndirectedGraph_T<MAP_TYPE>;

	using Builder = UndirectedGraphBuilder_T<Self>;

	friend class MapBase<MAP_TYPE>;
	friend class UndirectedGraphBuilder_T<Self>;
	friend class DartMarker_T<Self>;
	friend class cgogn::DartMarkerStore<Self>;

	using CDart = Cell<Orbit::DART>;
	using Vertex = Cell<Orbit::PHI21>;
	using Edge = Cell<Orbit::PHI2>;
	using ConnectedComponent = Cell<Orbit::PHI1>;

	using Boundary = Vertex;

	template <typename T>
	using ChunkArrayContainer = typename Inherit::template ChunkArrayContainer<T>;
	using typename Inherit::ChunkArrayGen;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	template <typename T>
	using CDartAttribute = Attribute<T, CDart::ORBIT>;
	template <typename T>
	using VertexAttribute = Attribute<T, Vertex::ORBIT>;
	template <typename T>
	using EdgeAttribute = Attribute<T, Edge::ORBIT>;

	using DartMarker = typename cgogn::DartMarker<Self>;
	using DartMarkerStore = typename cgogn::DartMarkerStore<Self>;
	using DartMarkerNoUnmark = typename cgogn::DartMarkerNoUnmark<Self>;

	template <Orbit ORBIT>
	using CellMarker = typename cgogn::CellMarker<Self, ORBIT>;
	template <Orbit ORBIT>
	using CellMarkerNoUnmark = typename cgogn::CellMarkerNoUnmark<Self, ORBIT>;
	template <Orbit ORBIT>
	using CellMarkerStore = typename cgogn::CellMarkerStore<Self, ORBIT>;

	using CellCache = typename cgogn::CellCache<Self>;
	using QuickTraversor = typename cgogn::QuickTraversor<Self>;
	using BoundaryCache = typename cgogn::BoundaryCache<Self>;

protected:

	ChunkArray<Dart>* alpha0_;
	ChunkArray<Dart>* alpha1_;
	ChunkArray<Dart>* alpha_1_;

	void init()
	{
		alpha0_ = this->topology_.template add_chunk_array<Dart>("alpha0");
		alpha1_ = this->topology_.template add_chunk_array<Dart>("alpha1");
		alpha_1_ = this->topology_.template add_chunk_array<Dart>("alpha_1");
	}

public:

	UndirectedGraph_T() : Inherit()
	{
		init();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(UndirectedGraph_T);

	~UndirectedGraph_T() override
	{}

	inline bool check_embedding_integrity()
	{
		bool result = true;

		if (this->template is_embedded<CDart>())
			result = result && this->template is_well_embedded<CDart>();

		if (this->template is_embedded<Vertex>())
			result = result && this->template is_well_embedded<Vertex>();

		if (this->template is_embedded<Edge>())
			result = result && this->template is_well_embedded<Edge>();

		return result;
	}

protected:

	inline void init_dart(Dart d)
	{
		(*alpha0_)[d.index] = d;
		(*alpha1_)[d.index] = d;
		(*alpha_1_)[d.index] = d;
	}

	inline bool check_integrity(Dart d) const
	{
		return alpha0(alpha0(d)) == d &&
				alpha1(alpha_1(d)) == d &&
				alpha_1(alpha1(d)) == d;
	}

	/* alpha0 is an involution */
	inline void alpha0_sew(Dart d, Dart e)
	{
		(*alpha0_)[d.index] = e;
		(*alpha0_)[e.index] = d;
	}

	inline void alpha0_unsew(Dart d)
	{
		Dart e = alpha0(d);
		(*alpha0_)[d.index] = d;
		(*alpha0_)[e.index] = e;
	}

	/* alpha1 is a permutation */
	inline void alpha1_sew(Dart d, Dart e)
	{
		Dart f = alpha1(d);
		Dart g = alpha1(e);
		(*alpha1_)[d.index] = g;
		(*alpha1_)[e.index] = f;
		(*alpha_1_)[g.index] = d;
		(*alpha_1_)[f.index] = e;
	}

	inline void alpha1_unsew(Dart d)
	{
		Dart e = alpha1(d);
		Dart f = alpha_1(d);
		(*alpha1_)[f.index] = e;
		(*alpha1_)[d.index] = d;
		(*alpha_1_)[e.index] = f;
		(*alpha_1_)[d.index] = d;
	}

public:

	inline Dart alpha0(Dart d) const
	{
		return (*alpha0_)[d.index];
	}

	inline Dart alpha1(Dart d) const
	{
		return (*alpha1_)[d.index];
	}

	inline Dart alpha_1(Dart d) const
	{
		return (*alpha_1_)[d.index];
	}

protected:

	inline Dart add_vertex_topo(std::size_t size)
	{
		cgogn_message_assert(size > 0u, "Cannot create an empty vertex");

		Dart d = this->add_topology_element();
		Dart first = d;
		Dart dit;
		for (size_t i = 0 ; i < size - 1 ; ++i)
		{
			dit = this->add_topology_element();
			alpha1_sew(d,dit);
			d = dit;
		}

		return first;
	}

	inline void remove_vertex_topo(Dart d)
	{
		Dart it = alpha1(d);
		while(it != d)
		{
			Dart next = alpha1(it);
			alpha0_unsew(it);
			this->remove_topology_element(it);
			it = next;
		}

		alpha0_unsew(d);
		this->remove_topology_element(d);
	}

	inline Dart insert_vertex_topo(Dart d)
	{
		Dart d0 = alpha0(d);
		alpha0_unsew(d);
		Dart v = add_vertex_topo(2);
		alpha0_sew(d, v);
		alpha0_sew(d0, alpha1(v));

		return v;
	}

public:

	inline Vertex insert_vertex(Edge e)
	{
		Vertex v(insert_vertex_topo(e.dart));

		if (this->template is_embedded<Vertex>())
			this->new_orbit_embedding(v);

		if (this->template is_embedded<Edge>())
		{
			this->template set_orbit_embedding<Edge>(e, this->embedding(e));
			Edge e2(alpha1(v.dart));
			this->new_orbit_embedding(e2);
		}

		return v;
	}

protected:

	inline Dart collapse_vertex_topo(Dart d)
	{
		Dart e = alpha0(d);
		Dart ee = alpha0(alpha1(d));

		remove_vertex_topo(d);
		alpha0_sew(e, ee);

		return e;
	}

public:

	inline void collapse_vertex(Vertex v)
	{
		cgogn_message_assert(degree(v) == 2, "Can only collapse vertex of degree 2");

		Edge e(collapse_vertex_topo(v.dart));

		if (this->template is_embedded<Edge>())
			this->template set_orbit_embedding<Edge>(e, this->embedding(e));
	}

protected:

	inline void merge_vertices_topo(Dart v1, Dart v2)
	{
		alpha1_sew(v1, v2);
	}

public:

	inline void merge_vertices(Vertex v1, Vertex v2)
	{
		merge_vertices_topo(v1.dart, v2.dart);
		if (this->template is_embedded<Vertex>())
			this->template set_orbit_embedding<Vertex>(v1, this->embedding(v1));
	}

protected:

	inline Dart add_edge_topo()
	{
		Dart d = this->add_topology_element();
		Dart e = this->add_topology_element();

		alpha0_sew(d,e);

		return d;
	}

public:

	inline Edge add_edge()
	{
		Edge e(add_edge_topo());

		if (this->template is_embedded<Vertex>())
		{
			this->new_orbit_embedding(Vertex(e.dart));
			this->new_orbit_embedding(Vertex(alpha0(e.dart)));
		}

		if (this->template is_embedded<Edge>())
			this->new_orbit_embedding(e);

		return e;
	}

protected:

	inline void disconnect_edge_topo(Dart e)
	{
		alpha1_unsew(e);
	}

public:

	inline void disconnect_edge(Edge e)
	{
		disconnect_edge_topo(e.dart);
		if (this->template is_embedded<Vertex>())
			this->new_orbit_embedding(Vertex(e.dart));
	}

protected:

	inline Dart collapse_edge_topo(Dart e)
	{
		Dart e0 = alpha0(e);
		Dart e1 = alpha1(e);
		Dart e2 = alpha_1(alpha0(e));

		alpha1_unsew(e);
		alpha1_unsew(e0);
		this->remove_topology_element(e);
		this->remove_topology_element(e0);

		alpha1_sew(e1, e2);

		return e1;
	}

public:

	inline Vertex collapse_edge(Edge e)
	{
		Vertex v(collapse_edge_topo(e.dart));
		if (this->template is_embedded<Vertex>())
		{
			this->template set_orbit_embedding<Vertex>(v, this->embedding(v));
		}

		return v;
	}

public:

	inline Vertex make_polyline(uint32 n)
	{
		Vertex last;

		for (uint32 i = 0; i < n - 1; ++i)
		{
			Edge cur = add_edge();
			if (last.is_valid())
			{
				alpha1_sew(last.dart, cur.dart);

				if (this->template is_embedded<Vertex>())
					this->template copy_embedding<Vertex>(cur.dart, last.dart);
			}
			last = Vertex(alpha0(cur.dart));
		}

		return last;
	}

	inline Vertex make_loop(uint32 n)
	{
		Vertex last;
		Vertex first;

		for (uint32 i = 0; i < n; ++i)
		{
			Edge cur = add_edge();
			if (last.is_valid())
			{
				alpha1_sew(last.dart, cur.dart);

				if (this->template is_embedded<Vertex>())
					this->template copy_embedding<Vertex>(cur.dart, last.dart);
			}
			else
				first = Vertex(cur.dart);

			last = Vertex(alpha0(cur.dart));
		}
		alpha1_sew(first.dart, last.dart);

		if (this->template is_embedded<Vertex>())
			this->template copy_embedding<Vertex>(last.dart, first.dart);

		return first;
	}

	inline std::pair<Vertex, Vertex> vertices(Edge e) const
	{
		return std::pair<Vertex, Vertex>(Vertex(e.dart), Vertex(alpha0(e.dart)));
	}

	/*******************************************************************************
	 * Connectivity information
	 *******************************************************************************/

public:

	inline uint32 degree(Vertex v) const
	{
		return this->nb_darts_of_orbit(v);
	}

	/*******************************************************************************
	* Orbits traversal                                                             *
	*******************************************************************************/

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Dart>::value, "Wrong function parameter type");
		static_assert(ORBIT == Orbit::DART || ORBIT == Orbit::PHI1 || ORBIT == Orbit::PHI2 ||
					  ORBIT == Orbit::PHI21, "Orbit not supported in a Primal CMap2");

		switch (ORBIT)
		{
			case Orbit::DART: foreach_dart_of_DART(c.dart, f); break;
			case Orbit::PHI21: foreach_dart_of_ALPHA1(c.dart, f); break;
			case Orbit::PHI2: foreach_dart_of_ALPHA0(c.dart, f); break;
			case Orbit::PHI1: foreach_dart_of_ALPHA01(c.dart, f); break;
			case Orbit::PHI1_PHI2:
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI21_PHI31:
			default: cgogn_assert_not_reached("Orbit not supported in a Graph"); break;
		}
	}

protected:

	template <typename FUNC>
	inline void foreach_dart_of_DART(Dart d, const FUNC& f) const
	{
		f(d);
	}

	template <typename FUNC>
	inline void foreach_dart_of_ALPHA01(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			if (!internal::void_to_true_binder(f, it))
				break;
			it = alpha1(alpha0(it));
		} while (it != d);
	}

	template <typename FUNC>
	inline void foreach_dart_of_ALPHA0(Dart d, const FUNC& f) const
	{
		if (internal::void_to_true_binder(f, d))
			f(alpha0(d));
	}

	template <typename FUNC>
	inline void foreach_dart_of_ALPHA1(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{			
			if (!internal::void_to_true_binder(f, it))
				break;
			it = alpha1(it);
		} while (it != d);
	}

public:

	/******************************************************************************
	* Incidence traversal                                                         *
	******************************************************************************/

	template <typename FUNC>
	inline void foreach_incident_edge(Vertex v, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [&] (Dart d) -> bool { return internal::void_to_true_binder(func, Edge(d)); });
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(Edge e, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(e, [&func] (Dart d) -> bool { return internal::void_to_true_binder(func , Vertex(d)); });
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(ConnectedComponent cc, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(cc, [&] (Dart d)
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit(Vertex(d));
				func(Vertex(d));
			}
		});
	}

	template <typename FUNC>
	inline void foreach_incident_edge(ConnectedComponent cc, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(cc, [&] (Dart d) -> bool
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit(Edge(d));
				return internal::void_to_true_binder(func, Edge(d));
			}
			return true;
		});
	}

	/*******************************************************************************
	* Adjacence traversal                                                          *
	*******************************************************************************/

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_edge(Vertex v, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		foreach_dart_of_orbit(v, [this, &f] (Dart d) -> bool { return internal::void_to_true_binder(f, Vertex(this->alpha0(d))); });
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_vertex(Edge e, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		static_assert(is_func_return_same<FUNC, bool>::value, "Wrong function return type");
		foreach_dart_of_orbit(e, [this, &f] (Dart ed) -> bool
		{
			bool res_nested_lambda = true;
			this->foreach_dart_of_orbit(Vertex(ed), [&f, &res_nested_lambda, ed] (Dart vd) -> bool
			{
				// skip Edge e itself
				if (vd != ed)
					res_nested_lambda = internal::void_to_true_binder(f, Edge(vd));
				return res_nested_lambda;
			});
			return res_nested_lambda;
		});
	}

	/**
	* @brief check if embedding of map is also embedded in this (create if not). Used by merge method
	* @param map
	*/
	void merge_check_embedding(const Self& map)
	{
		const static auto create_embedding = [=] (Self* map_ptr, Orbit orb)
		{
			switch (orb)
			{
				case Orbit::DART: map_ptr->template create_embedding<Orbit::DART>(); break;
				case Orbit::PHI1: map_ptr->template create_embedding<Orbit::PHI1>(); break;
				case Orbit::PHI2: map_ptr->template create_embedding<Orbit::PHI2>(); break;
				case Orbit::PHI21: map_ptr->template create_embedding<Orbit::PHI21>(); break;
				default: break;
			}
		};

		for (Orbit orb : { DART, PHI1, PHI2, PHI21})
			if (!this->is_embedded(orb) && map.is_embedded(orb))
				create_embedding(this, orb);
	}

	/**
	* @brief ensure all cells (introduced while merging) are embedded.
	* @param first index of first dart to scan
	*/
	void merge_finish_embedding(uint32 first)
	{
		const static auto new_orbit_embedding = [=] (Self* map, Dart d, cgogn::Orbit orb)
		{
			switch (orb)
			{
				case Orbit::DART: map->new_orbit_embedding(Cell<Orbit::DART>(d)); break;
				case Orbit::PHI1: map->new_orbit_embedding(Cell<Orbit::PHI1>(d)); break;
				case Orbit::PHI2: map->new_orbit_embedding(Cell<Orbit::PHI2>(d)); break;
				case Orbit::PHI21: map->new_orbit_embedding(Cell<Orbit::PHI21>(d)); break;
				default: break;
			}
		};

		for (uint32 j = first, end = this->topology_.end(); j != end; this->topology_.next(j))
		{
			for (Orbit orb : { DART, PHI1, PHI2, PHI21 })
			{
				if (this->is_embedded(orb))
				{
					if (!this->is_boundary(Dart(j)) && (*this->embeddings_[orb])[j] == INVALID_INDEX)
						new_orbit_embedding(this, Dart(j), orb);
				}
			}
		}
	}

};

struct UndirectedGraphType
{
	using TYPE = UndirectedGraph_T<UndirectedGraphType>;
};

using UndirectedGraph = UndirectedGraph_T<UndirectedGraphType>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_GRAPH_UNDIRECTED_GRAPH_CPP_))
extern template class CGOGN_CORE_API UndirectedGraph_T<UndirectedGraphType>;
extern template class CGOGN_CORE_API UndirectedGraphBuilder_T<UndirectedGraph>;
extern template class CGOGN_CORE_API DartMarker<UndirectedGraph>;
extern template class CGOGN_CORE_API DartMarkerStore<UndirectedGraph>;
extern template class CGOGN_CORE_API DartMarkerNoUnmark<UndirectedGraph>;
extern template class CGOGN_CORE_API CellMarker<UndirectedGraph, UndirectedGraph::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarker<UndirectedGraph, UndirectedGraph::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<UndirectedGraph, UndirectedGraph::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<UndirectedGraph, UndirectedGraph::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<UndirectedGraph, UndirectedGraph::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<UndirectedGraph, UndirectedGraph::Edge::ORBIT>;
extern template class CGOGN_CORE_API CellCache<UndirectedGraph>;
extern template class CGOGN_CORE_API BoundaryCache<UndirectedGraph>;
extern template class CGOGN_CORE_API QuickTraversor<UndirectedGraph>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_GRAPH_UNDIRECTED_GRAPH_CPP_))

} // end namespace cgogn

#endif // CGOGN_CORE_GRAPH_UNDIRECTED_GRAPH_H_

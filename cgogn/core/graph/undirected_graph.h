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


namespace cgogn
{

template <typename MAP>
class UndirectedGraphBuilder_T; // forward declaration

template <typename MAP_TYPE>
class UndirectedGraph_T : public MapBase<MAP_TYPE>
{
public:

	static const uint8 DIMENSION = 1;

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

	using FilteredQuickTraversor = typename cgogn::FilteredQuickTraversor<Self>;
	using QuickTraversor = typename cgogn::QuickTraversor<Self>;
	using CellCache = typename cgogn::CellCache<Self>;
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

	inline Dart add_vertex_topo()
	{
		Dart d = this->add_topology_element();
		Dart dd = this->add_topology_element();
		alpha0_sew(d, dd);
		this->set_boundary(dd, true);
		return d;
	}

public:

	inline Vertex add_vertex()
	{
		Dart d = add_vertex_topo();
		Vertex v(d);
		if (this->template is_embedded<Vertex>())
			this->new_orbit_embedding(v);
		if (this->template is_embedded<Edge>())
			this->new_orbit_embedding(Edge(d));
		return v;
	}

protected:

	inline void remove_vertex_topo(Dart d)
	{
		Dart dd = alpha0(d);
		cgogn_message_assert(this->is_boundary(dd), "Vertex is still connected to another vertex");
		this->remove_topology_element(d);
		this->remove_topology_element(dd);
	}

public:

	inline void remove_vertex(Vertex v)
	{
		Dart d = v.dart;
		Dart dd = alpha0(d);
		cgogn_message_assert(this->is_boundary(dd), "Vertex is still connected to another vertex");
		remove_vertex_topo(d);
	}

protected:

	inline Dart connect_vertices_topo(Dart d, Dart e)
	{
		Dart dd = alpha0(d);
		Dart ee = alpha0(e);
		if (this->is_boundary(dd))
		{
			if (this->is_boundary(ee))
			{
				this->remove_topology_element(dd);
				this->remove_topology_element(ee);
				alpha0_sew(d, e);
				return d;
			}
			else
			{
				this->set_boundary(dd, false);
				alpha1_sew(e, dd);
				return d;
			}
		}
		else
		{
			if (this->is_boundary(ee))
			{
				this->set_boundary(ee, false);
				alpha1_sew(d, ee);
				return ee;
			}
			else
			{
				Dart dd = this->add_topology_element();
				Dart ee = this->add_topology_element();
				alpha1_sew(d, dd);
				alpha1_sew(e, ee);
				alpha0_sew(dd, ee);
				return dd;
			}
		}
	}

public:

	inline Edge connect_vertices(Vertex v1, Vertex v2)
	{
		Dart d = v1.dart;
		Dart e = v2.dart;
		Dart dd = alpha0(d);
		Dart ee = alpha0(e);
		if (this->is_boundary(dd))
		{
			if (this->is_boundary(ee))
			{
				Edge edge(connect_vertices_topo(d, e));
				if (this->template is_embedded<Edge>())
					this->template copy_embedding<Edge>(e, d);
				return edge;
			}
			else
			{
				Edge edge(connect_vertices_topo(d, e));
				if (this->template is_embedded<Vertex>())
					this->template copy_embedding<Vertex>(dd, e);
				return edge;
			}
		}
		else
		{
			if (this->is_boundary(ee))
			{
				Edge edge(connect_vertices_topo(d, e));
				if (this->template is_embedded<Vertex>())
					this->template copy_embedding<Vertex>(ee, d);
				return edge;
			}
			else
			{
				Edge edge(connect_vertices_topo(d, e));
				if (this->template is_embedded<Vertex>())
				{
					this->template copy_embedding<Vertex>(alpha1(d), d);
					this->template copy_embedding<Vertex>(alpha1(e), e);
				}
				if (this->template is_embedded<Edge>())
					this->new_orbit_embedding(edge);
				return edge;
			}
		}
	}

protected:

	inline void disconnect_vertices_topo(Dart d)
	{
		Dart e = alpha0(d);
		cgogn_message_assert(!(this->is_boundary(d) || this->is_boundary(e)), "Given edge does not connect 2 vertices");
		if (alpha1(d) == d)
		{
			if (alpha1(e) == e)
			{
				alpha0_unsew(d);
				Dart dd = this->add_topology_element();
				Dart ee = this->add_topology_element();
				alpha0_sew(d, dd);
				alpha0_sew(e, ee);
				this->set_boundary(dd, true);
				this->set_boundary(ee, true);
			}
			else
			{
				alpha1_unsew(e);
				this->set_boundary(e, true);
			}
		}
		else
		{
			if (alpha1(e) == e)
			{
				alpha1_unsew(d);
				this->set_boundary(d, true);
			}
			else
			{
				alpha0_unsew(d);
				alpha1_unsew(d);
				alpha1_unsew(e);
				this->remove_topology_element(d);
				this->remove_topology_element(e);
			}
		}
	}

public:

	inline void disconnect_vertices(Edge edge)
	{
		Dart d = edge.dart;
		Dart e = alpha0(d);
		cgogn_message_assert(!(this->is_boundary(d) || this->is_boundary(e)), "Given edge does not connect 2 vertices");
		if (alpha1(d) == d)
		{
			if (alpha1(e) == e)
			{
				disconnect_vertices_topo(d);
				if (this->template is_embedded<Edge>())
				{
					this->template copy_embedding<Edge>(alpha0(d), d);
					this->new_orbit_embedding(Edge(e));
				}
			}
			else
			{
				disconnect_vertices_topo(d);
				this->template unset_embedding<Vertex>(alpha0(d));
			}
		}
		else
		{
			if (alpha1(e) == e)
			{
				disconnect_vertices_topo(d);
				this->template unset_embedding<Vertex>(alpha0(e));
			}
			else
			{
				disconnect_vertices_topo(d);
			}
		}
	}

	// n is the number of vertices
	inline Vertex make_polyline(uint32 n)
	{
		Vertex last = add_vertex();
		for (uint32 i = 1; i < n; ++i)
		{
			Vertex v = add_vertex();
			connect_vertices(v, last);
			last = v;
		}
		return last;
	}

	// n is the number of vertices
	inline Vertex make_loop(uint32 n)
	{
		Vertex first = add_vertex();
		Vertex last = first;
		for (uint32 i = 1; i < n; ++i)
		{
			Vertex v = add_vertex();
			connect_vertices(v, last);
			last = v;
		}
		connect_vertices(last, first);
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
			case Orbit::PHI1: foreach_dart_of_ALPHA01(c.dart, f); break;
			case Orbit::PHI2: foreach_dart_of_ALPHA0(c.dart, f); break;
			case Orbit::PHI21: foreach_dart_of_ALPHA1(c.dart, f); break;
			case Orbit::PHI1_PHI2:
			case Orbit::PHI2_PHI3:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI21_PHI31:
			case Orbit::PHI1_PHI2_PHI3:
			default: cgogn_assert_not_reached("Orbit not supported in an UndirectedGraph"); break;
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
		foreach_dart_of_orbit(e, [this, &func] (Dart d) -> bool
		{
			if (!this->is_boundary(d))
				return internal::void_to_true_binder(func , Vertex(d));
			else
				return true;
		});
	}

	template <typename FUNC>
	inline void foreach_incident_vertex(ConnectedComponent cc, const FUNC& func) const
	{
		static_assert(is_func_parameter_same<FUNC, Vertex>::value, "Wrong function cell parameter type");
		DartMarkerStore marker(*this);
		foreach_dart_of_orbit(cc, [&] (Dart d) -> bool
		{
			if (!marker.is_marked(d))
			{
				marker.mark_orbit(Vertex(d));
				if (!this->is_boundary(d))
					return internal::void_to_true_binder(func, Vertex(d));
			}
			return true;
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
		foreach_dart_of_orbit(v, [this, &f] (Dart d) -> bool
		{
			Dart dd = this->alpha0(d);
			if (!this->is_boundary(dd))
				return internal::void_to_true_binder(f, Vertex(dd));
			else
				return true;
		});
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_vertex(Edge e, const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Edge>::value, "Wrong function cell parameter type");
		static_assert(is_func_return_same<FUNC, bool>::value, "Wrong function return type");
		foreach_dart_of_orbit(e, [this, &f] (Dart ed) -> bool
		{
			bool res_nested_lambda = true;
			if (!this->is_boundary(ed))
			{
				this->foreach_dart_of_orbit(Vertex(ed), [&f, &res_nested_lambda, ed] (Dart vd) -> bool
				{
					// skip Edge e itself
					if (vd != ed)
						res_nested_lambda = internal::void_to_true_binder(f, Edge(vd));
					return res_nested_lambda;
				});
			}
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

		for (Orbit orb : { DART, PHI1, PHI2, PHI21 })
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

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_EXTERNAL_TEMPLATES_CPP_))
extern template class CGOGN_CORE_EXPORT UndirectedGraph_T<UndirectedGraphType>;
extern template class CGOGN_CORE_EXPORT DartMarker<UndirectedGraph>;
extern template class CGOGN_CORE_EXPORT DartMarkerStore<UndirectedGraph>;
extern template class CGOGN_CORE_EXPORT DartMarkerNoUnmark<UndirectedGraph>;
extern template class CGOGN_CORE_EXPORT CellMarker<UndirectedGraph, UndirectedGraph::Vertex::ORBIT>;
extern template class CGOGN_CORE_EXPORT CellMarker<UndirectedGraph, UndirectedGraph::Edge::ORBIT>;
extern template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<UndirectedGraph, UndirectedGraph::Vertex::ORBIT>;
extern template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<UndirectedGraph, UndirectedGraph::Edge::ORBIT>;
extern template class CGOGN_CORE_EXPORT CellMarkerStore<UndirectedGraph, UndirectedGraph::Vertex::ORBIT>;
extern template class CGOGN_CORE_EXPORT CellMarkerStore<UndirectedGraph, UndirectedGraph::Edge::ORBIT>;
extern template class CGOGN_CORE_EXPORT CellCache<UndirectedGraph>;
extern template class CGOGN_CORE_EXPORT BoundaryCache<UndirectedGraph>;
extern template class CGOGN_CORE_EXPORT QuickTraversor<UndirectedGraph>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_EXTERNAL_TEMPLATES_CPP_))

} // end namespace cgogn

#endif // CGOGN_CORE_GRAPH_UNDIRECTED_GRAPH_H_

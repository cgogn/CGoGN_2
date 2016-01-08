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

#ifndef CORE_MAP_MAP1_H_
#define CORE_MAP_MAP1_H_

#include <core/map/map_base.h>
#include <core/basic/dart.h>

namespace cgogn
{

template <typename DATA_TRAITS, typename TOPO_TRAITS>
class CMap1_T : public MapBase<DATA_TRAITS, TOPO_TRAITS>
{
public:

	typedef MapBase<DATA_TRAITS, TOPO_TRAITS> Inherit;
	typedef CMap1_T<DATA_TRAITS, TOPO_TRAITS> Self;

	friend typename Self::Inherit;
	friend class DartMarker_T<Self>;

	static const Orbit VERTEX = Orbit::DART;
	static const Orbit EDGE   = Orbit::DART;
	static const Orbit FACE   = Orbit::PHI1;

	typedef Cell<VERTEX> Vertex;
	typedef Cell<EDGE> Edge;
	typedef Cell<FACE> Face;

	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;
	template<typename T>
	using ChunkArrayContainer = typename Inherit::template ChunkArrayContainer<T>;

	template<typename T, Orbit ORBIT>
	using AttributeHandler = typename Inherit::template AttributeHandler<T, ORBIT>;
	template<typename T>
	using VertexAttributeHandler = AttributeHandler<T, Self::VERTEX>;
	template<typename T>
	using EdgeAttributeHandler = AttributeHandler<T, Self::EDGE>;
	template<typename T>
	using FaceAttributeHandler = AttributeHandler<T, Self::FACE>;

	using DartMarker = typename Inherit::DartMarker;
	using DartMarkerStore = typename Inherit::DartMarkerStore;

protected:

	ChunkArray<Dart>* phi1_;
	ChunkArray<Dart>* phi_1_;

	void init()
	{
		phi1_ = this->topology_.template add_attribute<Dart>("phi1");
		phi_1_ = this->topology_.template add_attribute<Dart>("phi_1");
	}

	/*******************************************************************************
	 * Low-level topological operations
	 *******************************************************************************/

	/**
	 * \brief Link the current dart to dart d with a permutation
	 * @param d the dart to which the current is linked
	 * - Before: d->f and e->g
	 * - After:  d->g and e->f
	 * Join the permutations cycles of dart d and e
	 * - Starting from two cycles : d->f->...->d and e->g->...->e
	 * - It makes one cycle d->g->...->e->f->...->d
	 * If e = g then insert e in the cycle of d : d->e->f->...->d
	 */
	void phi1_sew(Dart d, Dart e)
	{
		Dart f = phi1(d);
		Dart g = phi1(e);
		(*phi1_)[d.index] = g;
		(*phi1_)[e.index] = f;
		(*phi_1_)[g.index] = d;
		(*phi_1_)[f.index] = e;
	}

	/**
	 * \brief Unlink the successor of a given dart in a permutation
	 * @param d a dart
	 * - Before: d->e->f
	 * - After:  d->f and e->e
	 */
	void phi1_unsew(Dart d)
	{
		Dart e = phi1(d);
		Dart f = phi1(e);
		(*phi1_)[d.index] = f;
		(*phi1_)[e.index] = e;
		(*phi_1_)[f.index] = d;
		(*phi_1_)[e.index] = e;
	}

public:

	CMap1_T() : Inherit()
	{
		init();
	}

	virtual ~CMap1_T() override
	{}

	CMap1_T(Self const&) = delete;
	CMap1_T(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

	/*******************************************************************************
	 * Basic topological operations
	 *******************************************************************************/

	/**
	 * \brief phi1
	 * @param d
	 * @return phi1(d)
	 */
	inline Dart phi1(Dart d) const
	{
		return (*phi1_)[d.index];
	}

	/**
	 * \brief phi_1
	 * @param d
	 * @return phi_1(d)
	 */
	Dart phi_1(Dart d) const
	{
		return (*phi_1_)[d.index];
	}

protected:

	/**
	* \brief add a Dart in the map
	* @return the new Dart
	*/
	inline Dart add_dart()
	{
		unsigned int di = this->add_topology_element();

		Dart d(di);

		(*phi1_)[di] = d;
		(*phi_1_)[di] = d;

		return d;
	}

public:

	/*******************************************************************************
	 * High-level topological operations
	 *******************************************************************************/

	/**
	 * \brief add_face
	 * @param nb_edges
	 * @return
	 */
	Face add_face(unsigned int nb_edges)
	{
		cgogn_message_assert(nb_edges > 0, "Cannot create a face with no edge");

		Dart d = add_face_topo(nb_edges);

		Face f(d);

		if (this->template is_orbit_embedded<Orbit::DART>())
		{
			foreach_incident_vertex(f, [this] (Cell<Orbit::DART> c)
			{
				init_orbit_embedding(c, this->template add_attribute_element<Orbit::DART>());
			});
		}

		if (this->template is_orbit_embedded<Orbit::PHI1>())
			init_orbit_embedding(f, this->template add_attribute_element<Orbit::PHI1>());

		return f;
	}

protected:

	Dart add_face_topo(unsigned int nb_edges)
	{
		cgogn_message_assert(nb_edges > 0, "Cannot create a face with no edge");

		Dart d = this->to_concrete()->add_dart();
		for (unsigned int i = 1; i < nb_edges; ++i)
			cut_edge_topo(d);

		return d;
	}

	/**
	 * \brief cut_edge
	 * @param d
	 * @return
	 */
	Dart cut_edge_topo(Dart d)
	{
		Dart e = this->to_concrete()->add_dart(); // Create a new dart
		phi1_sew(d, e);				// Insert dart e between d and phi1(d)

		// TODO: doit on traiter les marker de bord 2/3 dans Map1
		//		if (this->template is_boundary_marked<2>(d))
		//			this->template boundary_mark<2>(e);

		//		if (this->template is_boundary_marked<3>(d))
		//			this->template boundary_mark<3>(e);

		return e;
	}

protected:

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_dart_of_face(Dart d, const FUNC& f) const
	{
		Dart it = d;
		do
		{
			f(it);
			it = phi1(it);
		} while (it != d);
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		switch (ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1: foreach_dart_of_face(c, f); break;
			case Orbit::PHI2:
			case Orbit::PHI1_PHI2:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI2_PHI3:
			case Orbit::PHI21:
			case Orbit::PHI21_PHI31:
			default: cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}

public:

	/*******************************************************************************
	 * Incidence traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_incident_vertex(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		foreach_dart_of_orbit<FACE>(f, func);
	}

	template <typename FUNC>
	inline void foreach_incident_edge(Face f, const FUNC& func) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		foreach_dart_of_orbit<FACE>(f, func);
	}

	/*******************************************************************************
	 * Adjacence traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_adjacent_vertex_through_edge(Vertex v, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Vertex), "Wrong function cell parameter type");
		f(Vertex(phi1(v.dart)));
		f(Vertex(phi_1(v.dart)));
	}

	template <typename FUNC>
	inline void foreach_adjacent_edge_through_vertex(Edge e, const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Edge), "Wrong function cell parameter type");
		f(Edge(phi1(e.dart)));
		f(Edge(phi_1(e.dart)));
	}

protected:

	/*******************************************************************************
	 * Embedding management
	 *******************************************************************************/

	template <Orbit ORBIT>
	inline void init_orbit_embedding(Cell<ORBIT> c, unsigned int emb)
	{
		foreach_dart_of_orbit(c, [this, emb] (Dart d) { this->template init_embedding<ORBIT>(d, emb); });
	}

	template <Orbit ORBIT>
	inline void set_orbit_embedding(Cell<ORBIT> c, unsigned int emb)
	{
		foreach_dart_of_orbit(c, [this, emb] (Dart d) { this->template set_embedding<ORBIT>(d, emb); });
	}
};

template <typename DataTraits>
struct CMap1TopoTraits
{
	static const int PRIM_SIZE = 1;
	typedef CMap1_T<DataTraits, CMap1TopoTraits<DataTraits>> CONCRETE;
};

using CMap1 = CMap1_T<DefaultDataTraits, CMap1TopoTraits<DefaultDataTraits>>;

} // namespace cgogn

#endif // CORE_MAP_MAP1_H_

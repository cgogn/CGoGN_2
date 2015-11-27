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
class Map1_T : public MapBase<DATA_TRAITS, TOPO_TRAITS>
{
public:

	typedef MapBase<DATA_TRAITS, TOPO_TRAITS> Inherit;
	typedef Map1_T<DATA_TRAITS, TOPO_TRAITS> Self;

	static const unsigned int VERTEX = VERTEX1;
	static const unsigned int EDGE   = VERTEX1;
	static const unsigned int FACE   = FACE2;

	typedef Cell<VERTEX> Vertex;
	typedef Cell<EDGE> Edge;
	typedef Cell<FACE> Face;

	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;
	template<typename T>
	using ChunkArrayContainer = typename Inherit::template ChunkArrayContainer<T>;

	template<typename T, unsigned int ORBIT>
	using AttributeHandler = typename Inherit::template AttributeHandler<T, ORBIT>;
	template<typename T>
	using VertexAttributeHandler = AttributeHandler<T, Self::VERTEX>;
	template<typename T>
	using EdgeAttributeHandler = AttributeHandler<T, Self::EDGE>;
	template<typename T>
	using FaceAttributeHandler = AttributeHandler<T, Self::FACE>;

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

	Map1_T() : Inherit()
	{
		init();
	}

	virtual ~Map1_T() override
	{}

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
	Dart add_face(unsigned int nb_edges)
	{
		cgogn_message_assert(nb_edges > 0, "Cannot create a face with no edge");

		Dart d = add_face_topo(nb_edges);

//		Face f(d);

		if (this->template is_orbit_embedded<VERTEX1>())
		{
//			for (Dart d : incident<VERTEX1>(f))
//				init_orbit_embedding<VERTEX1>(it, this->template add_attribute_element<VERTEX1>());

			Dart it = d;
			do
			{
				init_orbit_embedding<VERTEX1>(it, this->template add_attribute_element<VERTEX1>());
				it = phi1(it);
			} while (it != d);
		}

		if (this->template is_orbit_embedded<FACE2>())
			init_orbit_embedding<FACE2>(d, this->template add_attribute_element<FACE2>());

		return d;
	}

protected:

	Dart add_face_topo(unsigned int nb_edges)
	{
		cgogn_message_assert(nb_edges > 0, "Cannot create a face with no edge");

		Dart d = static_cast<typename TOPO_TRAITS::CONCRETE*>(this)->add_dart();
		for (unsigned int i = 1 ; i < nb_edges ; ++i)
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
		Dart e = static_cast<typename TOPO_TRAITS::CONCRETE*>(this)->add_dart(); // Create a new dart
		phi1_sew(d, e);				// Insert dart e between d and phi1(d)

		// TODO: doit on traiter les marker de bord 2/3 dans Map1
//		if (this->template is_boundary_marked<2>(d))
//			this->template boundary_mark<2>(e);

//		if (this->template is_boundary_marked<3>(d))
//			this->template boundary_mark<3>(e);

		return e;
	}

public:

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_dart_of_vertex(Dart d, const FUNC& f) const
	{
		f(d);
	}

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

	template <unsigned int ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		switch (ORBIT)
		{
			case VERTEX1: foreach_dart_of_vertex(c, f); break;
			case FACE2:   foreach_dart_of_face(c, f); break;
			default:      cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}

	/*******************************************************************************
	 * Embedding management
	 *******************************************************************************/

	template <unsigned int ORBIT>
	inline void init_orbit_embedding(Cell<ORBIT> c, unsigned int emb)
	{
		foreach_dart_of_orbit(c, [this, emb] (Dart d) { this->template init_embedding<ORBIT>(d, emb); });
	}

	template <unsigned int ORBIT>
	inline void set_orbit_embedding(Cell<ORBIT> c, unsigned int emb)
	{
		foreach_dart_of_orbit(c, [this, emb] (Dart d) { this->template set_embedding<ORBIT>(d, emb); });
	}
};

template <typename DataTraits>
struct Map1TopoTraits
{
	static const int PRIM_SIZE = 1;
	typedef Map1_T<DataTraits, Map1TopoTraits<DataTraits>> CONCRETE;
};

template <typename DataTraits>
using Map1 = Map1_T<DataTraits, Map1TopoTraits<DataTraits>>;

} // namespace cgogn

#endif // CORE_MAP_MAP1_H_

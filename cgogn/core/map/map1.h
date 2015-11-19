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

class Topo_Traits_Map1
{
	static const int PRIM_SIZE = 1;
};

template <typename DATA_TRAITS>
class Map1 : public MapBase<DATA_TRAITS, Topo_Traits_Map1>
{
public:
	typedef MapBase<DATA_TRAITS, Topo_Traits_Map1> Inherit;
	typedef Map1<DATA_TRAITS> Super;
	template<typename T>
	using ChunkArray =  typename Inherit::template ChunkArray<T>;
	template<typename T>
	using ChunkArrayContainer =  typename Inherit::template ChunkArrayContainer<T>;

	static const unsigned int VERTEX = VERTEX1;
	static const unsigned int EDGE   = VERTEX1;
	static const unsigned int FACE   = FACE2;

	template<typename T, unsigned int ORBIT>
	using AttributeHandler = typename Inherit::template AttributeHandler<T, ORBIT>;
	template<typename T>
	using VertexAttributeHandler = AttributeHandler<T, VERTEX>;

	template<typename T>
	using EdgeAttributeHandler = AttributeHandler<T, EDGE>;

	template<typename T>
	using FaceAttributeHandler = AttributeHandler<T, FACE>;

protected:

	void init()
	{
		ChunkArray<Dart>* phi1 = this->topology_.template add_attribute<Dart>("phi1");
		ChunkArray<Dart>* phi_1 = this->topology_.template add_attribute<Dart>("phi_1");
		this->topo_relations_.push_back(phi1);
		this->topo_relations_.push_back(phi_1);
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
		(*(this->topo_relations_[0]))[d.index] = g;
		(*(this->topo_relations_[0]))[e.index] = f;
		(*(this->topo_relations_[1]))[g.index] = d;
		(*(this->topo_relations_[1]))[f.index] = e;
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
		(*(this->topo_relations_[0]))[d.index] = f;
		(*(this->topo_relations_[0]))[e.index] = e;
		(*(this->topo_relations_[1]))[f.index] = d;
		(*(this->topo_relations_[1]))[e.index] = e;
	}

public:

	Map1() : Inherit()
	{
		init();
	}

	virtual ~Map1() override
	{}

	/*******************************************************************************
	 * Basic topological operations
	 *******************************************************************************/

	/**
	 * \brief phi1
	 * @param d
	 * @return
	 */
	inline Dart phi1(Dart d) const
	{
		return (*(this->topo_relations_[0]))[d.index];
	}

	/**
	 * \brief phi_1
	 * @param d
	 * @return
	 */
	Dart phi_1(Dart d) const
	{
		return (*(this->topo_relations_[1]))[d.index];
	}

	/**
	* \brief add a Dart in the map
	* @return the new Dart
	*/
	inline Dart add_dart()
	{
		unsigned int di = this->topology_.template insert_lines<1>();	// insert a new dart line
		this->topology_.init_markers_of_line(di);

		for(unsigned int i = 0; i < NB_ORBITS; ++i)
		{
			if (this->embeddings_[i])							// set all its embeddings
				(*(this->embeddings_[i]))[di] = EMBNULL;		// to EMBNULL
		}

		Dart d(di);

		for (auto ptr : this->topo_relations_)
			(*ptr)[di] = d;

		return d;
	}

	/*******************************************************************************
	 * High-level topological operations
	 *******************************************************************************/

	/**
	 * \brief add_cycle
	 * @param nbEdges
	 * @return
	 */
	Dart add_cycle(unsigned int nbEdges);

	/**
	 * \brief remove_cycle
	 * @param d
	 */
	void remove_cycle(Dart d);

	/**
	 * \brief cut_edge
	 * @param d
	 * @return
	 */
	Dart cut_edge(Dart d)
	{
		Dart e = this->new_dart();	// Create a new dart
		phi1_sew(d, e);				// Insert dart e between d and phi1(d)

		// TODO: doit on traiter les marker de bord 2/3 dans Map1
		if (this->template is_boundary_marked<2>(d))
			this->template boundary_mark<2>(e);

		if (this->template is_boundary_marked<3>(d))
			this->template boundary_mark<3>(e);

		return e;
	}

	/**
	 * \brief uncut_edge
	 * @param d
	 * @return
	 */
	void uncut_edge(Dart d)
	{
		Dart d1 = phi1(d);
		phi1_unsew(d);			// Dart d is linked to the successor of its successor
		this->delete_dart(d1);	// Dart d1 is erased
	}

	/**
	 * \brief collapse_edge
	 * @param d
	 * @return
	 */
	Dart collapse_edge(Dart d);

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	template <typename FUNC>
	inline void foreach_dart_of_vertex(Dart d, const FUNC& f) const
	{
		f(d);
	}

	template <typename FUNC>
	inline void foreach_dart_of_edge(Dart d, const FUNC& f) const
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
	void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		switch(ORBIT)
		{
			case Map1::VERTEX: f(c); break;
			case Map1::EDGE:   foreach_dart_of_edge(c, f); break;
			case Map1::FACE:   foreach_dart_of_face(c, f); break;
			default: cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}
};

} // namespace cgogn

#endif // CORE_MAP_MAP1_H_

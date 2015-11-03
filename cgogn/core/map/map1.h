/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *                                                                  *                                                                              *
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

#ifndef __CORE_MAP_MAP1_H__
#define __CORE_MAP_MAP1_H__

#include "core/map/map_base.h"
#include "core/basic/dart.h"

namespace cgogn
{

class Topo_Traits_map1
{
//	static const int CHUNK_SIZE=4096;
	static const int PRIM_SIZE=1;
};


template <typename DATA_TRAITS>
class Map1: public MapBase<DATA_TRAITS, Topo_Traits_map1>
{
public:

	static const unsigned int VERTEX	= VERTEX2;
	static const unsigned int EDGE		= EDGE2;
	static const unsigned int FACE		= FACE2;

	template<typename T>
	using VertexAttributeHandler =  AttributeHandler<DATA_TRAITS,T,VERTEX>;

	template<typename T>
	using EdgeAttributeHandler =  AttributeHandler<DATA_TRAITS,T,EDGE>;

	template<typename T>
	using FaceAttributeHandler =  AttributeHandler<DATA_TRAITS,T,FACE>;


	/**
	 * @brief phi1
	 * @param d
	 * @return
	 */
	Dart phi1(Dart d) const
	{
		// phi1 first topo relation
		return (*(this->topo_relations_[0]))[d.index];
	}

	/**
	 * @brief phi_1
	 * @param d
	 * @return
	 */
	Dart phi_1(Dart d) const
	{
		// phi1 first topo relation
		return (*(this->topo_relations_[1]))[d.index];
	}


	/**
	 * @brief add_cycle
	 * @param nbEdges
	 * @return
	 */
	Dart add_cycle(unsigned int nbEdges) ;

	/**
	 * @brief remove_cycle
	 * @param d
	 */
	void remove_cycle(Dart d) ;

	/**
	 * @brief cut_edge
	 * @param d
	 * @return
	 */
	Dart cut_edge(Dart d)
	{
		Dart e = this->newDart() ;	// Create a new dart
		phi1sew(d, e) ;				// Insert dart e between d and phi1(d)


		// TODO: doit on traiter les marker de bord 2/3 dans Map1
		if (this->template isBoundaryMarked<2>(d))
			this->template boundaryMark<2>(e);

		if (this->template isBoundaryMarked<3>(d))
			this->template boundaryMark<3>(e);

		return e ;
	}

	/**
	 * @brief uncut_edge
	 * @param d
	 * @return
	 */
	void uncut_edge(Dart d)
	{
		Dart d1 = phi1(d) ;
		phi1unsew(d) ;			// Dart d is linked to the successor of its successor
		this->deleteDart(d1) ;	// Dart d1 is erased
	}


	/**
	 * @brief collapse_edge
	 * @param d
	 * @return
	 */
	Dart collapse_edge(Dart d);



protected:

	void init()
	{
		ChunkArray<DATA_TRAITS::CHUNK_SIZE,Dart>* rel_phi1 = this->topology_.template addAttribute<Dart>("phi1");
		ChunkArray<DATA_TRAITS::CHUNK_SIZE,Dart>* rel_phi_1 = this->topology_.template addAttribute<Dart>("phi1");

		this->topo_relations_.push_back(rel_phi1);
		this->topo_relations_.push_back(rel_phi_1);
	}


	//! Link the current dart to dart d with a permutation
	/*! @param d the dart to which the current is linked
	 * - Before:	d->f and e->g
	 * - After:		d->g and e->f
	 * Join the permutations cycles of dart d and e
	 * - Starting from two cycles : d->f->...->d and e->g->...->e
	 * - It makes one cycle d->g->...->e->f->...->d
	 * If e = g then insert e in the cycle of d : d->e->f->...->d
	 */
	void phi1sew(Dart d, Dart e);

	//! Unlink the successor of a given dart in a permutation
	/*!	@param d a dart
	 * - Before:	d->e->f
	 * - After:		d->f and e->e
	 */
	void phi1unsew(Dart d);


};



}
#endif

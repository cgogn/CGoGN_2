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

#ifndef __CORE_MAP_MAP2_H__
#define __CORE_MAP_MAP2_H__

#include "core/map/map1.h"

namespace cgogn
{

template <typename DATA_TRAITS>
class Map2 : public Map1<DATA_TRAITS>
{
public:

	static const unsigned int VERTEX	= VERTEX2;
	static const unsigned int EDGE		= EDGE2;
	static const unsigned int FACE		= FACE2;

	template<typename T>
	using VertexAttributeHandler = cgogn::AttributeHandler<DATA_TRAITS, T, VERTEX>;

	template<typename T>
	using EdgeAttributeHandler = cgogn::AttributeHandler<DATA_TRAITS, T, EDGE>;

	template<typename T>
	using FaceAttributeHandler = cgogn::AttributeHandler<DATA_TRAITS, T, FACE>;

protected:

	void init()
	{
		ChunkArray<DATA_TRAITS::CHUNK_SIZE, Dart>* phi2 = this->topology_.template addAttribute<Dart>("phi2");
		this->topo_relations_.push_back(phi2);
	}

	//! Link dart d with dart e by an involution
	/*  @param d,e the darts to link
	 *	- Before:	d->d and e->e
	 *	- After:	d->e and e->d
	 */
	void phi2sew(Dart d, Dart e)
	{
		assert(phi2(d) == d);
		assert(phi2(e) == e);
		(*(this->topo_relations_[2]))[d.index] = e;
		(*(this->topo_relations_[2]))[e.index] = d;
	}

	//! Unlink the current dart by an involution
	/*  @param d the dart to unlink
	 * - Before:	d->e and e->d
	 * - After:		d->d and e->e
	 */
	void phi2unsew(Dart d)
	{
		Dart e = phi2(d) ;
		(*(this->topo_relations_[2]))[d.index] = d;
		(*(this->topo_relations_[2]))[e.index] = e;
	}

public:

	Map2() : Map1<DATA_TRAITS>()
	{
		init();
	}

	/**
	 * @brief phi2
	 * @param d
	 * @return
	 */
	Dart phi2(Dart d) const
	{
		// phi2 first topo relation
		return (*(this->topo_relations_[2]))[d.index];
	}
};

} // namespace cgogn

#endif // __CORE_MAP_MAP2_H__

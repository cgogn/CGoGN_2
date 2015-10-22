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

#ifndef __CORE_MAP_MAP_BASE_DATA_H__
#define __CORE_MAP_MAP_BASE_DATA_H__

#include "core/container/chunk_array_container.h"
#include "core/basic/cell.h"

namespace cgogn
{


/**
 * @brief Generic Map class for SCHNApps
 */
class MapGen
{
};



/**
 * @brief The MapBase class
 */
template<typename DATA_TRAITS>
class MapBaseData: public MapGen
{
protected:

	/// topooly & embedding indices
	ChunkArrayContainer<DATA_TRAITS::CHUNK_SIZE, unsigned char> topology_;

	/// embedding attributes
	ChunkArrayContainer<DATA_TRAITS::CHUNK_SIZE, unsigned int> attributes_[NB_ORBITS];

	ChunkArray<DATA_TRAITS::CHUNK_SIZE,unsigned int> embeddings_[NB_ORBITS];

public:
	MapBaseData() {}

	ChunkArrayContainer<DATA_TRAITS::CHUNK_SIZE, unsigned int>& getAttributeContainer(unsigned int orbit)
	{
		return attributes_[orbit];
	}


	template <unsigned int ORBIT>
	inline unsigned int getEmbedding(Cell<ORBIT> c) const
	{
//		assert(this->template isOrbitEmbedded<ORBIT>() || !"Invalid parameter: orbit not embedded");
		return (*this->embeddings_[ORBIT])[this->dartIndex(c.dart)] ;
	}

};

}

#endif

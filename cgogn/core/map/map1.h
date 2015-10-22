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

	template<typename T, unsigned int ORBIT>
	using AttributeHandler =  AttributeHandler<DATA_TRAITS,T,ORBIT>;


};



}
#endif

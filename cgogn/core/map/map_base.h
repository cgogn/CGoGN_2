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

#ifndef __CORE_MAP_MAP_BASE_H__
#define __CORE_MAP_MAP_BASE_H__

#include "core/map/map_base_data.h"
#include "core/map/attribute_handler.h"

namespace cgogn
{



template<typename DATA_TRAITS, typename TOPO_TRAITS>
class MapBase: public MapBaseData<DATA_TRAITS>
{
protected:
	std::multimap<ChunkArrayGen<DATA_TRAITS::CHUNK_SIZE>*, AttributeHandlerGen<DATA_TRAITS>*> attributeHandlers_ ;

public:

	//TODO remonter dans map_base
	template<typename T, unsigned int ORBIT>
	using AttributeHandler =  AttributeHandler<DATA_TRAITS,T,ORBIT>;


	MapBase() {}

	template <typename T, unsigned int ORBIT>
	inline AttributeHandler<T,ORBIT> addAttribute(const std::string& nameAttr = "")
	{
//		if(!this->template isOrbitEmbedded<ORBIT>())
//			this->template addEmbedding<ORBIT>() ;

		ChunkArray<DATA_TRAITS::CHUNK_SIZE,T>* ca = this->attributes_[ORBIT].template addAttribute<T>(nameAttr) ;
		return AttributeHandler<T, ORBIT>(this, ca) ;
	}

	/**
	 * remove an attribute
	 * @param attr a handler to the attribute to remove
	 * @return true if remove succeed else false
	 */
	template <typename T, unsigned int ORBIT>
	inline bool removeAttribute(AttributeHandler<T,ORBIT>& attr_handl)
	{

		ChunkArray<DATA_TRAITS::CHUNK_SIZE, T>* ca =  attr_handl.getDataVector();

		if (this->attributes_[ORBIT].removeAttribute(ca))
		{
			typedef typename std::multimap<ChunkArrayGen<DATA_TRAITS::CHUNK_SIZE>*, AttributeHandlerGen<DATA_TRAITS>*>::iterator IT ;
			std::pair<IT, IT> bounds = attributeHandlers_.equal_range(ca) ;
			for(IT i = bounds.first; i != bounds.second; ++i)
				(*i).second->setInvalid() ;
			attributeHandlers_.erase(bounds.first, bounds.second) ;
			return true ;
		}
		return false ;
	}

	/**
	* search an attribute for a given orbit
	* @param nameAttr attribute name
	* @return an AttributeHandler
	*/
	template <typename T, unsigned int ORBIT>
	inline AttributeHandler< T, ORBIT> getAttribute(const std::string& nameAttr)
	{
		ChunkArray<DATA_TRAITS::CHUNK_SIZE,T>* ca = this->attribs_[ORBIT].template getAttribute<T>(nameAttr) ;
			return AttributeHandler<T, ORBIT>(this, ca) ;
	}


	inline Dart add_dart()
	{

		unsigned int di = this->topology_.template insertLines<1>();	// insert a new dart line
		 this->topology_.initBooleansOfLine(di);

		for(unsigned int i = 0; i < NB_ORBITS; ++i)
		{
			if (this->embeddings_[i])							// set all its embeddings
				(*(this->embeddings_[i]))[di] = EMBNULL ;			// to EMBNULL
		}

		Dart d = Dart::create(di) ;

		for (auto relPtr: this->topo_relations_)
			(*relPtr)[di] = d;

		return d ;
	}

};

}

#endif

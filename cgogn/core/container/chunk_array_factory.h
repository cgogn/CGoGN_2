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

#ifndef CORE_CONTAINER_CHUNK_ARRAY_FACTORY_H_
#define CORE_CONTAINER_CHUNK_ARRAY_FACTORY_H_

#include <core/basic/nameTypes.h>
#include <core/container/chunk_array.h>

#include <iostream>
#include <map>
#include <memory>
#include <utils/make_unique.h>
namespace cgogn
{

template <unsigned int CHUNKSIZE>
class ChunkArrayFactory
{
public:
	typedef std::unique_ptr< ChunkArrayGen<CHUNKSIZE> > ChunkArrayGenPtr;
	typedef std::map<std::string, ChunkArrayGenPtr > Map;

	static Map mapCA_;

	/**
	 * @brief register a type
	 * @param keyType name of type
	 * @param obj a ptr on object (new ChunkArray<32,int> for example) ptr will be deleted by clean method
	 */
	template<typename T>
	static void registerCA()
	{
        std::string&& keyType(nameOfType(T()));
		if(mapCA_.find(keyType) == mapCA_.end())
            mapCA_[std::move(keyType)] =  make_unique<ChunkArray<CHUNKSIZE, T>>();
	}

	/**
	 * @brief create a ChunkArray from a typename
	 * @param keyType typename of type store in ChunkArray
	 * @return ptr on created ChunkArray
	 */
	static ChunkArrayGen<CHUNKSIZE>* create(const std::string& keyType)
	{
		ChunkArrayGen<CHUNKSIZE>* tmp = nullptr;
		typename Map::const_iterator it = mapCA_.find(keyType);

		if(it != mapCA_.end())
		{
			tmp = (it->second)->clone();
		}
		else
			std::cerr << "type " << keyType << " not registred in ChunkArrayFactory" << std::endl;

		return tmp;
	}
};

template <unsigned int CHUNKSIZE>
typename ChunkArrayFactory<CHUNKSIZE>::Map ChunkArrayFactory<CHUNKSIZE>::mapCA_= typename ChunkArrayFactory<CHUNKSIZE>::Map();

} // namespace cgogn

#endif // CORE_CONTAINER_CHUNK_ARRAY_FACTORY_H_

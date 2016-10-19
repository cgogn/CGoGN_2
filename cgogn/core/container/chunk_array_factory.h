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

#ifndef CGOGN_CORE_CONTAINER_CHUNK_ARRAY_FACTORY_H_
#define CGOGN_CORE_CONTAINER_CHUNK_ARRAY_FACTORY_H_

#include <cgogn/core/utils/logger.h>
#include <cgogn/core/utils/unique_ptr.h>
#include <cgogn/core/utils/name_types.h>
#include <cgogn/core/container/chunk_array.h>

#include <cgogn/core/cmap/map_traits.h>

#include <iostream>
#include <map>
#include <memory>
#include <array>

namespace cgogn
{


class ChunkArrayFactory
{
public:

	using Self = ChunkArrayFactory;
	using ChunkArrayGenPtr = std::unique_ptr< ChunkArrayGen >;
	using NamePtrMap = std::map<std::string, ChunkArrayGenPtr>;
	using UniqueNamePtrMap = std::unique_ptr<NamePtrMap>;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ChunkArrayFactory);

	static UniqueNamePtrMap map_CA_;

	/**
	 * @brief register a type
	 * @param keyType name of type
	 * @param obj a ptr on object (new ChunkArray<32,int> for example) ptr will be deleted by clean method
	 */
	template <typename T>
	static void register_CA()
	{
		// could be moved in register_known_types (dangerous) ??
		if (!map_CA_)
			reset();

		std::string&& keyType(name_of_type(T()));
		if(map_CA_->find(keyType) == map_CA_->end())
			(*map_CA_)[std::move(keyType)] = make_unique<ChunkArray<T>>();
	}

	static void register_known_types();

	/**
	 * @brief create a ChunkArray from a typename
	 * @param keyType typename of type store in ChunkArray
	 * @return ptr on created ChunkArray
	 */
	static ChunkArrayGenPtr create(const std::string& type_name, const std::string& name);

	static void reset();

private:

	inline ChunkArrayFactory() {}
};

} // namespace cgogn

#endif // CGOGN_CORE_CONTAINER_CHUNK_ARRAY_FACTORY_H_

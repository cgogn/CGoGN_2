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

template <uint32 CHUNK_SIZE>
class ChunkArrayFactory
{
	static_assert(CHUNK_SIZE >= 1u, "ChunkSize must be at least 1");
	static_assert(!(CHUNK_SIZE & (CHUNK_SIZE - 1)), "CHUNK_SIZE must be a power of 2");

public:

	using Self = ChunkArrayFactory<CHUNK_SIZE>;
	using ChunkArrayGenPtr = std::unique_ptr< ChunkArrayGen<CHUNK_SIZE> >;
	using NamePtrMap = std::map<std::string, ChunkArrayGenPtr>;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ChunkArrayFactory);

	/**
	 * @brief register a type
	 * @param keyType name of type
	 * @param obj a ptr on object (new ChunkArray<32,int> for example) ptr will be deleted by clean method
	 */
	template <typename T>
	static void register_CA()
	{
		std::string&& keyType(name_of_type(T()));
		if(map_CA_.find(keyType) == map_CA_.end())
			map_CA_[std::move(keyType)] = make_unique<ChunkArray<CHUNK_SIZE, T>>();
	}

	static void register_known_types()
	{
		static bool known_types_initialized_ = false;

		if (known_types_initialized_)
			return;

		register_CA<bool>();
		register_CA<char>();
		register_CA<int8>();
		register_CA<int16>();
		register_CA<int32>();
		register_CA<int64>();
		register_CA<uint8>();
		register_CA<uint16>();
		register_CA<uint32>();
		register_CA<uint64>();
		register_CA<float32>();
		register_CA<float64>();
		register_CA<std::string>();
		register_CA<std::array<float32, 3>>();
		register_CA<std::array<float64, 3>>();
		// NOT TODO : add Eigen.

		known_types_initialized_ = true;
	}

	/**
	 * @brief create a ChunkArray from a typename
	 * @param keyType typename of type store in ChunkArray
	 * @return ptr on created ChunkArray
	 */
	static ChunkArrayGenPtr create(const std::string& type_name, const std::string& name)
	{
		ChunkArrayGenPtr tmp = nullptr;
		typename NamePtrMap::const_iterator it = map_CA_.find(type_name);

		if(it != map_CA_.end())
			tmp = (it->second)->clone(name);
		else
			cgogn_log_warning("ChunkArrayFactory::create") << "Type \"" << type_name << "\" is not registered in ChunkArrayFactory.";

		return tmp;
	}

	static void reset()
	{
		map_CA_.clear();
	}

private:

	static NamePtrMap map_CA_;
	inline ChunkArrayFactory() {}
};

template <uint32 CHUNK_SIZE>
typename ChunkArrayFactory<CHUNK_SIZE>::NamePtrMap ChunkArrayFactory<CHUNK_SIZE>::map_CA_;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CONTAINER_CHUNK_ARRAY_FACTORY_CPP_))
extern template class CGOGN_CORE_API ChunkArrayFactory<CGOGN_CHUNK_SIZE>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CONTAINER_CHUNK_ARRAY_FACTORY_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CONTAINER_CHUNK_ARRAY_FACTORY_H_

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

#ifndef CGOGN_CORE_CMAP_ATTRIBUTE_FACTORY_H_
#define CGOGN_CORE_CMAP_ATTRIBUTE_FACTORY_H_

#include <map>
#include <memory>
#include <array>

#include <cgogn/core/container/chunk_array_gen.h>

#include <cgogn/core/utils/name_types.h>
#include <cgogn/core/cmap/attribute.h>
#include <cgogn/core/cmap/map_traits.h>

namespace cgogn
{

template <typename DATA_TRAITS>
class AttributeFactory
{

public:

	using Self = AttributeFactory<DATA_TRAITS>;
	using AttributeGen = cgogn::AttributeGen<DATA_TRAITS>;
	using NamePtrMap = std::map<std::string, std::unique_ptr<AttributeGen>>;
	template<typename T, Orbit ORBIT>
	using Attribute = cgogn::Attribute<DATA_TRAITS, T, ORBIT>;
	using MapBaseData = cgogn::MapBaseData<DATA_TRAITS>;
	using ChunkArrayGen = cgogn::ChunkArrayGen<DATA_TRAITS::CHUNK_SIZE>;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(AttributeFactory);

	static std::array<NamePtrMap, NB_ORBITS> attribute_map_;

	/**
	 * @brief register a type
	 * @param keyType name of type
	 * @param obj a ptr on object (new ChunkArray<32,int> for example) ptr will be deleted by clean method
	 */
	template <typename T>
	static void register_attribute()
	{

		Self::register_known_types();

		std::string keyType(name_of_type(T()));
		if(attribute_map_[0].find(keyType) == attribute_map_[0].end())
		{
			(attribute_map_[Orbit::DART][keyType]).reset(new Attribute<T,Orbit::DART>());
			(attribute_map_[Orbit::PHI1][keyType]).reset(new Attribute<T,Orbit::PHI1>());
			(attribute_map_[Orbit::PHI2][keyType]).reset(new Attribute<T,Orbit::PHI2>());
			(attribute_map_[Orbit::PHI1_PHI2][keyType]).reset(new Attribute<T,Orbit::PHI1_PHI2>());
			(attribute_map_[Orbit::PHI1_PHI3][keyType]).reset(new Attribute<T,Orbit::PHI1_PHI3>());
			(attribute_map_[Orbit::PHI2_PHI3][keyType]).reset(new Attribute<T,Orbit::PHI2_PHI3>());
			(attribute_map_[Orbit::PHI21][keyType]).reset(new Attribute<T,Orbit::PHI21>());
			(attribute_map_[Orbit::PHI21_PHI31][keyType]).reset(new Attribute<T,Orbit::PHI21_PHI31>());
			(attribute_map_[Orbit::PHI1_PHI2_PHI3][std::move(keyType)]).reset(new Attribute<T,Orbit::PHI1_PHI2_PHI3>());
		}

	}

	static void register_known_types()
	{
		static bool known_types_initialized_ = false;

		if (known_types_initialized_)
			return;
		known_types_initialized_ = true;
		register_attribute<bool>();
		register_attribute<char>();
		register_attribute<int8>();
		register_attribute<int16>();
		register_attribute<int32>();
		register_attribute<int64>();
		register_attribute<uint8>();
		register_attribute<uint16>();
		register_attribute<uint32>();
		register_attribute<uint64>();
		register_attribute<float32>();
		register_attribute<float64>();
		register_attribute<std::string>();
		register_attribute<std::array<float32, 3>>();
		register_attribute<std::array<float64, 3>>();
		// NOT TODO : add Eigen.

		known_types_initialized_ = true;
	}

	/**
	 * @brief create a ChunkArray from a typename
	 * @param keyType typename of type store in ChunkArray
	 * @return ptr on created ChunkArray
	 */
	static std::unique_ptr<AttributeGen> create(Orbit orb, MapBaseData* mapbd, ChunkArrayGen* cag)
	{

		std::unique_ptr<AttributeGen> res;
		typename NamePtrMap::const_iterator it = attribute_map_[orb].find(cag->get_type_name());

		if(it != attribute_map_[orb].end())
			res = (it->second)->clone(mapbd, cag);
		else
			cgogn_log_warning("AttributeFactory::create") << "Type \"" << cag->get_type_name() << "\" is not registered in AttributeFactory.";

		return res;
	}

private:
	inline AttributeFactory() {}

};

template <typename DATA_TRAITS>
std::array<typename AttributeFactory<DATA_TRAITS>::NamePtrMap, NB_ORBITS> AttributeFactory<DATA_TRAITS>::attribute_map_;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CMAP_ATTRIBUTE_FACTORY_CPP_))
extern template class CGOGN_CORE_API AttributeFactory<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CMAP_ATTRIBUTE_FACTORY_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CMAP_ATTRIBUTE_FACTORY_H_

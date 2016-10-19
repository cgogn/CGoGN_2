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

#define CGOGN_CORE_CONTAINER_CHUNK_ARRAY_FACTORY_CPP_

#include <cgogn/core/container/chunk_array_factory.h>

namespace cgogn
{

ChunkArrayFactory::UniqueNamePtrMap ChunkArrayFactory::map_CA_ = nullptr;

void ChunkArrayFactory::register_known_types()
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

ChunkArrayFactory::ChunkArrayGenPtr ChunkArrayFactory::create(const std::__cxx11::string& type_name, const std::__cxx11::string& name)
{
	ChunkArrayGenPtr tmp = nullptr;
	typename NamePtrMap::const_iterator it = map_CA_->find(type_name);

	if(it != map_CA_->end())
		tmp = (it->second)->clone(name);
	else
		cgogn_log_warning("ChunkArrayFactory::create") << "Type \"" << type_name << "\" is not registered in ChunkArrayFactory.";

	return tmp;
}

void ChunkArrayFactory::reset()
{
	ChunkArrayFactory::map_CA_ = make_unique<NamePtrMap>();
}


} // namespace cgogn

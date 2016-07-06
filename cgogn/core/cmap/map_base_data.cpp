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

#define CGOGN_CORE_DLL_EXPORT
#define CGOGN_CORE_MAP_MAP_BASE_DATA_CPP_

#include <cgogn/core/cmap/map_base_data.h>

namespace cgogn
{

std::vector<MapGen*>* MapGen::instances_ = nullptr;
bool MapGen::init_CA_factory = true;

std::array<int,12> MapGen::tetra_phi2 = {3,5,7,-3,7,2,-5,-2,2,-7,-2,-7};

std::array<int,24> MapGen::hexa_phi2 = {4,7,10,13, -4,14,17,2, -7,-2,12,2, -10,-2,7,2, -13,-2,2,-14, -2,-7,-12,-17};



MapGen::MapGen()
{
	if (instances_ == nullptr)
	{
		cgogn::thread_start();
		instances_ = new std::vector<MapGen*>;
	}

	cgogn_message_assert(std::find(instances_->begin(), instances_->end(), this) == instances_->end(), "This map is already present in the instances vector");

	// register the map in the vector of instances
	instances_->push_back(this);
}

MapGen::~MapGen()
{
	// remove the map from the vector of instances
	auto it = std::find(instances_->begin(), instances_->end(), this);
	*it = instances_->back();
	instances_->pop_back();

	if (instances_->empty())
	{
		cgogn::thread_stop();
		delete instances_;
		instances_ = nullptr;
	}
}

template class CGOGN_CORE_API MapBaseData<DefaultMapTraits>;

} // namespace cgogn

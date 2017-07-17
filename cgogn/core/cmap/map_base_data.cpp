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

#define CGOGN_CORE_MAP_MAP_BASE_DATA_CPP_

#include <cgogn/core/cmap/map_base_data.h>

namespace cgogn
{

std::vector<const MapBaseData*>* MapBaseData::instances_ = nullptr;
// tetra_phi2 = {3,5,7,-3,7,2,-5,-2,2,-7,-2,-7}
const std::array<uint32, 12> MapBaseData::tetra_phi2 = {3,5,7,uint32(-3),7,2,uint32(-5),uint32(-2),2,uint32(-7),uint32(-2),uint32(-7)};
// hexa_phi2 = {4,7,10,13, -4,14,17,2, -7,-2,12,2, -10,-2,7,2, -13,-2,2,-14, -2,-7,-12,-17}
const std::array<uint32, 24> MapBaseData::hexa_phi2 = {4,7,10,13, uint32(-4),14,17,2, uint32(-7),uint32(-2),12,2, uint32(-10),uint32(-2),7,2, uint32(-13),uint32(-2),2,uint32(-14), uint32(-2),uint32(-7),uint32(-12),uint32(-17)};

MapBaseData::MapBaseData()
{
	if (instances_ == nullptr)
	{
		cgogn::thread_start(0,0);
		instances_ = new std::vector<const MapBaseData*>;
	}

	// register the map in the vector of instances
	cgogn_assert(std::find(instances_->begin(), instances_->end(), this) == instances_->end());
	instances_->push_back(this);

	uint32 nb_mark_threads = thread_pool()->max_nb_workers() + external_thread_pool()->max_nb_workers() + 1; // +1 for main thread

	for (uint32 i = 0u; i < NB_ORBITS; ++i)
	{
		mark_attributes_[i].resize(nb_mark_threads);

		embeddings_[i] = nullptr;
		for (uint32 j = 0u; j < nb_mark_threads; ++j)
			mark_attributes_[i][j].reserve(8u);
	}

	mark_attributes_topology_.resize(nb_mark_threads);

	for (uint32 i = 0u; i < nb_mark_threads; ++i)
		mark_attributes_topology_[i].reserve(8u);

	boundary_marker_ = topology_.add_marker_attribute();
}

MapBaseData::~MapBaseData()
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

} // namespace cgogn

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
		cgogn::thread_start();
		instances_ = new std::vector<const MapBaseData*>;
	}

	// register the map in the vector of instances
	cgogn_assert(std::find(instances_->begin(), instances_->end(), this) == instances_->end());
	instances_->push_back(this);

	for (uint32 i = 0u; i < NB_ORBITS; ++i)
	{
		mark_attributes_[i].reserve(NB_UNKNOWN_THREADS + 2u*MAX_NB_THREADS);
		mark_attributes_[i].resize(NB_UNKNOWN_THREADS + MAX_NB_THREADS);

		embeddings_[i] = nullptr;
		for (uint32 j = 0u; j < NB_UNKNOWN_THREADS + MAX_NB_THREADS; ++j)
			mark_attributes_[i][j].reserve(8u);
	}

	mark_attributes_topology_.reserve(NB_UNKNOWN_THREADS + 2u*MAX_NB_THREADS);
	mark_attributes_topology_.resize(NB_UNKNOWN_THREADS + MAX_NB_THREADS);

	for (uint32 i = 0u; i < MAX_NB_THREADS; ++i)
		mark_attributes_topology_[i].reserve(8u);

	boundary_marker_ = topology_.add_marker_attribute();

	thread_ids_.reserve(NB_UNKNOWN_THREADS + 2u*MAX_NB_THREADS);
	thread_ids_.resize(NB_UNKNOWN_THREADS);

	this->add_thread(std::this_thread::get_id());
	const auto& pool_threads_ids = cgogn::thread_pool()->threads_ids();
	for (const std::thread::id& ids : pool_threads_ids)
		this->add_thread(ids);
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

uint32 MapBaseData::add_unknown_thread() const
{
	static uint32 index = 0u;
	const std::thread::id& th_id = std::this_thread::get_id();
	cgogn_log_warning("add_unknown_thread") << "Registration of an unknown thread (id :" << th_id << ") in the map. Data can be lost. Please use add_thread and remove_thread interface.";
	thread_ids_[index] = th_id;
	const unsigned old_index = index;
	index = (index+1u) % NB_UNKNOWN_THREADS;
	return old_index;
}

std::size_t MapBaseData::unknown_thread_index(std::thread::id thread_id) const
{
	auto end = thread_ids_.begin();
	std::advance(end, NB_UNKNOWN_THREADS);
	auto res_it = std::find(thread_ids_.begin(), end, thread_id);
	if (res_it != end)
		return std::size_t(std::distance(thread_ids_.begin(), res_it));

	return add_unknown_thread();
}

std::size_t MapBaseData::current_thread_index() const
{
	// avoid the unknown threads stored at the beginning of the vector
	auto real_begin = thread_ids_.begin();
	std::advance(real_begin, NB_UNKNOWN_THREADS);

	const auto end = thread_ids_.end();
	auto it_lower_bound = std::lower_bound(real_begin, end, std::this_thread::get_id());
	if (it_lower_bound != end)
		return std::size_t(std::distance(thread_ids_.begin(), it_lower_bound));

	return unknown_thread_index(std::this_thread::get_id());
}

void MapBaseData::remove_thread(std::thread::id thread_id) const
{
	// avoid the unknown threads stored at the beginning of the vector
	auto real_begin = thread_ids_.begin();
	std::advance(real_begin, NB_UNKNOWN_THREADS);

	cgogn_message_assert(std::binary_search(real_begin, thread_ids_.end(), thread_id), "Unable to find the thread.");
	auto it = std::lower_bound(real_begin, thread_ids_.end(), thread_id);
	cgogn_message_assert(*it == thread_id, "Unable to find the thread.");
	thread_ids_.erase(it);
}

void MapBaseData::add_thread(std::thread::id thread_id) const
{
	// avoid the unknown threads stored at the beginning of the vector
	auto real_begin =thread_ids_.begin();
	std::advance(real_begin, NB_UNKNOWN_THREADS);

	auto it = std::lower_bound(real_begin, thread_ids_.end(), thread_id);
	if (it == thread_ids_.end() || *it != thread_id)
		thread_ids_.insert(it, thread_id);
}

} // namespace cgogn

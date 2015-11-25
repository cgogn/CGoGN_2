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

#ifndef CORE_MAP_MAP_BASE_DATA_H_
#define CORE_MAP_MAP_BASE_DATA_H_

#include <core/container/chunk_array_container.h>
#include <utils/definitions.h>
#include <core/basic/cell.h>

#include <utils/thread.h>

#include <thread>
#include <mutex>
#include <algorithm>

namespace cgogn
{

/**
 * @brief Generic Map class
 */
class MapGen
{
private:

	/// vector of Map instances
	static std::vector<MapGen*>* instances_;

public:

	MapGen();

	virtual ~MapGen();

	static inline bool is_alive(MapGen* map)
	{
		return std::find(instances_->begin(), instances_->end(), map) != instances_->end();
	}
};


/**
 * @brief The MapBaseData class
 */
template<typename DATA_TRAITS>
class MapBaseData : public MapGen
{
protected:

	/// topology & embedding indices
	ChunkArrayContainer<DATA_TRAITS::CHUNK_SIZE, unsigned char> topology_;

	/// embedding attributes
	ChunkArrayContainer<DATA_TRAITS::CHUNK_SIZE, unsigned int> attributes_[NB_ORBITS];

	/// embedding indices shortcuts
	ChunkArray<DATA_TRAITS::CHUNK_SIZE, unsigned int>* embeddings_[NB_ORBITS];

	/// boundary markers shortcuts
	ChunkArray<DATA_TRAITS::CHUNK_SIZE, bool>* boundary_markers_[2];
	// TODO: ?? store in a std::vector ?

	/// vector of available mark attributes per orbit per thread
	std::vector<ChunkArray<DATA_TRAITS::CHUNK_SIZE, bool>*> mark_attributes_[NB_ORBITS][NB_THREADS];
	std::mutex mark_attributes_mutex_[NB_ORBITS];

	/// vector of available mark attributes per thread on the topology container
	std::vector<ChunkArray<DATA_TRAITS::CHUNK_SIZE, bool>*> mark_attributes_topology_[NB_THREADS];
	std::mutex mark_attributes_topology_mutex_;

	/// vector of thread ids known by the map that can pretend to data such as mark vectors
	std::vector<std::thread::id> thread_ids_;

public:

	typedef MapGen Inherit;

	static const unsigned int CHUNK_SIZE = DATA_TRAITS::CHUNK_SIZE;

	MapBaseData() : Inherit()
	{
		for (unsigned int i = 0; i < NB_ORBITS; ++i)
		{
			embeddings_[i] = nullptr;
			for (unsigned int j = 0; j < NB_THREADS; ++j)
			{
				mark_attributes_[i][j].reserve(8);
			}
		}

		thread_ids_.reserve(NB_THREADS + 1);
		thread_ids_.push_back(std::this_thread::get_id());
	}

	~MapBaseData() override
	{}

	/*******************************************************************************
	 * Containers management
	 *******************************************************************************/

	inline ChunkArrayContainer<DATA_TRAITS::CHUNK_SIZE, unsigned int>& get_attribute_container(unsigned int orbit)
	{
		return attributes_[orbit];
	}

	inline const ChunkArrayContainer<DATA_TRAITS::CHUNK_SIZE, unsigned int>& get_attribute_container(unsigned int orbit) const
	{
		return attributes_[orbit];
	}

	inline ChunkArray<DATA_TRAITS::CHUNK_SIZE, bool>* get_topology_mark_attribute()
	{
		unsigned int thread = get_current_thread_index();
		if (!mark_attributes_topology_[thread].empty())
		{
			ChunkArray<DATA_TRAITS::CHUNK_SIZE, bool>* ca = mark_attributes_topology_[thread].back();
			mark_attributes_topology_[thread].pop_back();
			return ca;
		}
		else
		{
			std::lock_guard<std::mutex> lock(mark_attributes_topology_mutex_);
			ChunkArray<DATA_TRAITS::CHUNK_SIZE, bool>* ca = topology_.add_marker_attribute();
			return ca;
		}
	}

	inline void release_topology_mark_attribute(ChunkArray<DATA_TRAITS::CHUNK_SIZE, bool>* ca)
	{
		unsigned int thread = get_current_thread_index();
		mark_attributes_topology_[thread].push_back(ca);
	}

	template <unsigned int ORBIT>
	inline ChunkArray<DATA_TRAITS::CHUNK_SIZE, bool>* get_mark_attribute()
	{
		cgogn_message_assert(embeddings_[ORBIT] != NULL, "Invalid parameter: orbit not embedded");

		unsigned int thread = get_current_thread_index();
		if (!mark_attributes_[ORBIT][thread].empty())
		{
			ChunkArray<DATA_TRAITS::CHUNK_SIZE, bool>* ca = mark_attributes_[ORBIT][thread].back();
			mark_attributes_[ORBIT][thread].pop_back();
			return ca;
		}
		else
		{
			std::lock_guard<std::mutex> lock(mark_attributes_mutex_[ORBIT]);
			ChunkArray<DATA_TRAITS::CHUNK_SIZE, bool>* ca = attributes_[ORBIT].add_marker_attribute();
			return ca;
		}
	}

	template <unsigned int ORBIT>
	inline void release_mark_attribute(ChunkArray<DATA_TRAITS::CHUNK_SIZE, bool>* ca)
	{
		cgogn_message_assert(embeddings_[ORBIT] != NULL, "Invalid parameter: orbit not embedded");

		unsigned int thread = get_current_thread_index();
		mark_attributes_[ORBIT][thread].push_back(ca);
	}

protected:

	/*******************************************************************************
	 * Thread management
	 *******************************************************************************/

	inline unsigned int get_current_thread_index() const
	{
		std::thread::id id = std::this_thread::get_id();
		unsigned int i = 0;
		while (id != thread_ids_[i])
		{
			i++;
			cgogn_assert(i < thread_ids_.size());
		}
		return i;
	}
};

} // namespace cgogn

#endif // CORE_MAP_MAP_BASE_DATA_H_

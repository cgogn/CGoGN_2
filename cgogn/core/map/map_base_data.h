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
#include <type_traits>
#include <sstream>

namespace cgogn
{

/**
 * @brief Generic Map class
 */
class CGOGN_CORE_API MapGen
{
public:

	typedef MapGen Self;

private:

	/// vector of Map instances
	static std::vector<MapGen*>* instances_;

public:

	MapGen();

	virtual ~MapGen();
	MapGen(MapGen const&) = delete;
	MapGen(MapGen &&) = delete;
	MapGen& operator=(MapGen const&) = delete;
	MapGen& operator=(MapGen &&) = delete;
	static inline bool is_alive(MapGen* map)
	{
		return std::find(instances_->begin(), instances_->end(), map) != instances_->end();
	}
};


/**
 * @brief The MapBaseData class
 */
template <typename DATA_TRAITS>
class MapBaseData : public MapGen
{
public:

	typedef MapGen Inherit;
	typedef MapBaseData<DATA_TRAITS> Self;

	static const unsigned int CHUNKSIZE = DATA_TRAITS::CHUNK_SIZE;

	template<typename T>
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNKSIZE, T>;
	using ChunkArrayGen = cgogn::ChunkArrayGen<CHUNKSIZE>;
	template<typename T>
	using ChunkArray = cgogn::ChunkArray<CHUNKSIZE, T>;

protected:

	/// topology & embedding indices
	ChunkArrayContainer<unsigned char> topology_;

	/// embedding attributes
	ChunkArrayContainer<unsigned int> attributes_[NB_ORBITS];

	/// embedding indices shortcuts
	ChunkArray<unsigned int>* embeddings_[NB_ORBITS];

	/// boundary markers shortcuts
	ChunkArray<bool>* boundary_markers_[2];
	// TODO: ?? store in a std::vector ?

	/// vector of available mark attributes per orbit per thread
	std::vector<ChunkArray<bool>*> mark_attributes_[NB_ORBITS][NB_THREADS];
	unsigned int mark_attribute_id_[NB_ORBITS];
	std::mutex mark_attributes_mutex_[NB_ORBITS];

	/// vector of available mark attributes per thread on the topology container
	std::vector<ChunkArray<bool>*> mark_attributes_topology_[NB_THREADS];
	unsigned int mark_attribute_topology_id_;
	std::mutex mark_attributes_topology_mutex_;

	/// vector of thread ids known by the map that can pretend to data such as mark vectors
	std::vector<std::thread::id> thread_ids_;

public:

	MapBaseData() : Inherit()
	{
		static bool initCAFactory = true;
		if (initCAFactory)
		{
			ChunkArrayFactory<CHUNKSIZE>::reset();
			initCAFactory = false;
		}
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

	MapBaseData(Self const&) = delete;
	MapBaseData(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

	/*******************************************************************************
	 * Containers management
	 *******************************************************************************/

	inline const ChunkArrayContainer<unsigned int>& get_attribute_container(unsigned int orbit) const
	{
		cgogn_message_assert(orbit < NB_ORBITS, "Unknown orbit parameter");
		return attributes_[orbit];
	}

	inline ChunkArrayContainer<unsigned int>& get_attribute_container(unsigned int orbit)
	{
		cgogn_message_assert(orbit < NB_ORBITS, "Unknown orbit parameter");
		return attributes_[orbit];
	}

	/*******************************************************************************
	 * Embedding (orbit indexing) management
	 *******************************************************************************/

	template <unsigned int ORBIT>
	inline bool is_orbit_embedded() const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		return embeddings_[ORBIT] != nullptr;
	}

	template <unsigned int ORBIT>
	inline unsigned int get_embedding(Cell<ORBIT> c) const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		return (*embeddings_[ORBIT])[c.dart.index];
	}

	template <unsigned int ORBIT>
	inline void init_embedding(Dart d, unsigned int emb)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		this->attributes_[ORBIT].ref_line(emb);     // ref the new emb
		(*this->embeddings_[ORBIT])[d.index] = emb; // affect the embedding to the dart
	}

	template <unsigned int ORBIT>
	inline void set_embedding(Dart d, unsigned int emb)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		unsigned int old = get_embedding<ORBIT>(d);

		if (old == emb)	return;

		this->attributes_[ORBIT].unref_line(old); // unref the old emb
		this->attributes_[ORBIT].ref_line(emb);   // ref the new emb

		(*this->embeddings_[ORBIT])[d.index] = emb; // affect the embedding to the dart
	}

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

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

#ifndef CORE_CMAP_MAP_BASE_DATA_H_
#define CORE_CMAP_MAP_BASE_DATA_H_

#include <core/utils/definitions.h>
#include <core/utils/thread.h>
#include <core/utils/thread_pool.h>
#include <core/container/chunk_array_container.h>
#include <core/basic/cell.h>
#include <core/cmap/map_traits.h>

#include <thread>
#include <mutex>
#include <algorithm>
#include <type_traits>
#include <sstream>
#include <iterator>

namespace cgogn
{

/**
 * @brief Generic Map class
 */
class CGOGN_CORE_API MapGen
{
public:

	typedef MapGen Self;

protected:

	/// vector of Map instances
	static std::vector<MapGen*>* instances_;
	static bool init_CA_factory;

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

// forward declaration of class AttributeHandlerOrbit
template <typename DATA_TRAITS, Orbit ORBIT>
class AttributeHandlerOrbit;

/**
 * @brief The MapBaseData class
 */
template <typename MAP_TRAITS>
class MapBaseData : public MapGen
{
public:

	typedef MapGen Inherit;
	typedef MapBaseData<MAP_TRAITS> Self;

	static const unsigned int CHUNKSIZE = MAP_TRAITS::CHUNK_SIZE;
	static const unsigned int NB_UNKNOWN_THREADS = 4u;
	template <typename DT, Orbit ORBIT> friend class AttributeHandlerOrbit;

	template <typename T_REF>
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNKSIZE, T_REF>;
	using ChunkArrayGen = cgogn::ChunkArrayGen<CHUNKSIZE>;
	template <typename T>
	using ChunkArray = cgogn::ChunkArray<CHUNKSIZE, T>;

protected:

	/// topology & embedding indices
	ChunkArrayContainer<unsigned char> topology_;

	/// per orbit attributes
	std::array<ChunkArrayContainer<unsigned int>, NB_ORBITS> attributes_;

	/// embedding indices shortcuts
	std::array<ChunkArray<unsigned int>*, NB_ORBITS> embeddings_;

	/// boundary markers shortcuts
	std::array<ChunkArray<bool>*, 2> boundary_markers_;
	// TODO: ?? store in a std::vector ?

	/// vector of available mark attributes per thread on the topology container
	std::vector<std::vector<ChunkArray<bool>*>> mark_attributes_topology_;
	std::mutex mark_attributes_topology_mutex_;

	/// vector of available mark attributes per orbit per thread on attributes containers
	std::array<std::vector<std::vector<ChunkArray<bool>*>>, NB_ORBITS> mark_attributes_;
	std::array<std::mutex, NB_ORBITS> mark_attributes_mutex_;

	/// Before accessing the map, a thread should call map.add_thread(std::this_thread::get_id()) (and do a map.remove_thread(std::this_thread::get_id() before it terminates)
	/// The first part of the vector ( 0 to NB_UNKNOWN_THREADS -1) stores threads that want to access the map without using this interface. They might be deleted if we have too many of them.
	/// The second part (NB_UNKNOWN_THREADS to infinity) of the vector stores threads IDs added using this interface and they are guaranteed not to be deleted.
	mutable std::vector<std::thread::id> thread_ids_;

	/// global topo cache shortcuts
	std::array<ChunkArray<Dart>*, NB_ORBITS> global_topo_cache_;

public:

	MapBaseData() : Inherit()
	{
		if (init_CA_factory)
		{
			ChunkArrayFactory<CHUNKSIZE>::reset();
			init_CA_factory = false;
		}
		for (unsigned int i = 0; i < NB_ORBITS; ++i)
		{
			mark_attributes_[i].reserve(NB_UNKNOWN_THREADS + 2u*MAX_NB_THREADS);
			mark_attributes_[i].resize(NB_UNKNOWN_THREADS + MAX_NB_THREADS);

			embeddings_[i] = nullptr;
			global_topo_cache_[i] = nullptr;
			for (unsigned int j = 0; j < NB_UNKNOWN_THREADS + MAX_NB_THREADS; ++j)
				mark_attributes_[i][j].reserve(8);
		}

		mark_attributes_topology_.reserve(NB_UNKNOWN_THREADS + 2u*MAX_NB_THREADS);
		mark_attributes_topology_.resize(NB_UNKNOWN_THREADS + MAX_NB_THREADS);

		for (unsigned int i = 0; i < MAX_NB_THREADS; ++i)
			mark_attributes_topology_[i].reserve(8);

		thread_ids_.reserve(NB_UNKNOWN_THREADS + 2u*MAX_NB_THREADS);
		thread_ids_.resize(NB_UNKNOWN_THREADS);
		this->add_thread(std::this_thread::get_id());
		const auto& pool_threads_ids = cgogn::get_thread_pool()->get_threads_ids();
		for (const std::thread::id& ids : pool_threads_ids)
			this->add_thread(ids);
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

	template <Orbit ORBIT>
	inline const ChunkArrayContainer<unsigned int>& get_attribute_container() const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		return attributes_[ORBIT];
	}

protected:

	template <Orbit ORBIT>
	inline ChunkArrayContainer<unsigned int>& get_attribute_container()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		return attributes_[ORBIT];
	}

	/*******************************************************************************
	 * Marking attributes management
	 *******************************************************************************/

	/**
	* \brief get a mark attribute on the topology container (from pool or created)
	* @return a mark attribute on the topology container
	*/
	inline ChunkArray<bool>* get_topology_mark_attribute()
	{
		unsigned int thread = this->get_current_thread_index();
		if (!this->mark_attributes_topology_[thread].empty())
		{
			ChunkArray<bool>* ca = this->mark_attributes_topology_[thread].back();
			this->mark_attributes_topology_[thread].pop_back();
			return ca;
		}
		else
		{
			std::lock_guard<std::mutex> lock(this->mark_attributes_topology_mutex_);
			ChunkArray<bool>* ca = this->topology_.add_marker_attribute();
			return ca;
		}
	}

	/**
	* \brief release a mark attribute on the topology container
	* @param the mark attribute to release
	*/
	inline void release_topology_mark_attribute(ChunkArray<bool>* ca)
	{
		unsigned int thread = this->get_current_thread_index();
		this->mark_attributes_topology_[thread].push_back(ca);
	}

public:

	/*******************************************************************************
	 * Embedding (orbit indexing) management
	 *******************************************************************************/

	template <Orbit ORBIT>
	inline bool is_orbit_embedded() const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		return embeddings_[ORBIT] != nullptr;
	}

	template <Orbit ORBIT>
	inline unsigned int get_embedding(Cell<ORBIT> c) const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		return (*embeddings_[ORBIT])[c.dart.index];
	}

protected:

	template <Orbit ORBIT>
	inline void init_embedding(Dart d, unsigned int emb)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		this->attributes_[ORBIT].ref_line(emb); // ref the new emb
		(*this->embeddings_[ORBIT])[d.index] = emb; // affect the embedding to the dart
	}

	template <Orbit ORBIT>
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

	inline unsigned int add_unknown_thread() const
	{
		static unsigned int index = 0u;
		const std::thread::id& th_id = std::this_thread::get_id();
		std::cerr << "WARNING: registration of an unknown thread (id :" << th_id << ") in the map." << std::endl;
		std::cerr << "Data can be lost. Please use add_thread and remove_thread interface." << std::endl;
		thread_ids_[index] = th_id;
		const unsigned old_index = index;
		index  = (index+1u)% NB_UNKNOWN_THREADS;
		return old_index;
	}

	inline unsigned int get_unknown_thread_index(std::thread::id thread_id) const
	{
		auto end = thread_ids_.begin();
		std::advance(end, NB_UNKNOWN_THREADS);
		auto res_it = std::find(thread_ids_.begin(), end, thread_id);
		if (res_it != end)
			return std::distance(thread_ids_.begin(), res_it);

		return add_unknown_thread();
	}

	inline unsigned int get_current_thread_index() const
	{
		// avoid the unknown threads stored at the beginning of the vector
		auto real_begin =thread_ids_.begin();
		std::advance(real_begin, NB_UNKNOWN_THREADS);

		const auto end = thread_ids_.end();
		auto it_lower_bound = std::lower_bound(real_begin, end, std::this_thread::get_id());
		if (it_lower_bound != end)
			return std::distance(thread_ids_.begin(),it_lower_bound);

		return get_unknown_thread_index(std::this_thread::get_id());
	}

	inline void remove_thread(std::thread::id thread_id) const
	{
		// avoid the unknown threads stored at the beginning of the vector
		auto real_begin =thread_ids_.begin();
		std::advance(real_begin, NB_UNKNOWN_THREADS);

		cgogn_message_assert(std::binary_search(real_begin, thread_ids_.end(), thread_id),"Unable to find the thread.");
		auto it = std::lower_bound(real_begin, thread_ids_.end(),thread_id);
		cgogn_message_assert((*it)  == thread_id,"Unable to find the thread.");
		thread_ids_.erase(it);
	}

	inline void add_thread(std::thread::id thread_id) const
	{
		// avoid the unknown threads stored at the beginning of the vector
		auto real_begin =thread_ids_.begin();
		std::advance(real_begin, NB_UNKNOWN_THREADS);

		auto it = std::lower_bound(real_begin, thread_ids_.end(),thread_id);
		if ((it == thread_ids_.end()) || (*it  != thread_id))
		{
			thread_ids_.insert(it,thread_id);
		}

	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP_BASE_DATA_CPP_))
extern template class CGOGN_CORE_API MapBaseData<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP_BASE_DATA_CPP_))

} // namespace cgogn

#endif // CORE_CMAP_MAP_BASE_DATA_H_

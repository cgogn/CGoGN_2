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

#ifndef CGOGN_CORE_CMAP_MAP_BASE_DATA_H_
#define CGOGN_CORE_CMAP_MAP_BASE_DATA_H_

#include <thread>
#include <mutex>
#include <algorithm>
#include <type_traits>
#include <sstream>
#include <iterator>

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/utils/thread.h>
#include <cgogn/core/utils/thread_pool.h>
#include <cgogn/core/utils/name_types.h>
#include <cgogn/core/container/chunk_array_container.h>
#include <cgogn/core/basic/cell.h>
#include <cgogn/core/cmap/map_traits.h>

#define CGOGN_CHECK_DYNAMIC_TYPE cgogn_message_assert( (std::is_same<typename MapType::TYPE, Self>::value),\
	std::string("dynamic type of current object : ") + cgogn::internal::demangle(std::string(typeid(*this).name())) + std::string(",\nwhereas Self = ") + cgogn::name_of_type(Self()))

#ifndef _MSC_VER
#define CGOGN_CHECK_CONCRETE_TYPE static_assert(std::is_same<typename MapType::TYPE, Self>::value,"The concrete map type has to be equal to Self")
#else
#define CGOGN_CHECK_CONCRETE_TYPE CGOGN_CHECK_DYNAMIC_TYPE
#endif

namespace cgogn
{

/**
 * @brief Generic Map class
 */
class CGOGN_CORE_API MapGen
{
public:

	using Self = MapGen;

protected:

	/// vector of Map instances
	static std::vector<MapGen*>* instances_;
	static bool init_CA_factory;

public:

	MapGen();
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MapGen);
	virtual ~MapGen();

	static inline bool is_alive(MapGen* map)
	{
		return std::find(instances_->begin(), instances_->end(), map) != instances_->end();
	}
};

// forward declaration of class AttributeOrbit
template <typename DATA_TRAITS, Orbit ORBIT>
class AttributeOrbit;

/**
 * @brief The MapBaseData class
 */
template <typename MAP_TRAITS>
class MapBaseData : public MapGen
{
public:

	using Inherit = MapGen;
	using Self = MapBaseData<MAP_TRAITS>;

	static const uint32 CHUNKSIZE = MAP_TRAITS::CHUNK_SIZE;
	static const uint32 NB_UNKNOWN_THREADS = 4u;
	template <typename DT, Orbit ORBIT> friend class AttributeOrbit;
	template <typename DT, typename T, Orbit ORBIT> friend class Attribute;

	template <typename T_REF>
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNKSIZE, T_REF>;
	using ChunkArrayGen = cgogn::ChunkArrayGen<CHUNKSIZE>;
	template <typename T>
	using ChunkArray = cgogn::ChunkArray<CHUNKSIZE, T>;

protected:

	// topology & embedding indices
	ChunkArrayContainer<uint8> topology_;

	/// per orbit attributes
	std::array<ChunkArrayContainer<uint32>, NB_ORBITS> attributes_;

	/// embedding indices shortcuts
	std::array<ChunkArray<uint32>*, NB_ORBITS> embeddings_;

	/// boundary marker shortcut
	ChunkArray<bool>* boundary_marker_;

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

public:

	MapBaseData() : Inherit()
	{
		if (init_CA_factory)
		{
			ChunkArrayFactory<CHUNKSIZE>::reset();
			init_CA_factory = false;
		}
		for (uint32 i = 0; i < NB_ORBITS; ++i)
		{
			mark_attributes_[i].reserve(NB_UNKNOWN_THREADS + 2u*MAX_NB_THREADS);
			mark_attributes_[i].resize(NB_UNKNOWN_THREADS + MAX_NB_THREADS);

			embeddings_[i] = nullptr;
			for (uint32 j = 0; j < NB_UNKNOWN_THREADS + MAX_NB_THREADS; ++j)
				mark_attributes_[i][j].reserve(8);
		}

		mark_attributes_topology_.reserve(NB_UNKNOWN_THREADS + 2u*MAX_NB_THREADS);
		mark_attributes_topology_.resize(NB_UNKNOWN_THREADS + MAX_NB_THREADS);

		for (uint32 i = 0; i < MAX_NB_THREADS; ++i)
			mark_attributes_topology_[i].reserve(8);

		boundary_marker_ = topology_.add_marker_attribute();

		thread_ids_.reserve(NB_UNKNOWN_THREADS + 2u*MAX_NB_THREADS);
		thread_ids_.resize(NB_UNKNOWN_THREADS);

		this->add_thread(std::this_thread::get_id());
		const auto& pool_threads_ids = cgogn::get_thread_pool()->get_threads_ids();
		for (const std::thread::id& ids : pool_threads_ids)
			this->add_thread(ids);
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MapBaseData);

	~MapBaseData() override
	{}

	/*******************************************************************************
	 * Containers management
	 *******************************************************************************/

	template <Orbit ORBIT>
	inline const ChunkArrayContainer<uint32>& get_attribute_container() const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		return attributes_[ORBIT];
	}

	inline const ChunkArrayContainer<uint8>& get_topology_container() const
	{
		return topology_;
	}

protected:

	template <Orbit ORBIT>
	inline ChunkArrayContainer<uint32>& get_attribute_container()
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
		std::size_t thread = this->get_current_thread_index();
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
		std::size_t thread = this->get_current_thread_index();
		this->mark_attributes_topology_[thread].push_back(ca);
	}

	/*******************************************************************************
	 * Embedding (orbit indexing) management
	 *******************************************************************************/

public:

	template <Orbit ORBIT>
	inline bool is_embedded() const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		return embeddings_[ORBIT] != nullptr;
	}

	template <class CellType>
	inline bool is_embedded() const
	{
		return is_embedded<CellType::ORBIT>();
	}

	template <Orbit ORBIT>
	inline uint32 get_embedding(Cell<ORBIT> c) const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");
		cgogn_message_assert((*embeddings_[ORBIT])[c.dart.index] != EMBNULL, "get_embedding result is EMBNULL");

		return (*embeddings_[ORBIT])[c.dart.index];
	}

protected:

	template <class CellType>
	inline void set_embedding(Dart d, uint32 emb)
	{
		static const Orbit ORBIT = CellType::ORBIT;
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");
		cgogn_message_assert(emb != EMBNULL,"cannot set an embedding to EMBNULL.");

		const uint32 old = (*embeddings_[ORBIT])[d.index];

		// ref_line() is done before unref_line() to avoid deleting the indexed line if old == emb
		attributes_[ORBIT].ref_line(emb);			// ref the new emb
		if (old != EMBNULL)
			attributes_[ORBIT].unref_line(old);	// unref the old emb

		(*embeddings_[ORBIT])[d.index] = emb;		// affect the embedding to the dart
	}

	template <class CellType>
	inline void copy_embedding(Dart dest, Dart src)
	{
		static const Orbit ORBIT = CellType::ORBIT;
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		this->template set_embedding<CellType>(dest, get_embedding(CellType(src)));
	}

	template <class CellType>
	inline void copy_cell_attributes(Dart dest, Dart src)
	{
		static const Orbit ORBIT = CellType::ORBIT;
		uint32 dest_emb = get_embedding(CellType(dest));
		uint32 src_emb = get_embedding(CellType(src));

		if(dest_emb != EMBNULL)
		{
			if(src_emb != EMBNULL)
			{
				this->attributes_[ORBIT].copy_line(dest_emb, src_emb, true, false);
			}
		}

	}

protected:

	/*******************************************************************************
	 * Thread management
	 *******************************************************************************/

	inline uint32 add_unknown_thread() const
	{
		static uint32 index = 0u;
		const std::thread::id& th_id = std::this_thread::get_id();
		cgogn_log_warning("add_unknown_thread") << "Registration of an unknown thread (id :" << th_id << ") in the map. Data can be lost. Please use add_thread and remove_thread interface.";
		thread_ids_[index] = th_id;
		const unsigned old_index = index;
		index = (index+1u) % NB_UNKNOWN_THREADS;
		return old_index;
	}

	inline std::size_t get_unknown_thread_index(std::thread::id thread_id) const
	{
		auto end = thread_ids_.begin();
		std::advance(end, NB_UNKNOWN_THREADS);
		auto res_it = std::find(thread_ids_.begin(), end, thread_id);
		if (res_it != end)
			return std::distance(thread_ids_.begin(), res_it);

		return add_unknown_thread();
	}

	inline std::size_t get_current_thread_index() const
	{
		// avoid the unknown threads stored at the beginning of the vector
		auto real_begin = thread_ids_.begin();
		std::advance(real_begin, NB_UNKNOWN_THREADS);

		const auto end = thread_ids_.end();
		auto it_lower_bound = std::lower_bound(real_begin, end, std::this_thread::get_id());
		if (it_lower_bound != end)
			return std::distance(thread_ids_.begin(), it_lower_bound);

		return get_unknown_thread_index(std::this_thread::get_id());
	}

	inline void remove_thread(std::thread::id thread_id) const
	{
		// avoid the unknown threads stored at the beginning of the vector
		auto real_begin = thread_ids_.begin();
		std::advance(real_begin, NB_UNKNOWN_THREADS);

		cgogn_message_assert(std::binary_search(real_begin, thread_ids_.end(), thread_id), "Unable to find the thread.");
		auto it = std::lower_bound(real_begin, thread_ids_.end(), thread_id);
		cgogn_message_assert(*it == thread_id, "Unable to find the thread.");
		thread_ids_.erase(it);
	}

	inline void add_thread(std::thread::id thread_id) const
	{
		// avoid the unknown threads stored at the beginning of the vector
		auto real_begin =thread_ids_.begin();
		std::advance(real_begin, NB_UNKNOWN_THREADS);

		auto it = std::lower_bound(real_begin, thread_ids_.end(), thread_id);
		if (it == thread_ids_.end() || *it != thread_id)
			thread_ids_.insert(it, thread_id);
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP_BASE_DATA_CPP_))
extern template class CGOGN_CORE_API MapBaseData<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP_BASE_DATA_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CMAP_MAP_BASE_DATA_H_

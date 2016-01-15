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
#include <core/utils/definitions.h>
#include <core/utils/thread.h>
#include <core/basic/cell.h>

// TODO Pour MapGen => Besoin de MR_MapGen ?
#include <core/cmap/map_base_data.h>

#include <thread>
#include <mutex>
#include <algorithm>
#include <type_traits>
#include <sstream>

namespace cgogn
{

struct DefaultMR_MapTraits : DefaultMapTraits
{
	static const unsigned int NB_LEVELS = 16;
};

/**
 * @brief The MapBaseData class
 */
template <typename MAP_TRAITS>
class MR_MapBaseData : public MapGen
{
public:

	typedef MapGen Inherit;
	typedef MR_MapBaseData<MAP_TRAITS> Self;

	static const unsigned int CHUNKSIZE = MAP_TRAITS::CHUNK_SIZE;
	static const unsigned int NB_LEVELS = MAP_TRAITS::NB_LEVELS;

	template <typename DT, Orbit ORBIT> friend class AttributeHandlerOrbit;

	template<typename T_REF>
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNKSIZE, T_REF>;
	using ChunkArrayGen = cgogn::ChunkArrayGen<CHUNKSIZE>;
	template<typename T>
	using ChunkArray = cgogn::ChunkArray<CHUNKSIZE, T>;

protected:

	/// current level of resolution
	unsigned int current_level_;

	/// topology & embedding indices
	ChunkArrayContainer<unsigned char> topology_[NB_LEVELS];

	/// per orbit attributes
	ChunkArrayContainer<unsigned int> attributes_[NB_LEVELS][NB_ORBITS];

	/// embedding indices shortcuts
	ChunkArray<unsigned int>* embeddings_[NB_LEVELS][NB_ORBITS];

	/// boundary markers shortcuts
	ChunkArray<bool>* boundary_markers_[2];
	// TODO: ?? store in a std::vector ?
	// TODO: Do we need disctinct markers for each MR levels ?

	/// vector of available mark attributes per thread on the topology container
	std::vector<ChunkArray<bool>*> mark_attributes_topology_[NB_LEVELS][NB_THREADS];
	std::mutex mark_attributes_topology_mutex_[NB_LEVELS];

	/// vector of available mark attributes per orbit per thread on attributes containers
	std::vector<ChunkArray<bool>*> mark_attributes_[NB_LEVELS][NB_ORBITS][NB_THREADS];
	std::mutex mark_attributes_mutex_[NB_LEVELS][NB_ORBITS];

	/// vector of thread ids known by the map that can pretend to data such as mark vectors
	std::vector<std::thread::id> thread_ids_;

	/// global topo cache shortcuts
	ChunkArray<Dart>* global_topo_cache_[NB_LEVELS][NB_ORBITS];

public:

	MR_MapBaseData() : Inherit(), current_level_(0)
	{
		if (init_CA_factory)
		{
			ChunkArrayFactory<CHUNKSIZE>::reset();
			init_CA_factory = false;
		}

		for (unsigned int l = 0; l < NB_LEVELS; ++l) {
			for (unsigned int i = 0; i < NB_ORBITS; ++i)
			{
				embeddings_[l][i] = nullptr;
				global_topo_cache_[l][i] = nullptr;
				for (unsigned int j = 0; j < NB_THREADS; ++j)
					mark_attributes_[l][i][j].reserve(8);
			}
			for (unsigned int i = 0; i < NB_THREADS; ++i)
				mark_attributes_topology_[l][i].reserve(8);
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

protected:

	/*******************************************************************************
	 * Containers management
	 *******************************************************************************/

	inline const ChunkArrayContainer<unsigned int>& get_attribute_container(unsigned int orbit) const
	{
		cgogn_message_assert(orbit < NB_ORBITS, "Unknown orbit parameter");
		return attributes_[current_level_][orbit];
	}

	inline ChunkArrayContainer<unsigned int>& get_attribute_container(unsigned int orbit)
	{
		cgogn_message_assert(orbit < NB_ORBITS, "Unknown orbit parameter");
		return attributes_[current_level_][orbit];
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
		if (!this->mark_attributes_topology_[current_level_][thread].empty())
		{
			ChunkArray<bool>* ca = this->mark_attributes_topology_[current_level_][thread].back();
			this->mark_attributes_topology_[current_level_][thread].pop_back();
			return ca;
		}
		else
		{
			std::lock_guard<std::mutex> lock(this->mark_attributes_topology_mutex_[current_level_]);
			ChunkArray<bool>* ca = this->topology_[current_level_].add_marker_attribute();
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
		this->mark_attributes_topology_[current_level_][thread].push_back(ca);
	}

public:

	/*******************************************************************************
	 * Level of resolution management
	 *******************************************************************************/

	inline void current_level_inc() {
		cgogn_message_assert(current_level_+1 < NB_LEVELS,"No more MR level left");
		++current_level_;
	}

	inline void current_level_dec() {
		cgogn_message_assert(current_level_ > 0,"First MR level alrady reached");
		--current_level_;
	}

	/*******************************************************************************
	 * Embedding (orbit indexing) management
	 *******************************************************************************/

	template <Orbit ORBIT>
	inline bool is_orbit_embedded() const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		return embeddings_[current_level_][ORBIT] != nullptr;
	}

	template <Orbit ORBIT>
	inline unsigned int get_embedding(Cell<ORBIT> c) const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		return (*embeddings_[current_level_][ORBIT])[c.dart.index];
	}

protected:

	template <Orbit ORBIT>
	inline void init_embedding(Dart d, unsigned int emb)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		this->attributes_[current_level_][ORBIT].ref_line(emb);     // ref the new emb
		(*this->embeddings_[current_level_][ORBIT])[d.index] = emb; // affect the embedding to the dart
	}

	template <Orbit ORBIT>
	inline void set_embedding(Dart d, unsigned int emb)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		unsigned int old = get_embedding<ORBIT>(d);

		if (old == emb)	return;

		this->attributes_[current_level_][ORBIT].unref_line(old); // unref the old emb
		this->attributes_[current_level_][ORBIT].ref_line(emb);   // ref the new emb

		(*this->embeddings_[current_level_][ORBIT])[d.index] = emb; // affect the embedding to the dart
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

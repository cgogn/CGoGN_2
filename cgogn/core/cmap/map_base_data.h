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
#define CGOGN_CHECK_CONCRETE_TYPE static_assert(std::is_same<typename MapType::TYPE, Self>::value, "The concrete map type has to be equal to Self")
#else
#define CGOGN_CHECK_CONCRETE_TYPE CGOGN_CHECK_DYNAMIC_TYPE
#endif

namespace cgogn
{

// forward declarations
class AttributeGen;
template <typename T> class Attribute_T;
template <typename T, Orbit ORBIT> class Attribute;

/**
 * @brief The MapBaseData class
 */
class CGOGN_CORE_API MapBaseData
{
public:

	using Self = MapBaseData;

	static const uint32 CHUNK_SIZE = CGOGN_CHUNK_SIZE;

	template <typename T> friend class Attribute_T;
	template <typename T, Orbit ORBIT> friend class Attribute;

	template <typename T_REF>
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNK_SIZE, T_REF>;
	using ChunkArrayGen = cgogn::ChunkArrayGen<CHUNK_SIZE>;
	template <typename T>
	using ChunkArray = cgogn::ChunkArray<CHUNK_SIZE, T>;
	using ChunkArrayBool = cgogn::ChunkArrayBool<CHUNK_SIZE>;

protected:
#pragma warning(push)
#pragma warning(disable:4251)
	// topology & embedding indices
	ChunkArrayContainer<uint8> topology_;

	// per orbit attributes
	std::array<ChunkArrayContainer<uint32>, NB_ORBITS> attributes_;

	// embedding indices shortcuts
	std::array<ChunkArray<uint32>*, NB_ORBITS> embeddings_;

	// boundary marker shortcut
	ChunkArrayBool* boundary_marker_;

	// vector of available mark attributes per thread on the topology container
	std::vector<std::vector<ChunkArrayBool*>> mark_attributes_topology_;
	std::mutex mark_attributes_topology_mutex_;

	// vector of available mark attributes per orbit per thread on attributes containers
	std::array<std::vector<std::vector<ChunkArrayBool*>>, NB_ORBITS> mark_attributes_;
	std::array<std::mutex, NB_ORBITS> mark_attributes_mutex_;

	// vector of Map instances
	static std::vector<const MapBaseData*>* instances_;

	// table of tetra phi2 indices
	static const std::array<uint32, 12> tetra_phi2;
	// table of hexa phi2 indices
	static const std::array<uint32, 24> hexa_phi2;
#pragma warning(pop)

public:

	MapBaseData();
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MapBaseData);
	virtual ~MapBaseData();

	static inline bool is_alive(const MapBaseData* map)
	{
		return (instances_ != nullptr) && (std::find(instances_->begin(), instances_->end(), map) != instances_->end());
	}

	/*******************************************************************************
	 * Containers management
	 *******************************************************************************/

	template <Orbit ORBIT>
	inline const ChunkArrayContainer<uint32>& attribute_container() const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		return attributes_[ORBIT];
	}

	inline const ChunkArrayContainer<uint32>& attribute_container(Orbit orbit) const
	{
		cgogn_message_assert(orbit < NB_ORBITS, "Unknown orbit parameter");
		return attributes_[orbit];
	}

	inline const ChunkArrayContainer<uint8>& topology_container() const
	{
		return topology_;
	}

protected:

	template <Orbit ORBIT>
	inline ChunkArrayContainer<uint32>& non_const_attribute_container()
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
	inline ChunkArrayBool* topology_mark_attribute()
	{
		const std::size_t thread = cgogn::current_thread_marker_index();
		cgogn_assert(thread < mark_attributes_topology_.size());
		if (!this->mark_attributes_topology_[thread].empty())
		{
			ChunkArrayBool* ca = this->mark_attributes_topology_[thread].back();
			this->mark_attributes_topology_[thread].pop_back();
			return ca;
		}
		else
		{
			std::lock_guard<std::mutex> lock(this->mark_attributes_topology_mutex_);
			ChunkArrayBool* ca = this->topology_.add_marker_attribute();
			return ca;
		}
	}

	/**
	* \brief release a mark attribute on the topology container
	* @param the mark attribute to release
	*/
	inline void release_topology_mark_attribute(ChunkArrayBool* ca)
	{
		const std::size_t thread = cgogn::current_thread_marker_index();
		cgogn_assert(thread < mark_attributes_topology_.size());
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

	inline bool is_embedded(Orbit orb) const
	{
		cgogn_message_assert(orb < NB_ORBITS, "Unknown orbit parameter");
		return embeddings_[orb] != nullptr;
	}

	template <class CellType>
	inline bool is_embedded() const
	{
		return is_embedded<CellType::ORBIT>();
	}

	template <Orbit ORBIT>
	inline uint32 embedding(Cell<ORBIT> c) const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");
		cgogn_message_assert((*embeddings_[ORBIT])[c.dart.index] != INVALID_INDEX, "embedding result is INVALID_INDEX");

		return (*embeddings_[ORBIT])[c.dart.index];
	}

	inline uint32 embedding(Dart d, Orbit orb) const
	{
		cgogn_message_assert(orb < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_embedded(orb), "Invalid parameter: orbit not embedded");
		cgogn_message_assert((*embeddings_[orb])[d.index] != INVALID_INDEX, "embedding result is INVALID_INDEX");

		return (*embeddings_[orb])[d.index];
	}

protected:

	template <class CellType>
	inline void set_embedding(Dart d, uint32 emb)
	{
		static const Orbit ORBIT = CellType::ORBIT;
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");
		cgogn_message_assert(emb != INVALID_INDEX, "Cannot set an embedding to INVALID_INDEX.");

		const uint32 old = (*embeddings_[ORBIT])[d.index];

		// ref_line() is done before unref_line() to avoid deleting the indexed line if old == emb
		attributes_[ORBIT].ref_line(emb);			// ref the new emb
		if (old != INVALID_INDEX)
			attributes_[ORBIT].unref_line(old);		// unref the old emb

		(*embeddings_[ORBIT])[d.index] = emb;		// affect the embedding to the dart
	}

	template <class CellType>
	inline void copy_embedding(Dart dest, Dart src)
	{
		static const Orbit ORBIT = CellType::ORBIT;
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		this->template set_embedding<CellType>(dest, embedding(CellType(src)));
	}

};

} // namespace cgogn

#endif // CGOGN_CORE_CMAP_MAP_BASE_DATA_H_

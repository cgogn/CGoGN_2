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

#include <utils/buffers.h>

#include <thread>

namespace cgogn
{

/**
 * @brief Generic Map class (for SCHNApps)
 */
class MapGen
{
public:
	virtual ~MapGen() {}
};


/**
 * @brief The MapBase class
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
	ChunkArray<DATA_TRAITS::CHUNK_SIZE,unsigned int>* embeddings_[NB_ORBITS];

	/// boundary markers shortcuts
	ChunkArray<DATA_TRAITS::CHUNK_SIZE,bool>* boundaryMarkers_[2];
	// TODO: ?? store in a std::vector ?

	/// topo relations shortcuts
	std::vector<ChunkArray<DATA_TRAITS::CHUNK_SIZE,Dart>*> topo_relations_;

	/// buffers of pre-allocated vectors of dart or unsigned int
	Buffers<Dart> dart_buffers_[NB_THREADS];
	Buffers<unsigned int> uint_buffers_[NB_THREADS];

	/// vector of thread ids known by the map that can pretend to data such as mark vectors and buffers
	std::vector<std::thread::id> thread_ids_;

public:

	MapBaseData()
	{
		for (unsigned int i = 0; i < NB_ORBITS; ++i)
			embeddings_[i] = nullptr;

		thread_ids_.reserve(NB_THREADS + 1);
		thread_ids_.push_back(std::this_thread::get_id());
	}

	~MapBaseData() override {}

	inline ChunkArrayContainer<DATA_TRAITS::CHUNK_SIZE, unsigned int>& getAttributeContainer(unsigned int orbit)
	{
		return attributes_[orbit];
	}

	template <unsigned int ORBIT>
	inline unsigned int getEmbedding(const Cell<ORBIT>& c) const
	{
		cgogn_message_assert(embeddings_[ORBIT] != NULL, "Invalid parameter: orbit not embedded");
		return (*embeddings_[ORBIT])[c.dart.index] ;
	}

	inline std::vector<Dart>* getDartBuffer()
	{
		unsigned int thread = getCurrentThreadIndex();
		return dart_buffers_[thread].getBuffer();
	}

	inline void releaseDartBuffer(std::vector<Dart>* v)
	{
		unsigned int thread = getCurrentThreadIndex();
		dart_buffers_[thread].releaseBuffer(v);
	}

	inline std::vector<unsigned int>* getUIntBuffer()
	{
		unsigned int thread = getCurrentThreadIndex();
		return uint_buffers_[thread].getBuffer();
	}

	inline void releaseUIntBuffer(std::vector<unsigned int>* v)
	{
		unsigned int thread = getCurrentThreadIndex();
		uint_buffers_[thread].releaseBuffer(v);
	}

protected:

	inline unsigned int getCurrentThreadIndex() const
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

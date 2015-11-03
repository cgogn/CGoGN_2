/*
 * CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps
 * Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at your
 * option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.
 *
 * Web site: http://cgogn.unistra.fr/
 * Contact information: cgogn@unistra.fr
 *
 */

#ifndef __CORE_CONTAINER_CHUNK_HEAP__
#define __CORE_CONTAINER_CHUNK_HEAP__

#include "core/container/chunk_array.h"

#include <cassert>

namespace cgogn
{

/**
 * @brief Heap implemented in a chunk array
 * @tparam CHUNKSIZE chunk size of array
 * @tparam T type stored in heap
 */
template <unsigned int CHUNKSIZE, typename T>
class ChunkStack : public ChunkArray<CHUNKSIZE, T>
{
	unsigned int heapSize_;

public:
	/**
	 * @brief ChunkStack constructor
	 */
	ChunkStack():
		heapSize_(0)
	{}

	/**
	 * @brief ChunkStack destructor
	 */
	~ChunkStack() {}

	/**
	 * @brief push a value on top of heap
	 * @param val
	 */
	void push(const T& val)
	{
		heapSize_++;
		unsigned int offset = heapSize_ % CHUNKSIZE;
		unsigned int blkId  = heapSize_ / CHUNKSIZE;

		if (blkId >= this->tableData_.size())
			this->addChunk();

		this->tableData_[blkId][offset] = val;
	}

	/**
	 * @brief empty
	 * @return true if heap empty
	 */
	bool empty()
	{
		return heapSize_==0;
	}

	/**
	 * @return number of elements in the heap
	 */
	unsigned int size()
	{
		return heapSize_;
	}

	/**
	 * @brief pop the head of heap
	 */
	void pop()
	{
		assert(heapSize_>0);
		heapSize_--;
	}

	/**
	 * @brief get head of heap
	 * @return copy of head element
	 */
	T head() const
	{
		unsigned int offset = heapSize_ % CHUNKSIZE;
		unsigned int blkId  = heapSize_ / CHUNKSIZE;

		return this->tableData_[blkId][offset];
	}

	/**
	 * @brief compact the heap (free useless memory)
	 */
	void compact()
	{
		unsigned int keep = (heapSize_+CHUNKSIZE-1) / CHUNKSIZE;
		while (this->tableData_.size() > keep)
		{
			delete[] this->tableData_.back();
			this->tableData_.pop_back();
		}
	}

	/**
	 * @brief clear the heap and free memory
	 */
	void clear()
	{
		heapSize_=0;
		ChunkArray<CHUNKSIZE, T>::clear();
	}
};


}



#endif

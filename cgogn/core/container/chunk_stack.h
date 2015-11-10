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

#ifndef __CORE_CONTAINER_CHUNK_STACK_H__
#define __CORE_CONTAINER_CHUNK_STACK_H__

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
public:
	typedef ChunkArray<CHUNKSIZE, T> Inherit;
protected:

	unsigned int stackSize_;

public:
	/**
	 * @brief ChunkStack constructor
	 */
	ChunkStack():
		stackSize_(0u)
	{}

	/**
	 * @brief ChunkStack destructor
	 */
	inline ~ChunkStack() override
	{}

    inline ChunkStack(const ChunkStack<CHUNKSIZE, T>& cs) = delete;
    inline ChunkStack(ChunkStack<CHUNKSIZE, T>&& cs) = delete;
    inline ChunkStack& operator=(const ChunkStack<CHUNKSIZE, T>& cs) = delete;
    inline ChunkStack& operator=(ChunkStack<CHUNKSIZE, T>&& cs) = delete;

	/**
	 * @brief push a value on top of heap
	 * @param val
	 */
	void push(const T& val)
	{
		stackSize_++;
		unsigned int offset = stackSize_ % CHUNKSIZE;
		unsigned int blkId  = stackSize_ / CHUNKSIZE;

		if (blkId >= this->tableData_.size())
			this->addChunk();

		this->tableData_[blkId][offset] = val;
	}

	/**
	 * @brief empty
	 * @return true if heap empty
	 */
	inline bool empty() const
	{
		return stackSize_ == 0u;
	}

	/**
	 * @return number of elements in the heap
	 */
	unsigned int size() const
	{
		return stackSize_;
	}

	/**
	 * @brief pop the head of heap
	 */
	inline void pop()
	{
		assert(stackSize_ > 0u);
		stackSize_--;
	}

	/**
	 * @brief get head of heap
	 * @return copy of head element
	 */
	inline T head() const
	{
		const unsigned int offset = stackSize_ % CHUNKSIZE;
		const unsigned int blkId  = stackSize_ / CHUNKSIZE;

		return this->tableData_[blkId][offset];
	}

	/**
	 * @brief compact the heap (free useless memory)
	 */
	void compact()
	{
		const unsigned int keep = (stackSize_+CHUNKSIZE-1u) / CHUNKSIZE;
		while (this->tableData_.size() > keep)
		{
			delete[] this->tableData_.back();
			this->tableData_.pop_back();
		}
	}

	/**
	 * @brief clear the heap and free memory
	 */
	void clear() override
	{
		stackSize_ = 0u;
		ChunkArray<CHUNKSIZE, T>::clear();
	}
};

} // namespace cgogn

#endif // __CORE_CONTAINER_CHUNK_STACK_H__

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

#ifndef CORE_CONTAINER_CHUNK_STACK_H_
#define CORE_CONTAINER_CHUNK_STACK_H_

#include <core/container/chunk_array.h>
#include <core/utils/assert.h>
#include <core/basic/dll.h>

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
	typedef ChunkStack<CHUNKSIZE, T> Self;
	typedef T value_type;

protected:

	unsigned int stack_size_;

public:
	/**
	 * @brief ChunkStack constructor
	 */
	ChunkStack():
		Inherit(),
		stack_size_(0u)
	{}

	/**
	 * @brief ChunkStack destructor
	 */
	inline ~ChunkStack() override
	{}

	inline ChunkStack(const Self& cs) = delete;
	inline ChunkStack(Self&& cs) = delete;
	inline ChunkStack& operator=(const Self& cs) = delete;
	inline ChunkStack& operator=(Self&& cs) = delete;

	/**
	 * @brief push a value on top of heap
	 * @param val
	 */
	void push(const T& val)
	{
		stack_size_++;
		unsigned int offset = stack_size_ % CHUNKSIZE;
		unsigned int blkId  = stack_size_ / CHUNKSIZE;

		if (blkId >= this->table_data_.size())
			this->add_chunk();

		this->table_data_[blkId][offset] = val;
	}

	/**
	 * @brief empty
	 * @return true if heap empty
	 */
	inline bool empty() const
	{
		return stack_size_ == 0u;
	}

	/**
	 * @return number of elements in the heap
	 */
	unsigned int size() const
	{
		return stack_size_;
	}

	/**
	 * @brief pop the head of heap
	 */
	inline void pop()
	{
		cgogn_assert(stack_size_ > 0u);
		stack_size_--;
	}

	/**
	 * @brief get head of heap
	 * @return copy of head element
	 */
	inline T head() const
	{
		const unsigned int offset = stack_size_ % CHUNKSIZE;
		const unsigned int blkId  = stack_size_ / CHUNKSIZE;

		return this->table_data_[blkId][offset];
	}

	/**
	 * @brief compact the heap (free useless memory)
	 */
	void compact()
	{
		const unsigned int keep = (stack_size_+CHUNKSIZE-1u) / CHUNKSIZE;
		while (this->table_data_.size() > keep)
		{
			delete[] this->table_data_.back();
			this->table_data_.pop_back();
		}
	}

	/**
	 * @brief clear the heap and free memory
	 */
	void clear() override
	{
		stack_size_ = 0u;
		Inherit::clear();
	}

	void swap(Self &cs)
	{
		Inherit::swap(cs);
		std::swap(stack_size_, cs.stack_size_);
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_CONTAINER_CHUNK_STACK_CPP_))
extern template class CGOGN_CORE_API ChunkStack<DefaultMapTraits::CHUNK_SIZE, unsigned int>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_CONTAINER_CHUNK_STACK_CPP_))

} // namespace cgogn

#endif // CORE_CONTAINER_CHUNK_STACK_H_

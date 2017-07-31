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

#ifndef CGOGN_CORE_CONTAINER_CHUNK_STACK_H_
#define CGOGN_CORE_CONTAINER_CHUNK_STACK_H_

#include <cgogn/core/container/chunk_array.h>
#include <cgogn/core/utils/assert.h>
#include <cgogn/core/dll.h>

#include <cgogn/core/cmap/map_traits.h>

namespace cgogn
{

/**
 * @brief Heap implemented in a chunk array
 * @tparam CHUNK_SIZE chunk size of array
 * @tparam T type stored in heap
 */
template <uint32 CHUNK_SIZE, typename T>
class ChunkStack : public ChunkArray<CHUNK_SIZE, T>
{
public:

	using Inherit = ChunkArray<CHUNK_SIZE, T>;
	using Self = ChunkStack<CHUNK_SIZE, T>;
	using value_type = T;

protected:

	uint32 stack_size_;

public:

	/**
	 * @brief ChunkStack constructor
	 */
	inline ChunkStack() :
		Inherit(),
		stack_size_(0u)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ChunkStack);

	/**
	 * @brief ChunkStack destructor
	 */
	inline ~ChunkStack() override
	{}

	/**
	 * @brief push a value on top of heap
	 * @param val
	 */
	void push(const T& val)
	{
		stack_size_++;
		uint32 offset = stack_size_ % CHUNK_SIZE;
		uint32 blkId  = stack_size_ / CHUNK_SIZE;

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
	uint32 size() const
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
		const uint32 offset = stack_size_ % CHUNK_SIZE;
		const uint32 blkId  = stack_size_ / CHUNK_SIZE;

		return this->table_data_[blkId][offset];
	}

	/**
	 * @brief compact the heap (free useless memory)
	 */
	void compact()
	{
		const uint32 keep = (stack_size_+CHUNK_SIZE-1u) / CHUNK_SIZE;
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

	bool swap_data(ChunkArrayGen<CHUNK_SIZE>* cag) override
	{
		if (Inherit::swap_data(cag))
		{
			Self* cs = dynamic_cast<Self*>(cag);
			std::swap(stack_size_, cs->stack_size_);
			return true;
		}
		return false;
	}

	/**
	 * @brief copy the stack into this
	 * @param from source of copy
	 */
	void copy(const typename Inherit::Inherit& cag_src) override
	{
		const Self* from = dynamic_cast<const Self*>(&cag_src);
		if (from == nullptr)
		{
			cgogn_log_error("ChunkStack") << "trying to copy from a different types";
			return;
		}

		clear();
		Inherit::copy(*from);
		stack_size_ = from->stack_size_;
	}

};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CONTAINER_CHUNK_STACK_CPP_))
extern template class CGOGN_CORE_API ChunkStack<CGOGN_CHUNK_SIZE, uint32>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CONTAINER_CHUNK_STACK_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CONTAINER_CHUNK_STACK_H_

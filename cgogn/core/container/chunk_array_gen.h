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

#ifndef CORE_CONTAINER_CHUNK_ARRAY_GEN_H_
#define CORE_CONTAINER_CHUNK_ARRAY_GEN_H_

#include <vector>
#include <iostream>

namespace cgogn
{

/**
 * @brief Virtual version of ChunkArray
 */
template<unsigned int CHUNKSIZE>
class ChunkArrayGen
{
public:
	typedef ChunkArrayGen<CHUNKSIZE> Self;

	ChunkArrayGen() = default;
	ChunkArrayGen(ChunkArrayGen<CHUNKSIZE>const& ) = delete;
	ChunkArrayGen(ChunkArrayGen<CHUNKSIZE>&& ) = delete;
	ChunkArrayGen& operator=(ChunkArrayGen<CHUNKSIZE>const& ) = delete;
	ChunkArrayGen& operator=(ChunkArrayGen<CHUNKSIZE>&& ) = delete;

	/**
	 * @brief virtual destructor
	 */
	virtual ~ChunkArrayGen() {}

	/**
	 * @brief create a ChunkArray object without knowning type
	 * @return generic pointer
	 */
	virtual Self* clone() const = 0;

	virtual bool is_boolean_array() const = 0;

	/**
	 * @brief add a chunk (T[CHUNKSIZE])
	 */
	virtual void add_chunk() = 0;

	/**
	 * @brief set number of chunks
	 * @param nbc number of chunks
	 */
	virtual void set_nb_chunks(unsigned int nbb) = 0;

	/**
	 * @brief get the number of chunks of the array
	 * @return the number of chunks
	 */
	virtual unsigned int get_nb_chunks() const = 0;

	/**
	 * @brief get the capacity of the array
	 * @return number of allocated lines
	 */
	virtual unsigned int capacity() const = 0;

	/**
	 * @brief clear the array
	 */
	virtual void clear() = 0;

	/**
	 * @brief fill a vector with pointers to all chunks
	 * @param addr vector to fill
	 * @param byte_block_size filled with CHUNKSIZE*sizeof(T)
	 * @return addr.size()
	 */
	virtual unsigned int get_chunks_pointers(std::vector<void*>& addr, unsigned int& byte_block_size) const = 0;

	/**
	 * @brief initialize an element of the array (overwrite with T())
	 * @param id index of the element
	 */
	virtual void init_element(unsigned int id) = 0;

	/**
	 * @brief copy an element to another one
	 * @param dst destination element index
	 * @param src source element index
	 */
	virtual void copy_element(unsigned int dst, unsigned int src) = 0;

	/**
	 * @brief swap two elements
	 * @param idx1 first element index
	 * @param idx2 second element index
	 */
	virtual void swap_elements(unsigned int idx1, unsigned int idx2) = 0;

	/**
	 * @brief save
	 * @param fs file stream
	 * @param nb_lines number of line to save
	 */
	virtual void save(std::ostream& fs, unsigned int nb_lines) const = 0;

	/**
	 * @brief load
	 * @param fs file stream
	 * @return ok
	 */
	virtual bool load(std::istream& fs) = 0;
};

} // namespace cgogn

#endif // CORE_CONTAINER_CHUNK_ARRAY_GEN_H_

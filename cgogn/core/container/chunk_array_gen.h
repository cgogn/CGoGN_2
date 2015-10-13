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

#ifndef __CORE_CONTAINER_CHUNK_ARRAY_GEN__
#define __CORE_CONTAINER_CHUNK_ARRAY_GEN__


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

	/**
	 * @brief virtual destructor
	 */
	virtual ~ChunkArrayGen() {}

	/**
	 * @brief create a ChunkArray object without knowning type
	 * @return generic pointer
	 */
	virtual ChunkArrayGen<CHUNKSIZE>* clone() = 0;


	/**
	 * @brief add a chunk (T[CHUNKSIZE])
	 */
	virtual void addChunk() = 0;

	/**
	 * @brief set number of chunks
	 * @param nbc number of chunks
	 */
	virtual void setNbChunks(unsigned int nbb) = 0;


	/**
	 * @brief get the number of chunks of the array
	 * @return the number of chunks
	 */
	virtual unsigned int getNbChunks() const = 0;

	/**
	 * @brief number of allocated elements
	 * @return  allocated lines
	 */
	virtual unsigned int capacity() const = 0;

	/**
	 * @brief clear
	 */
	virtual void clear() = 0;

	/**
	 * @brief get pointer on all chunks data
	 * @param addr vector to fill
	 * @param byteBlockSize filled with CHUNKSIZE*sizeof(T)
	 * @return addr.size()
	 */
	virtual unsigned int getChunksPointers(std::vector<void*>& addr, unsigned int& byteBlockSize) const = 0;

	/**
	 * @brief init an element (overwrite with T())
	 * @param id index of element
	 */
	virtual void initElt(unsigned int id) = 0;

	/**
	 * @brief copy element
	 * @param dst destination
	 * @param src source
	 */
	virtual void copyElt(unsigned int dst, unsigned int src) = 0;

	/**
	 * @brief swap two elements
	 * @param id1 idx first
	 * @param id2 idx second
	 */
	virtual void swapElt(unsigned int id1, unsigned int id2) = 0;

	/**
	 * @brief save
	 * @param fs file stream
	 * @param nbLines number of line to save
	 */
	virtual void save(std::ostream& fs, unsigned int nbLines) const = 0;


	/**
	 * @brief load
	 * @param fs file stream
	 * @return ok
	 */
	virtual bool load(std::istream& fs) = 0;

};


//template<unsigned int CHUNKSIZE>
//ChunkArrayGen<CHUNKSIZE>::~ChunkArrayGen()
//{}


} // namespace CGoGN

#endif

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

#ifndef __CORE_CONTAINER_CHUNK_ARRAY__
#define __CORE_CONTAINER_CHUNK_ARRAY__

#include <vector>

/**
 * \file cgogn/container/chunk_array.h
 * \brief The class that stores data
 */
namespace cgogn {

	/**
	 * \brief Class for blocks of data storage.
	 * \details 
	 * ChunckArray is not a two-dimensional array. It is a vector
	 * of values where each value is itself an array.
	 * ChunckArrays provides functions to efficiently setting and getting
	 * arrays.
	 * \todo
	 * - multi-threaded environment : a thread safety mode to access in read each chunck with a different thread ?
	 * - use std::array<T> instead of T[] ?
	 * \relates ChunckArrayContainer
	 */
	// template <typename T, class ThreadPolicy>
	template <typename T>
	class CGOGN_API ChunkArray {

	private:
		/**  
		 * \brief Vector of chunks 
		 */
		std::vector<T*> table_data_;		

		/**
		 * \brief A symbolic constant that represents the size of a chunk
		 */
		const unsigned int kChunkSize = 4096;

		/**
		 * \brief A symbolic constant that represents the initial size of a vector of chunks
		 */
		const unsigned int kVectorSize = 1024;

		/**
		 * \brief Represents size type of the vector of chunks
		 */
		typedef typename std::vector<T*>::size_type size_type;

	public:
		
		/**
		 * \brief Constructs a new ChunckArray.
		 */
		ChunkArray() {
			table_data_.reserve(kVectorSize);
		}

		/**
		 * \brief ChunckArray destructor.
		 */
		~ChunckArray() {
			for(size_type i = 0 ; i < table_data_.size() ; ++i)
				delete[] table_data_[i];
		}

		/**
		 * \brief Add a new chunk to the vector of chunks
		 */
		inline void addChunk() {
			T* ptr = new T[kChunkSize];
			table_data_.push_back(ptr);

			// TODO test to replace with c++11 version
			// table_data_.emplace_back(kChunkSize);
		}

		/**
		 * \brief Gets the number of chunks of the array
		 * \retval the actual number of chunks
		 */
		inline unsigned int getNbChunks() const {
			//TODO add a typedef for basic types to avoid cast
			return uint32(table_data_.size());
		}

		/**
		 * \brief Sets the number of chunks of the array
		 * \param[in] nbc the number of chuncks
		 */
		inline void setNbChunks(unsigned int nbc) {
			if (nbc >= table_data_.size()) {
				for (size_type i = table_data_.size(); i < nbc; ++i)
					addBlock();
			} else {
				for (size_type i = nbc; i < table_data_.size(); ++i)
					delete[] table_data_[i];
				table_data_.resize(nbc);
			}
		}

	private:
		/**
		 * Forbid copy constructor
		 */
		ChunkArray(const ChunkArray& rhs);

		/**
		 * Forbid assignment operator 
		 */
		ChunkArray& operator=(const ChunkArray& rhs);
	};

	/**
	 *
	 */
	// template<T> using ChunkArray = ChunkArray<T, SingleThreadPolicy>;

	/**
	 *
	 */
	// template<T> using ConcurrentChunkArray = ChunkArray<T, MultipleThreadPolicy>;

}

#endif // __CORE_CONTAINER_CHUNK_ARRAY__
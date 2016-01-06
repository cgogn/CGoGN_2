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

#ifndef CORE_CONTAINER_CHUNK_ARRAY_H_
#define CORE_CONTAINER_CHUNK_ARRAY_H_

#include <core/container/chunk_array_gen.h>
#include <utils/serialization.h>
#include <utils/assert.h>

#include <iostream>
#include <string>
#include <cstring>

namespace cgogn
{

/**
 *	@brief chunk array class storage
 *	@tparam CHUNKSIZE size of each chunk (in T, not in bytes!), must be a power of 2 >=32
 *	@tparam T type of stored data
 */
template <unsigned int CHUNKSIZE, typename T>
class ChunkArray : public ChunkArrayGen<CHUNKSIZE>
{
public:

	typedef ChunkArrayGen<CHUNKSIZE> Inherit;
	typedef ChunkArray<CHUNKSIZE, T> Self;
	typedef T value_type;

protected:

	/// vector of block pointers
	std::vector<T*> table_data_;

public:

	/**
	 * @brief Constructor of ChunkArray
	 */
	inline ChunkArray() : Inherit()
	{
		table_data_.reserve(1024u);
	}

	ChunkArray(const ChunkArray<CHUNKSIZE, T>& ca) = delete;
	ChunkArray(ChunkArray<CHUNKSIZE, T>&& ca) = delete;
	ChunkArray<CHUNKSIZE, T>& operator=(ChunkArray<CHUNKSIZE, T>&& ca) = delete;
	ChunkArray<CHUNKSIZE, T>& operator=(const ChunkArray<CHUNKSIZE, T>& ca) = delete;

	~ChunkArray() override
	{
		for(auto chunk : table_data_)
			delete[] chunk;
	}

	/**
	 * @brief create a ChunkArray<CHUNKSIZE,T>
	 * @return generic pointer
	 */
	ChunkArrayGen<CHUNKSIZE>* clone() const override
	{
		return new Self();
	}

	void swap(Self& ca)
	{
		table_data_.swap(ca.table_data_);
	}

	bool is_boolean_array() const override
	{
		return false;
	}

	/**
	 * @brief add a chunk (T[CHUNKSIZE])
	 */
	void add_chunk() override
	{
		table_data_.emplace_back(new T[CHUNKSIZE]());
	}

	/**
	 * @brief set number of chunks
	 * @param nbc number of chunks
	 */
	void set_nb_chunks(unsigned int nbc) override
	{
		if (nbc >= table_data_.size())
		{
			for (std::size_t i = table_data_.size(); i < nbc; ++i)
				add_chunk();
		}
		else
		{
			for (std::size_t i = static_cast<std::size_t>(nbc); i < table_data_.size(); ++i)
				delete[] table_data_[i];
			table_data_.resize(nbc);
		}
	}

	/**
	 * @brief get the number of chunks of the array
	 * @return the number of chunks
	 */
	unsigned int get_nb_chunks() const override
	{
		return static_cast<unsigned int>(table_data_.size());
	}

	/**
	 * @brief get the capacity of the array
	 * @return number of allocated lines
	 */
	unsigned int capacity() const override
	{
		return static_cast<unsigned int>(table_data_.size())*CHUNKSIZE;
	}

	/**
	 * @brief clear the array
	 */
	void clear() override
	{
		for(auto chunk : table_data_)
			delete[] chunk;
		table_data_.clear();
	}

	/**
	 * @brief fill a vector with pointers to all chunks
	 * @param addr vector to fill
	 * @param byte_block_size filled with CHUNKSIZE*sizeof(T)
	 * @return addr.size()
	 */
	unsigned int get_chunks_pointers(std::vector<void*>& addr, unsigned int& byte_block_size) const override
	{
		byte_block_size = CHUNKSIZE * sizeof(T);

		addr.reserve(table_data_.size());
		addr.clear();

		for (typename std::vector<T*>::const_iterator it = table_data_.begin(); it != table_data_.end(); ++it)
			addr.push_back(*it);

		return static_cast<unsigned int>(addr.size());
	}

	/**
	 * @brief initialize an element (overwrite with T())
	 * @param id index of the element
	 */
	void init_element(unsigned int id) override
	{
		table_data_[id / CHUNKSIZE][id % CHUNKSIZE] = T();
	}

	/**
	 * @brief copy an element to another one
	 * @param dst destination index
	 * @param src source index
	 */
	void copy_element(unsigned int dst, unsigned int src) override
	{
		table_data_[dst / CHUNKSIZE][dst % CHUNKSIZE] = table_data_[src / CHUNKSIZE][src % CHUNKSIZE];
	}

	/**
	 * @brief swap two elements
	 * @param idx1 first element index
	 * @param idx2 second element index
	 */
	void swap_elements(unsigned int idx1, unsigned int idx2) override
	{
		std::swap(table_data_[idx1 / CHUNKSIZE][idx1 % CHUNKSIZE], table_data_[idx2 / CHUNKSIZE][idx2 % CHUNKSIZE] );
	}

//	void save(std::ostream& fs, unsigned int nb_lines) const
//	{
//		unsigned int nbs[3];
//		nbs[0] = (unsigned int)(table_data_.size());
//		nbs[1] = nb_lines;
//		nbs[2] = CHUNKSIZE*sizeof(T);

//		assert(nb_lines/CHUNKSIZE <= table_data_.size());
//		// TODO: if (nb_lines == 0) nb_lines = CHUNKSIZE*table_data_.size(); ??

//		fs.write(reinterpret_cast<const char*>(nbs),3*sizeof(unsigned int));

//		// no data -> finished
//		if (nbs[0] == 0)
//			return;

//		unsigned int nbca = nbs[0]-1;
//		// save data chunks except last
//		for(unsigned int i=0; i<nbca; ++i)
//		{
//			fs.write(reinterpret_cast<const char*>(table_data_[i]),CHUNKSIZE*sizeof(T));
//		}

//		// save last
//		unsigned nbl = nb_lines - nbca*CHUNKSIZE;
//		fs.write(reinterpret_cast<const char*>(table_data_[nbca]),std::streamsize(nbl*sizeof(T)));
//	}

//	bool load(std::istream& fs)
//	{
//		unsigned int nbs[3];
//		fs.read(reinterpret_cast<char*>(nbs), 3*sizeof(unsigned int));

//		if (nbs[2] != CHUNKSIZE*sizeof(T))
//		{
//			std::cerr << "Error loading ChunkArray wrong CHUNKSIZE"<< std::endl;
//			return false;
//		}

//		this->setNbChunks(nbs[0]);

//		// no data -> finished
//		if (nbs[0] == 0)
//			return true;

//		// load data chunks except last
//		unsigned int nbca = nbs[0]-1;
//		for(unsigned int i = 0; i < nbca; ++i)
//			fs.read(reinterpret_cast<char*>(table_data_[i]),CHUNKSIZE*sizeof(T));

//		// load last chunk
//		unsigned int nbl = nbs[1] - CHUNKSIZE*nbca;
//		fs.read(reinterpret_cast<char*>(table_data_[nbca]),std::streamsize(nbl*sizeof(T)));

//		return true;
//	}

	void save(std::ostream& fs, unsigned int nb_lines) const override
	{
		cgogn_assert(nb_lines / CHUNKSIZE <= get_nb_chunks());

		// no data -> finished
		if (nb_lines == 0)
		{
			std::size_t chunk_bytes = 0;
			serialization::save(fs, &chunk_bytes, 1);
			serialization::save(fs, &nb_lines, 1);
			return;
		}

		unsigned int nbc = get_nb_chunks() - 1u;
		// nb of lines of last chunk
		const unsigned nb = nb_lines - nbc*CHUNKSIZE;

		// compute number of bytes to save
		std::size_t chunk_bytes = 0;
		if (serialization::known_size(table_data_[0]))
		{
			chunk_bytes += nbc * serialization::data_length(table_data_[0], CHUNKSIZE);
		}
		else
		{
			for(unsigned int i = 0u; i < nbc; ++i)
				chunk_bytes += serialization::data_length(table_data_[i], CHUNKSIZE);
		}
		chunk_bytes +=serialization::data_length(table_data_[nbc], nb);
		// save it
		serialization::save(fs, &chunk_bytes, 1);

		// save nb lines
		serialization::save(fs, &nb_lines, 1);


		// save data chunks except last
		for(unsigned int i = 0u; i < nbc; ++i)
		{
			serialization::save(fs, table_data_[i], CHUNKSIZE);
		}

		// save last incomplete chunk
		serialization::save(fs, table_data_[nbc], nb);
	}

	bool load(std::istream& fs) override
	{
		std::size_t chunk_bytes;
		serialization::load(fs, &chunk_bytes, 1);

		unsigned int nb_lines;
		serialization::load(fs, &nb_lines, 1);
		// no data -> finished
		if (nb_lines == 0)
			return true;

		// compute number of chunks
		unsigned int nbc = nb_lines / CHUNKSIZE;
		if (nb_lines % CHUNKSIZE != 0)
			nbc++;

		this->set_nb_chunks(nbc);

		// load data chunks except last
		nbc--;
		for(unsigned int i = 0u; i < nbc; ++i)
			serialization::load(fs, table_data_[i], CHUNKSIZE);

		// load last incomplete chunk
		const unsigned int nb = nb_lines - nbc*CHUNKSIZE;
		serialization::load(fs, table_data_[nbc], nb);

		return true;
	}

	/**
	 * @brief ref operator []
	 * @param i index of element to access
	 * @return ref to the element
	 */
	inline T& operator[](unsigned int i)
	{
		cgogn_assert(i / CHUNKSIZE < table_data_.size());
		return table_data_[i / CHUNKSIZE][i % CHUNKSIZE];
	}

	/**
	 * @brief const ref operator []
	 * @param i index of element to access
	 * @return const ref to the element
	 */
	inline const T& operator[](unsigned int i) const
	{
		cgogn_assert(i / CHUNKSIZE < table_data_.size());
		return table_data_[i / CHUNKSIZE][i % CHUNKSIZE];
	}

	/**
	 * @brief set the value of an element (works also with bool)
	 * @param i index of element to set
	 * @param v value
	 */
	inline void set_value(unsigned int i, const T& v)
	{
		cgogn_assert(i / CHUNKSIZE < table_data_.size());
		table_data_[i / CHUNKSIZE][i % CHUNKSIZE] = v;
	}
};

/**
 * @brief specialized version of ChunkArray for bool data. One bit per bool
 */
template <unsigned int CHUNKSIZE>
class ChunkArray<CHUNKSIZE, bool> : public ChunkArrayGen<CHUNKSIZE>
{
public:

	typedef ChunkArrayGen<CHUNKSIZE> Inherit;
	typedef ChunkArray<CHUNKSIZE, bool> Self;
	typedef unsigned int value_type;

protected:

	/// vector of block pointers
	std::vector<unsigned int*> table_data_;

public:

	inline ChunkArray() : ChunkArrayGen<CHUNKSIZE>()
	{
		table_data_.reserve(1024u);
	}

	ChunkArray(const Self& ca) = delete;
	ChunkArray(Self&& ca) = delete;
	ChunkArray<CHUNKSIZE, bool>& operator=(Self&& ca) = delete;
	ChunkArray<CHUNKSIZE, bool>& operator=(Self& ca) = delete;

	~ChunkArray() override
	{
		for(auto chunk : table_data_)
			delete[] chunk;
	}

	/**
	 * @brief create a ChunkArray<CHUNKSIZE,T>
	 * @return generic pointer
	 */
	ChunkArrayGen<CHUNKSIZE>* clone() const override
	{
		return new Self();
	}

	bool is_boolean_array() const override
	{
		return true;
	}

	/**
	 * @brief add a chunk (T[CHUNKSIZE/32])
	 */
	void add_chunk() override
	{
		// adding the empty parentheses for default-initialization
		table_data_.push_back(new unsigned int[CHUNKSIZE/32u]());
	}

	/**
	 * @brief set number of chunks
	 * @param nbc number of chunks
	 */
	void set_nb_chunks(unsigned int nbc) override
	{
		if (nbc >= table_data_.size())
		{
			for (std::size_t i = table_data_.size(); i < nbc; ++i)
				add_chunk();
		}
		else
		{
			for (std::size_t i = nbc; i < table_data_.size(); ++i)
				delete[] table_data_[i];
			table_data_.resize(nbc);
		}
	}

	/**
	 * @brief get the number of chunks of the array
	 * @return the number of chunks
	 */
	unsigned int get_nb_chunks() const override
	{
		return static_cast<unsigned int>(table_data_.size());
	}

	/**
	 * @brief get the capacity of the array
	 * @return number of allocated lines
	 */
	unsigned int capacity() const override
	{
		return static_cast<unsigned int>(table_data_.size())*CHUNKSIZE/32u;
	}

	/**
	 * @brief clear the array
	 */
	void clear() override
	{
		for(auto chunk : table_data_)
			delete[] chunk;
		table_data_.clear();
	}

	/**
	 * @brief fill a vector with pointers to all chunks
	 * @param addr vector to fill
	 * @param byte_block_size filled with CHUNKSIZE*sizeof(T)
	 * @return addr.size()
	 */
	inline unsigned int get_chunks_pointers(std::vector<void*>& addr, unsigned int& byte_block_size) const override
	{
		byte_block_size = CHUNKSIZE / 8u;

		addr.reserve(table_data_.size());
		addr.clear();

		for (typename std::vector<unsigned int*>::const_iterator it = table_data_.begin(); it != table_data_.end(); ++it)
			addr.push_back(*it);

		return static_cast<unsigned int>(addr.size());
	}

	/**
	 * @brief initialize an element (overwrite with T())
	 * @param id index of the element
	 */
	inline void init_element(unsigned int id) override
	{
		set_false(id);
	}

	/**
	 * @brief copy an element to another one
	 * @param dst destination index
	 * @param src source index
	 */
	inline void copy_element(unsigned int dst, unsigned int src) override
	{
		set_value(dst, this->operator[](src));
	}

	/**
	 * @brief swap two elements
	 * @param idx1 first element index
	 * @param idx2 second element index
	 */
	inline void swap_elements(unsigned int idx1, unsigned int idx2) override
	{
		bool data = this->operator[](idx1);
		set_value(idx1, this->operator[](idx2));
		set_value(idx2, data);
	}

	void save(std::ostream& fs, unsigned int nb_lines) const override
	{
		// no data -> finished
		if (nb_lines == 0)
		{
			std::size_t chunk_bytes = 0;
			serialization::save(fs, &chunk_bytes, 1);
			serialization::save(fs, &nb_lines, 1);
			return;
		}


		// round nbLines to 32 multiple
		if (nb_lines % 32u)
			nb_lines = ((nb_lines / 32u) + 1u) * 32u;

		cgogn_assert(nb_lines / CHUNKSIZE <= table_data_.size());
		// TODO: if (nb_lines==0) nb_lines = CHUNKSIZE*table_data_.size(); ??

		// save number of bytes
		std::size_t chunk_bytes = nb_lines / 8u;
		serialization::save(fs, &chunk_bytes, 1);

		// save number of lines
		serialization::save(fs, &nb_lines, 1);


		const unsigned int nbc = get_nb_chunks() - 1u;
		// save data chunks except last
		for(unsigned int i = 0u; i < nbc; ++i)
		{
			fs.write(reinterpret_cast<const char*>(table_data_[i]), CHUNKSIZE / 8u); // /8 because bool = 1 bit & octet = 8 bit
		}

		// save last
		const unsigned int nb = nb_lines - nbc*CHUNKSIZE;
		fs.write(reinterpret_cast<const char*>(table_data_[nbc]), nb / 8u);
	}

	bool load(std::istream& fs) override
	{
		// get number of bytes
		std::size_t chunk_bytes;
		serialization::load(fs, &chunk_bytes, 1);

		// get number of lines to load
		unsigned int nb_lines;
		serialization::load(fs, &nb_lines, 1);

		// no data -> finished
		if (nb_lines == 0)
			return true;

		// compute number of chunks
		unsigned int nbc = nb_lines / CHUNKSIZE;
		if (nb_lines % CHUNKSIZE != 0u)
			nbc++;

		this->set_nb_chunks(nbc);

		// load data chunks except last
		nbc--;
		for(unsigned int i = 0u; i < nbc; ++i)
			fs.read(reinterpret_cast<char*>(table_data_[i]), CHUNKSIZE / 8u);// /8 because bool = 1 bit & octet = 8 bit

		// load last chunk
		unsigned int nb = nb_lines - nbc*CHUNKSIZE;
		fs.read(reinterpret_cast<char*>(table_data_[nbc]), nb / 8u);

		return true;
	}

	/**
	 * @brief operator []
	 * @param i index of element to access
	 * @return value of the element
	 */
	inline bool operator[](unsigned int i) const
	{
		const unsigned int jj = i / CHUNKSIZE;
		cgogn_assert(jj < table_data_.size());
		const unsigned int j = i % CHUNKSIZE;
		const unsigned int x = j/32u;
		const unsigned int y = j%32u;

		const unsigned int mask = 1u << y;

		return (table_data_[jj][x] & mask) != 0u;
	}

	inline void set_false(unsigned int i)
	{
		const unsigned int jj = i / CHUNKSIZE;
		cgogn_assert(jj < table_data_.size());
		const unsigned int j = i % CHUNKSIZE;
		const unsigned int x = j / 32u;
		const unsigned int y = j % 32u;
		const unsigned int mask = 1u << y;
		table_data_[jj][x] &= ~mask;
	}

	inline void set_true(unsigned int i)
	{
		const unsigned int jj = i / CHUNKSIZE;
		cgogn_assert(jj < table_data_.size());
		const unsigned int j = i % CHUNKSIZE;
		const unsigned int x = j / 32u;
		const unsigned int y = j % 32u;
		const unsigned int mask = 1u << y;
		table_data_[jj][x] |= mask;
	}

	inline void set_value(unsigned int i, bool b)
	{
		const unsigned int jj = i / CHUNKSIZE;
		cgogn_assert(jj < table_data_.size());
		const unsigned int j = i % CHUNKSIZE;
		const unsigned int x = j / 32u;
		const unsigned int y = j % 32u;
		const unsigned int mask = 1u << y;
		if (b)
			table_data_[jj][x] |= mask;
		else
			table_data_[jj][x] &= ~mask;
	}

	/**
	 * @brief special optimized version of setFalse when goal is to set all to false;
	 * @param i index of element to set to false
	 *
	 * This version overwrites element AND SOME OF HIS NEIGHBOURS with 0
	 * Use only if final goal is to set all array to 0 (MarkerStore)
	 */
	inline void set_false_byte(unsigned int i)
	{
		const unsigned int jj = i / CHUNKSIZE;
		cgogn_assert(jj < table_data_.size());
		const unsigned int j = (i % CHUNKSIZE)/32u;
		table_data_[jj][j] = 0u;
	}

	inline void all_false()
	{
		for (auto ptr : table_data_)
		{
			for (unsigned int j = 0u; j < CHUNKSIZE/32u; ++j)
				*ptr++ = 0u;
		}
	}

//	inline void all_true()
//	{
//		for (auto ptr : table_data_)
//		{
//			for (unsigned int j = 0u; j < CHUNKSIZE/32u; ++j)
//				*ptr++ = 0xffffffff;
//		}
//	}
};

} // namespace cgogn

#endif // CORE_CONTAINER_CHUNK_ARRAY_H_

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

#ifndef CGOGN_CORE_CONTAINER_CHUNK_ARRAY_H_
#define CGOGN_CORE_CONTAINER_CHUNK_ARRAY_H_

#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <cstring>

#include <cgogn/core/dll.h>
#include <cgogn/core/container/chunk_array_gen.h>
#include <cgogn/core/utils/name_types.h>
#include <cgogn/core/utils/serialization.h>
#include <cgogn/core/utils/assert.h>
#include <cgogn/core/utils/logger.h>
#include <cgogn/core/utils/endian.h>
#include <cgogn/core/utils/string.h>

#include <cgogn/core/cmap/map_traits.h>

namespace cgogn
{

/**
 *	@brief chunk array class storage
 *	@tparam CHUNK_SIZE size of each chunk (in T, not in bytes!), must be a power of 2 >=32
 *	@tparam T type of stored data
 */
template <uint32 CHUNK_SIZE, typename T>
class ChunkArray : public ChunkArrayGen<CHUNK_SIZE>
{
public:

	using Inherit = ChunkArrayGen<CHUNK_SIZE>;
	using Self = ChunkArray<CHUNK_SIZE, T>;
	using value_type = T;

protected:

	// vector of block pointers
	std::vector<T*> table_data_;

public:

	/**
	 * @brief Constructor of ChunkArray
	 */
	inline ChunkArray(const std::string& name) :
		Inherit(name, name_of_type(T()))
	{
		table_data_.reserve(1024u);
	}

	inline ChunkArray() : Inherit()
	{
		table_data_.reserve(1024u);
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ChunkArray);

	~ChunkArray() override
	{
		for(auto chunk : table_data_)
			delete[] chunk;
	}

	/**
	 * @brief create a ChunkArray<CHUNK_SIZE,T>
	 * @return generic pointer
	 */
	std::unique_ptr<Inherit> clone(const std::string& clone_name) const override
	{
		if (clone_name == this->name_)
			return nullptr;
		return std::unique_ptr<Inherit>(new Self(clone_name));
	}

	bool swap(Inherit* cag) override
	{
		Self* ca = dynamic_cast<Self*>(cag);
		if (!ca)
		{
			cgogn_log_warning("swap") << "Warning: trying to swap attribute of different type";
			return false;
		}

		table_data_.swap(ca->table_data_);
		return true;
	}

//	bool is_boolean_array() const override
//	{
//		return false;
//	}

	/**
	 * @brief add a chunk (T[CHUNK_SIZE])
	 */
	void add_chunk() override
	{
		table_data_.push_back(new T[CHUNK_SIZE]());
	}

	/**
	 * @brief set number of chunks
	 * @param nbc number of chunks
	 */
	void set_nb_chunks(uint32 nbc) override
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
	uint32 nb_chunks() const override
	{
		return uint32(table_data_.size());
	}

	/**
	 * @brief get the capacity of the array
	 * @return number of allocated lines
	 */
	uint32 capacity() const override
	{
		return uint32(table_data_.size())*CHUNK_SIZE;
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
	 * @param byte_chunk_size filled with CHUNK_SIZE*sizeof(T)
	 * @return addr.size()
	 */
	std::vector<const void*> chunks_pointers(uint32& byte_chunk_size) const override
	{
		std::vector<const void*> addr;
		byte_chunk_size = CHUNK_SIZE * sizeof(T);

		addr.reserve(table_data_.size());

		for (typename std::vector<T*>::const_iterator it = table_data_.begin(); it != table_data_.end(); ++it)
			addr.push_back(*it);

		return addr;
	}

	/**
	 * @brief initialize an element (overwrite with T())
	 * @param id index of the element
	 */
	void init_element(uint32 id) override
	{
		table_data_[id / CHUNK_SIZE][id % CHUNK_SIZE] = T();
	}

	/**
	 * @brief copy an element to another one
	 * @param dst destination index
	 * @param src source index
	 */
	void copy_element(uint32 dst, uint32 src) override
	{
		table_data_[dst / CHUNK_SIZE][dst % CHUNK_SIZE] = table_data_[src / CHUNK_SIZE][src % CHUNK_SIZE];
	}

	/**
	 * @brief copy an element (of another C.A.) to another one
	 * @param dst destination index
	 * @param cag_src chunk_array source (Precond: same type as this)
	 * @param src source index
	 */
	void copy_external_element(uint32 dst, Inherit* cag_src, uint32 src) override
	{
		Self* ca = static_cast<Self*>(cag_src);
		table_data_[dst / CHUNK_SIZE][dst % CHUNK_SIZE] = ca->table_data_[src / CHUNK_SIZE][src % CHUNK_SIZE];
	}

	/**
	 * @brief move an element to another one
	 * @param dst destination index
	 * @param src source index
	 */
	void move_element(uint32 dst, uint32 src) override
	{
		table_data_[dst / CHUNK_SIZE][dst % CHUNK_SIZE] = std::move(table_data_[src / CHUNK_SIZE][src % CHUNK_SIZE]);
	}

	/**
	 * @brief swap two elements
	 * @param idx1 first element index
	 * @param idx2 second element index
	 */
	void swap_elements(uint32 idx1, uint32 idx2) override
	{
// small workaround to avoid difficulties with std::swap when _GLIBCXX_DEBUG is defined.
#ifndef _GLIBCXX_DEBUG
		std::swap(table_data_[idx1 / CHUNK_SIZE][idx1 % CHUNK_SIZE], table_data_[idx2 / CHUNK_SIZE][idx2 % CHUNK_SIZE] );
#else
		T& a = table_data_[idx1 / CHUNK_SIZE][idx1 % CHUNK_SIZE];
		T& b = table_data_[idx2 / CHUNK_SIZE][idx2 % CHUNK_SIZE];
		T tmp(std::move(a));
		a = std::move(b);
		b = std::move(tmp);
#endif // _GLIBCXX_DEBUG
	}

//	void save(std::ostream& fs, uint32 nb_lines) const
//	{
//		uint32 nbs[3];
//		nbs[0] = (uint32)(table_data_.size());
//		nbs[1] = nb_lines;
//		nbs[2] = CHUNK_SIZE*sizeof(T);

//		assert(nb_lines/CHUNK_SIZE <= table_data_.size());
//		// TODO: if (nb_lines == 0) nb_lines = CHUNK_SIZE*table_data_.size(); ??

//		fs.write(reinterpret_cast<const char*>(nbs),3*sizeof(uint32));

//		// no data -> finished
//		if (nbs[0] == 0)
//			return;

//		uint32 nbca = nbs[0]-1;
//		// save data chunks except last
//		for(uint32 i=0; i<nbca; ++i)
//		{
//			fs.write(reinterpret_cast<const char*>(table_data_[i]),CHUNK_SIZE*sizeof(T));
//		}

//		// save last
//		unsigned nbl = nb_lines - nbca*CHUNK_SIZE;
//		fs.write(reinterpret_cast<const char*>(table_data_[nbca]),std::streamsize(nbl*sizeof(T)));
//	}

//	bool load(std::istream& fs)
//	{
//		uint32 nbs[3];
//		fs.read(reinterpret_cast<char*>(nbs), 3*sizeof(uint32));

//		if (nbs[2] != CHUNK_SIZE*sizeof(T))
//		{
//			std::cerr << "Error loading ChunkArray wrong CHUNK_SIZE"<< std::endl;
//			return false;
//		}

//		this->setNbChunks(nbs[0]);

//		// no data -> finished
//		if (nbs[0] == 0)
//			return true;

//		// load data chunks except last
//		uint32 nbca = nbs[0]-1;
//		for(uint32 i = 0; i < nbca; ++i)
//			fs.read(reinterpret_cast<char*>(table_data_[i]),CHUNK_SIZE*sizeof(T));

//		// load last chunk
//		uint32 nbl = nbs[1] - CHUNK_SIZE*nbca;
//		fs.read(reinterpret_cast<char*>(table_data_[nbca]),std::streamsize(nbl*sizeof(T)));

//		return true;
//	}

	void save(std::ostream& fs, uint32 nb_lines) const override
	{
		cgogn_assert(fs.good());
		cgogn_assert(nb_lines / CHUNK_SIZE <= nb_chunks());

		// no data -> finished
		if (nb_lines == 0)
		{
			std::size_t chunk_bytes = 0;
			serialization::save(fs, &chunk_bytes, 1);
			serialization::save(fs, &nb_lines, 1);
			return;
		}

		uint32 nbc = nb_chunks() - 1u;
		// nb of lines of last chunk
		const unsigned nb = nb_lines - nbc*CHUNK_SIZE;

		// compute number of bytes to save
		std::size_t chunk_bytes = 0;
		if (serialization::known_size(table_data_[0]))
		{
			chunk_bytes += nbc * serialization::data_length(table_data_[0], CHUNK_SIZE);
		}
		else
		{
			for(uint32 i = 0u; i < nbc; ++i)
				chunk_bytes += serialization::data_length(table_data_[i], CHUNK_SIZE);
		}
		chunk_bytes +=serialization::data_length(table_data_[nbc], nb);
		// save it
		serialization::save(fs, &chunk_bytes, 1);

		// save nb lines
		serialization::save(fs, &nb_lines, 1);


		// save data chunks except last
		for(uint32 i = 0u; i < nbc; ++i)
		{
			serialization::save(fs, table_data_[i], CHUNK_SIZE);
		}

		// save last incomplete chunk
		serialization::save(fs, table_data_[nbc], nb);

		cgogn_assert(fs.good());
	}

	bool load(std::istream& fs) override
	{
		cgogn_assert(fs.good());

		std::size_t chunk_bytes;
		serialization::load(fs, &chunk_bytes, 1);

		uint32 nb_lines;
		serialization::load(fs, &nb_lines, 1);
		// no data -> finished
		if (nb_lines == 0)
			return true;

		// compute number of chunks
		uint32 nbc = nb_lines / CHUNK_SIZE;
		if (nb_lines % CHUNK_SIZE != 0)
			nbc++;

		this->set_nb_chunks(nbc);

		// load data chunks except last
		nbc--;
		for(uint32 i = 0u; i < nbc; ++i)
			serialization::load(fs, table_data_[i], CHUNK_SIZE);

		// load last incomplete chunk
		const uint32 nb = nb_lines - nbc*CHUNK_SIZE;
		serialization::load(fs, table_data_[nbc], nb);
		cgogn_assert(fs.good());

		return true;
	}

	/**
	 * @brief ref operator[]
	 * @param i index of element to access
	 * @return ref to the element
	 */
	inline T& operator[](uint32 i)
	{
		cgogn_assert(i / CHUNK_SIZE < table_data_.size());
		return table_data_[i / CHUNK_SIZE][i % CHUNK_SIZE];
	}

	/**
	 * @brief const ref operator[]
	 * @param i index of element to access
	 * @return const ref to the element
	 */
	inline const T& operator[](uint32 i) const
	{
		cgogn_assert(i / CHUNK_SIZE < table_data_.size());
		return table_data_[i / CHUNK_SIZE][i % CHUNK_SIZE];
	}

	/**
	 * @brief set the value of an element (works also with bool)
	 * @param i index of element to set
	 * @param v value
	 */
	inline void set_value(uint32 i, const T& v)
	{
		cgogn_assert(i / CHUNK_SIZE < table_data_.size());
		table_data_[i / CHUNK_SIZE][i % CHUNK_SIZE] = v;
	}

	inline void set_all_values(const T& v)
	{
		for (T* chunk : table_data_)
		{
			for(uint32 i = 0; i < CHUNK_SIZE; ++i)
				*chunk++ = v;
		}
	}

	virtual std::string nested_type_name() const override
	{
		return name_of_type(nested_type<T>());
	}

	virtual uint32 nb_components() const override
	{
		// Warning : the line 0 might be unused.
		return cgogn::nb_components(this->operator[](0u));
	}

	virtual void export_element(uint32 idx, std::ostream& o, bool binary, bool little_endian, std::size_t precision) const override
	{
		switch (precision)
		{
			case 1ul: serialization::ostream_writer<T, 1ul>(o, this->operator[](idx), binary, little_endian); break;
			case 2ul: serialization::ostream_writer<T, 2ul>(o, this->operator[](idx), binary, little_endian); break;
			case 4ul: serialization::ostream_writer<T, 4ul>(o, this->operator[](idx), binary, little_endian); break;
			default:  serialization::ostream_writer<T, 8ul>(o, this->operator[](idx), binary, little_endian); break;
		}
	}

	virtual void import_element(uint32 idx, std::istream& in) override
	{
		serialization::parse(in, this->operator [](idx));
	}


	virtual const void* element_ptr(uint32 idx) const override
	{
		return &(this->operator[](idx));
	}

	virtual uint32 element_size() const override
	{
		return sizeof(std::declval<T>());
	}
};

/**
 * @brief separate version of ChunkArray specialized for bool data. One bit per bool.
 */
template <uint32 CHUNK_SIZE>
class ChunkArrayBool : public ChunkArrayGen<CHUNK_SIZE>
{
public:

	using Inherit = ChunkArrayGen<CHUNK_SIZE>;
	using Self = ChunkArrayBool;
	using value_type = uint32;

protected:

	// ensure we can use CHUNK_SIZE value < 32
	const uint32 BOOLS_PER_INT = (CHUNK_SIZE<32u) ? CHUNK_SIZE : 32u;

	// vector of block pointers
	std::vector<uint32*> table_data_;

public:

	inline ChunkArrayBool(const std::string& name) :
		Inherit(name, name_of_type(bool()))
	{
		table_data_.reserve(1024u);
	}

	inline ChunkArrayBool() : Inherit()
	{
		table_data_.reserve(1024u);
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ChunkArrayBool);

	~ChunkArrayBool() override
	{
		for(auto chunk : table_data_)
			delete[] chunk;
	}

	/**
	 * @brief create a ChunkArray<CHUNK_SIZE,T>
	 * @return generic pointer
	 */
	std::unique_ptr<Inherit> clone(const std::string& clone_name) const override
	{
		if (clone_name == this->name_)
			return nullptr;
		return std::unique_ptr<Inherit>(new Self(clone_name));
	}

	bool swap(Inherit* cag) override
	{
		Self* ca = dynamic_cast<Self*>(cag);
		if (!ca)
		{
			cgogn_log_warning("swap") << "Warning: trying to swap attribute of different type";
			return false;
		}

		table_data_.swap(ca->table_data_);
		return true;
	}

	/**
	 * @brief add a chunk (T[CHUNK_SIZE/32])
	 */
	void add_chunk() override
	{
		// adding the empty parentheses for default-initialization
		table_data_.push_back(new uint32[CHUNK_SIZE/BOOLS_PER_INT]());
	}

	/**
	 * @brief set number of chunks
	 * @param nbc number of chunks
	 */
	void set_nb_chunks(uint32 nbc) override
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
	uint32 nb_chunks() const override
	{
		return uint32(table_data_.size());
	}

	/**
	 * @brief get the capacity of the array
	 * @return number of allocated lines
	 */
	uint32 capacity() const override
	{
		return uint32(table_data_.size())*CHUNK_SIZE/BOOLS_PER_INT;
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
	 * @param byte_block_size filled with CHUNK_SIZE*sizeof(T)
	 * @return addr.size()
	 */
	inline std::vector<const void*> chunks_pointers(uint32& byte_block_size) const override
	{
		std::vector<const void*> addr;
		byte_block_size = CHUNK_SIZE / 8u;

		addr.reserve(table_data_.size());

		for (typename std::vector<uint32*>::const_iterator it = table_data_.begin(); it != table_data_.end(); ++it)
			addr.push_back(*it);

		return addr;
	}

	/**
	 * @brief initialize an element (overwrite with T())
	 * @param id index of the element
	 */
	inline void init_element(uint32 id) override
	{
		set_false(id);
	}

	/**
	 * @brief copy an element to another one
	 * @param dst destination index
	 * @param src source index
	 */
	inline void copy_element(uint32 dst, uint32 src) override
	{
		set_value(dst, this->operator[](src));
	}

	/**
	 * @brief copy an element (of another C.A.) to another one
	 * @param dst destination index
	 * @param cag_src chunk_array source (Precond: same type as this)
	 * @param src source index
	 */
	void copy_external_element(uint32 dst, Inherit* cag_src, uint32 src) override
	{
		Self* ca = static_cast<Self*>(cag_src);
		set_value(dst, ca->operator[](src));
	}

	/**
	 * @brief swap two elements
	 * @param idx1 first element index
	 * @param idx2 second element index
	 */
	inline void swap_elements(uint32 idx1, uint32 idx2) override
	{
		bool data = this->operator[](idx1);
		set_value(idx1, this->operator[](idx2));
		set_value(idx2, data);
	}

	void save(std::ostream& fs, uint32 nb_lines) const override
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
		if (nb_lines % BOOLS_PER_INT)
			nb_lines = ((nb_lines / BOOLS_PER_INT) + 1u) * BOOLS_PER_INT;

		cgogn_assert(nb_lines / CHUNK_SIZE <= table_data_.size());
		// TODO: if (nb_lines==0) nb_lines = CHUNK_SIZE*table_data_.size(); ??

		// save number of bytes
		std::size_t chunk_bytes = nb_lines / 8u;
		serialization::save(fs, &chunk_bytes, 1);

		// save number of lines
		serialization::save(fs, &nb_lines, 1);

		const uint32 nbc = nb_chunks() - 1u;
		// save data chunks except last
		for(uint32 i = 0u; i < nbc; ++i)
			fs.write(reinterpret_cast<const char*>(table_data_[i]), CHUNK_SIZE / 8u); // /8 because bool = 1 bit & octet = 8 bit

		// save last
		const uint32 nb = nb_lines - nbc * CHUNK_SIZE;
		fs.write(reinterpret_cast<const char*>(table_data_[nbc]), nb / 8u);
	}

	bool load(std::istream& fs) override
	{
		// get number of bytes
		std::size_t chunk_bytes;
		serialization::load(fs, &chunk_bytes, 1);

		// get number of lines to load
		uint32 nb_lines;
		serialization::load(fs, &nb_lines, 1);

		// no data -> finished
		if (nb_lines == 0)
			return true;

		// compute number of chunks
		uint32 nbc = nb_lines / CHUNK_SIZE;
		if (nb_lines % CHUNK_SIZE != 0u)
			nbc++;

		this->set_nb_chunks(nbc);

		// load data chunks except last
		nbc--;
		for(uint32 i = 0u; i < nbc; ++i)
			fs.read(reinterpret_cast<char*>(table_data_[i]), CHUNK_SIZE / 8u);// /8 because bool = 1 bit & octet = 8 bit

		// load last chunk
		uint32 nb = nb_lines - nbc*CHUNK_SIZE;
		fs.read(reinterpret_cast<char*>(table_data_[nbc]), nb / 8u);

		return true;
	}

	/**
	 * @brief operator[]
	 * @param i index of element to access
	 * @return value of the element
	 */
	inline bool operator[](uint32 i) const
	{
		const uint32 jj = i / CHUNK_SIZE;
		cgogn_assert(jj < table_data_.size());
		const uint32 j = i % CHUNK_SIZE;
		const uint32 x = j / BOOLS_PER_INT;
		const uint32 y = j % BOOLS_PER_INT;

		const uint32 mask = 1u << y;

		return (table_data_[jj][x] & mask) != 0u;
	}

	inline void set_false(uint32 i)
	{
		const uint32 jj = i / CHUNK_SIZE;
		cgogn_assert(jj < table_data_.size());
		const uint32 j = i % CHUNK_SIZE;
		const uint32 x = j / BOOLS_PER_INT;
		const uint32 y = j % BOOLS_PER_INT;
		const uint32 mask = 1u << y;
		table_data_[jj][x] &= ~mask;
	}

	inline void set_true(uint32 i)
	{
		const uint32 jj = i / CHUNK_SIZE;
		cgogn_assert(jj < table_data_.size());
		const uint32 j = i % CHUNK_SIZE;
		const uint32 x = j / BOOLS_PER_INT;
		const uint32 y = j % BOOLS_PER_INT;
		const uint32 mask = 1u << y;
		table_data_[jj][x] |= mask;
	}

	inline void set_value(uint32 i, bool b)
	{
		const uint32 jj = i / CHUNK_SIZE;
		cgogn_assert(jj < table_data_.size());
		const uint32 j = i % CHUNK_SIZE;
		const uint32 x = j / BOOLS_PER_INT;
		const uint32 y = j % BOOLS_PER_INT;
		const uint32 mask = 1u << y;
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
	inline void set_false_byte(uint32 i)
	{
		const uint32 jj = i / CHUNK_SIZE;
		cgogn_assert(jj < table_data_.size());
		const uint32 j = (i % CHUNK_SIZE) / BOOLS_PER_INT;
		table_data_[jj][j] = 0u;
	}

	inline void all_false()
	{
		for (uint32 * const ptr : table_data_)
		{
//#pragma omp for
			for (int32 j = 0; j < int32(CHUNK_SIZE / BOOLS_PER_INT); ++j)
				ptr[j] = 0u;
		}
	}

//	inline void all_true()
//	{
//		for (auto ptr : table_data_)
//		{
//			for (uint32 j = 0u; j < CHUNK_SIZE/BOOLS_PER_INT; ++j)
//				*ptr++ = 0xffffffff;
//		}
//	}

	virtual std::string nested_type_name() const override
	{
		return name_of_type(bool());
	}

	virtual uint32 nb_components() const override
	{
		return 1u;
	}

	virtual void export_element(uint32 idx, std::ostream& o, bool binary, bool little_endian, std::size_t /*precision*/) const override
	{
		serialization::ostream_writer(o, this->operator[](idx),binary, little_endian);
	}

	virtual void import_element(uint32 idx, std::istream& in) override
	{
		std::string val;
		in >> val;
		val = to_lower(val);
		const bool b = (val == "true") || (std::stoi(val) == 1);
		if (b)
			set_true(idx);
		else
			set_false(idx);
	}

	virtual const void* element_ptr(uint32) const override
	{
		return nullptr; // shall not be used with ChunkArrayBool
	}

	virtual uint32 element_size() const override
	{
		return UINT32_MAX;
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CONTAINER_CHUNK_ARRAY_CPP_))
extern template class CGOGN_CORE_API ChunkArray<CGOGN_CHUNK_SIZE, bool>;
extern template class CGOGN_CORE_API ChunkArray<CGOGN_CHUNK_SIZE, uint32>;
extern template class CGOGN_CORE_API ChunkArray<CGOGN_CHUNK_SIZE, unsigned char>;
extern template class CGOGN_CORE_API ChunkArray<CGOGN_CHUNK_SIZE, std::array<float32, 3>>;
extern template class CGOGN_CORE_API ChunkArray<CGOGN_CHUNK_SIZE, std::array<float64, 3>>;
extern template class CGOGN_CORE_API ChunkArrayBool<CGOGN_CHUNK_SIZE>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CONTAINER_CHUNK_ARRAY_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CONTAINER_CHUNK_ARRAY_H_

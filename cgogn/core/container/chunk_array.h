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

#include <cgogn/core/cgogn_core_export.h>
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

	inline ChunkArray() : Inherit("",name_of_type(T()))
	{
		table_data_.reserve(1024u);
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ChunkArray);

	~ChunkArray() override
	{
		for(auto chunk : table_data_)
			delete[] chunk;
	}

	std::string nested_type_name() const override
	{
		return name_of_type(nested_type<T>());
	}

	uint32 nb_components() const override
	{
		// Warning : the line 0 might be unused.
		return cgogn::nb_components(this->operator[](0u));
	}

	uint32 element_size() const override
	{
		return sizeof(std::declval<T>());
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
	 * @brief return a vector with pointers to all chunks
	 * @param byte_chunk_size filled with CHUNK_SIZE*sizeof(T)
	 * @return the vector of pointers
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
	 * @brief create a ChunkArray<CHUNK_SIZE,T>
	 * @return generic pointer
	 */
	std::unique_ptr<Inherit> clone(const std::string& clone_name) const override
	{
		if (clone_name == this->name_)
			return nullptr;
		return std::unique_ptr<Inherit>(new Self(clone_name));
	}

	bool swap_data(Inherit* cag) override
	{
		Self* ca = dynamic_cast<Self*>(cag);
		if (!ca)
		{
			cgogn_log_warning("swap_data") << "Trying to swap attribute of different types";
			return false;
		}
		table_data_.swap(ca->table_data_);
		return true;
	}

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
	 * @brief clear the array
	 */
	void clear() override
	{
		for(auto chunk : table_data_)
			delete[] chunk;
		table_data_.clear();
		table_data_.shrink_to_fit();
		table_data_.reserve(1024u);
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

	void export_element(uint32 idx, std::ostream& o, bool binary, bool little_endian, std::size_t precision) const override
	{
		switch (precision)
		{
			case 1ul: serialization::ostream_writer<T, 1ul>(o, this->operator[](idx), binary, little_endian); break;
			case 2ul: serialization::ostream_writer<T, 2ul>(o, this->operator[](idx), binary, little_endian); break;
			case 4ul: serialization::ostream_writer<T, 4ul>(o, this->operator[](idx), binary, little_endian); break;
			default:  serialization::ostream_writer<T, 8ul>(o, this->operator[](idx), binary, little_endian); break;
		}
	}

	void import_element(uint32 idx, std::istream& in) override
	{
		serialization::parse(in, this->operator [](idx));
	}

	const void* element_ptr(uint32 idx) const override
	{
		return &(this->operator[](idx));
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

	void copy(const Inherit& cag_src) override
	{
		clear();
		const Self* ca = dynamic_cast<const Self*>(&cag_src);
		if (ca == nullptr)
		{
			cgogn_log_error("ChunkArray") << "trying to copy between different types";
			return;
		}
		for (T* chunk : ca->table_data_)
		{
			add_chunk();
			T* ptr = table_data_.back();
			for(uint32 i=0; i< CHUNK_SIZE; ++i)
				*ptr++ = *chunk++;
		}
	}

	void copy_data(const Inherit& cag_src) override
	{
		const Self* ca = dynamic_cast<const Self*>(&cag_src);
		if (ca == nullptr)
		{
			cgogn_log_error("ChunkArray") << "trying to copy between different types";
			return;
		}

		cgogn_message_assert(ca->nb_chunks()==this->nb_chunks(), "copy_data only with same sized ChunkArray");

		auto td = table_data_.begin();
		for (T* chunk : ca->table_data_)
		{
			T* ptr = *td++;
			for(uint32 i=0; i< CHUNK_SIZE; ++i)
				*ptr++ = *chunk++;
		}
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
	static const uint32 BOOLS_PER_INT = (CHUNK_SIZE<32u) ? CHUNK_SIZE : 32u;

	// vector of block pointers
	std::vector<uint32*> table_data_;

public:

	inline ChunkArrayBool(const std::string& name) :
		Inherit(name, name_of_type(bool()))
	{
		table_data_.reserve(1024u);
	}

	inline ChunkArrayBool() : Inherit("", name_of_type(bool()))
	{
		table_data_.reserve(1024u);
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ChunkArrayBool);

	~ChunkArrayBool() override
	{
		for(auto chunk : table_data_)
			delete[] chunk;
	}

	std::string nested_type_name() const override
	{
		return name_of_type(bool());
	}

	uint32 nb_components() const override
	{
		return 1u;
	}

	uint32 element_size() const override
	{
		return UINT32_MAX;
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
	 * @brief return a vector with pointers to all chunks
	 * @param byte_block_size filled with CHUNK_SIZE*sizeof(T)
	 * @return the vector of pointers
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
	 * @brief create a ChunkArray<CHUNK_SIZE,T>
	 * @return generic pointer
	 */
	std::unique_ptr<Inherit> clone(const std::string& clone_name) const override
	{
		if (clone_name == this->name_)
			return nullptr;
		return std::unique_ptr<Inherit>(new Self(clone_name));
	}

	bool swap_data(Inherit* cag) override
	{
		Self* ca = dynamic_cast<Self*>(cag);
		if (!ca)
		{
			cgogn_log_warning("swap_data") << "Trying to swap attribute of different types";
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
	 * @brief clear the array
	 */
	void clear() override
	{
		for(auto chunk : table_data_)
			delete[] chunk;
		table_data_.clear();
		table_data_.shrink_to_fit();
		table_data_.reserve(1024u);
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
		const bool data = this->operator[](idx1);
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
		const uint32 nb = nb_lines - nbc*CHUNK_SIZE;
		fs.read(reinterpret_cast<char*>(table_data_[nbc]), nb / 8u);

		return true;
	}

	void export_element(uint32 idx, std::ostream& o, bool binary, bool little_endian, std::size_t /*precision*/) const override
	{
		serialization::ostream_writer(o, this->operator[](idx),binary, little_endian);
	}

	void import_element(uint32 idx, std::istream& in) override
	{
		std::string val;
		in >> val;
		val = to_lower(val);
		const bool b = (val == "true") || (std::stoi(val) != 0);
		set_value(idx,b);
	}

	const void* element_ptr(uint32) const override
	{
		return nullptr; // shall not be used with ChunkArrayBool
	}

	/**
	 * @brief operator[]
	 * @param i index of element to access
	 * @return value of the element
	 */
	inline bool operator[](uint32 i) const
	{
		cgogn_assert(i / CHUNK_SIZE < table_data_.size());
		return (table_data_[i / CHUNK_SIZE][(i % CHUNK_SIZE)/BOOLS_PER_INT] & (1u << ((i % CHUNK_SIZE) % BOOLS_PER_INT))) != 0u;
	}

	inline void set_false(uint32 i)
	{
		cgogn_assert(i / CHUNK_SIZE < table_data_.size());
		table_data_[i / CHUNK_SIZE][(i % CHUNK_SIZE)/ BOOLS_PER_INT] &= ~(1u << ((i % CHUNK_SIZE) % BOOLS_PER_INT));
	}

	inline void set_true(uint32 i)
	{
		cgogn_assert(i / CHUNK_SIZE < table_data_.size());
		table_data_[i / CHUNK_SIZE][(i % CHUNK_SIZE)/ BOOLS_PER_INT] |= 1u << ((i % CHUNK_SIZE) % BOOLS_PER_INT);
	}

	inline void set_value(uint32 i, bool b)
	{
		if (b)
			set_true(i);
		else
			set_false(i);
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
		cgogn_assert(i / CHUNK_SIZE < table_data_.size());
		table_data_[i / CHUNK_SIZE][(i % CHUNK_SIZE) / BOOLS_PER_INT] = 0u;
	}

	inline void all_false()
	{
		for (uint32* const ptr : table_data_)
		{
			for (int32 j = 0; j < int32(CHUNK_SIZE / BOOLS_PER_INT); ++j)
				ptr[j] = 0u;
		}
	}

	void copy(const Inherit& cag_src) override
	{
		clear();
		const Self* ca = dynamic_cast<const Self*>(&cag_src);
		if (ca == nullptr)
		{
			cgogn_log_error("ChunkArray") << "trying to copy between different types";
			return;
		}
		for (uint32* chunk : ca->table_data_)
		{
			add_chunk();
			uint32* ptr = table_data_.back();
			for(uint32 i=0; i< CHUNK_SIZE/BOOLS_PER_INT; ++i)
				*ptr++ = *chunk++;
		}
	}

	void copy_data(const Inherit& cag_src) override
	{
		const Self* ca = dynamic_cast<const Self*>(&cag_src);
		if (ca == nullptr)
		{
			cgogn_log_error("ChunkArray") << "trying to copy between different types";
			return;
		}
		cgogn_message_assert(ca->nb_chunks()==this->nb_chunks(), "copy_data only with same sized ChunkArray");

		auto td = table_data_.begin();
		for (uint32* chunk : ca->table_data_)
		{
			uint32* ptr = *td++;
			for(uint32 i=0; i< CHUNK_SIZE; ++i)
				*ptr++ = *chunk++;
		}
	}

	inline uint32 count_true()
	{
		uint32 nb=0;
		for (uint32* ptr : table_data_)
		{
			for (int32 j = 0; j < int32(CHUNK_SIZE / BOOLS_PER_INT); ++j)
			{
				uint32 word = ptr[j];
				while (word != 0)
				{
					nb += (word & 1u);
					word >>= 1; // /=2 ?
				}
			}
		}
		return nb;
	}

};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_EXTERNAL_TEMPLATES_CPP_))
//extern template class CGOGN_CORE_EXPORT ChunkArray<CGOGN_CHUNK_SIZE, bool>;
extern template class CGOGN_CORE_EXPORT ChunkArray<CGOGN_CHUNK_SIZE, uint32>;
extern template class CGOGN_CORE_EXPORT ChunkArray<CGOGN_CHUNK_SIZE, uint8>;
extern template class CGOGN_CORE_EXPORT ChunkArray<CGOGN_CHUNK_SIZE, std::array<float32, 3>>;
extern template class CGOGN_CORE_EXPORT ChunkArray<CGOGN_CHUNK_SIZE, std::array<float64, 3>>;
extern template class CGOGN_CORE_EXPORT ChunkArrayBool<CGOGN_CHUNK_SIZE>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_EXTERNAL_TEMPLATES_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CONTAINER_CHUNK_ARRAY_H_

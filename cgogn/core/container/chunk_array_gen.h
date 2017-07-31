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

#ifndef CGOGN_CORE_CONTAINER_CHUNK_ARRAY_GEN_H_
#define CGOGN_CORE_CONTAINER_CHUNK_ARRAY_GEN_H_

#include <cgogn/core/utils/serialization.h>
#include <cgogn/core/dll.h>

#include <cgogn/core/cmap/map_traits.h>

#include <vector>
#include <iostream>
#include <algorithm>
#include <memory>

namespace cgogn
{

template <uint32 CHUNK_SIZE, typename T_REF> class ChunkArrayContainer;

/**
 * @brief Virtual version of ChunkArray
 */
template <uint32 CHUNK_SIZE>
class ChunkArrayGen
{
public:

	using Self = ChunkArrayGen<CHUNK_SIZE>;

	template <uint32 CS, typename T_REF> friend class ChunkArrayContainer;

	inline ChunkArrayGen(const std::string& name, const std::string& type_name) :
		name_(name),
		type_name_(type_name)
	{}

	inline ChunkArrayGen()
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ChunkArrayGen);

protected:

	std::vector<ChunkArrayGen**> external_refs_;

	std::string name_;

	std::string type_name_;

public:

	/**
	 * @brief virtual destructor
	 */
	virtual ~ChunkArrayGen()
	{
		invalidate_external_refs();
	}

protected:

	void invalidate_external_refs()
	{
		for (auto ref : external_refs_)
		{
			*ref = nullptr;
		}
	}

public:

	void add_external_ref(ChunkArrayGen** ref)
	{
		cgogn_message_assert(*ref == this, "ChunkArrayGen add_external_ref on other ChunkArrayGen");
		external_refs_.push_back(ref);
	}

	void remove_external_ref(ChunkArrayGen** ref)
	{
		cgogn_message_assert(*ref == this, "ChunkArrayGen remove_external_ref on other ChunkArrayGen");
		auto it = std::find(external_refs_.begin(), external_refs_.end(), ref);
		cgogn_message_assert(it != external_refs_.end(), "ChunkArrayGen external ref not found");
		std::swap(*it, external_refs_.back());
		external_refs_.pop_back();
	}

	inline const std::string& name() const { return name_; }

	inline const std::string& type_name() const { return type_name_; }

	virtual std::string nested_type_name() const = 0;

	virtual uint32 nb_components() const = 0;

	/**
	 * @brief element_size
	 * @return the size of an element of the ChunkArray
	 * Warning : shall not be used with ChunkArrayBool.
	 */
	virtual uint32 element_size() const = 0;

	/**
	 * @brief get the number of chunks of the array
	 * @return the number of chunks
	 */
	virtual uint32 nb_chunks() const = 0;

	/**
	 * @brief get the capacity of the array
	 * @return number of allocated lines
	 */
	virtual uint32 capacity() const = 0;

	/**
	 * @brief return a vector with pointers to all chunks
	 * @param byte_block_size filled with CHUNK_SIZE*sizeof(T)
	 * @return the vector of pointers
	 */
	virtual std::vector<const void*> chunks_pointers(uint32& byte_block_size) const = 0;

	/**
	 * @brief create a ChunkArray object without knowing type
	 * @return generic pointer
	 */
	virtual std::unique_ptr<Self> clone(const std::string& clone_name) const = 0;

	virtual bool swap_data(Self*) = 0;

	/**
	 * @brief add a chunk (T[CHUNK_SIZE])
	 */
	virtual void add_chunk() = 0;

	/**
	 * @brief set number of chunks
	 * @param nbc number of chunks
	 */
	virtual void set_nb_chunks(uint32 nbb) = 0;

	/**
	 * @brief clear the array
	 */
	virtual void clear() = 0;


	/**
	 * @brief copy an element to another one
	 * @param dst destination element index
	 * @param src source element index
	 */
	virtual void copy_element(uint32 dst, uint32 src) = 0;

	/**
	 * @brief copy an element (of another C.A.) to another one
	 * @param dst destination index
	 * @param cag_src chunk_array source ptr (Precond: same type as this)
	 * @param src source index
	 */
	virtual void copy_external_element(uint32 dst, Self* cag_src, uint32 src) = 0;

	/**
	 * @brief move an element to another one
	 * @param dst destination index
	 * @param src source index
	 */
	virtual void move_element(uint32 dst, uint32 src)
	{
		this->copy_element(dst,src);
	}

	/**
	 * @brief swap two elements
	 * @param idx1 first element index
	 * @param idx2 second element index
	 */
	virtual void swap_elements(uint32 idx1, uint32 idx2) = 0;

	/**
	 * @brief save
	 * @param fs file stream
	 * @param nb_lines number of line to save
	 */
	virtual void save(std::ostream& fs, uint32 nb_lines) const = 0;

	/**
	 * @brief load
	 * @param fs file stream
	 * @return ok
	 */
	virtual bool load(std::istream& fs) = 0;

	virtual void export_element(uint32 idx, std::ostream& o, bool binary, bool little_endian, std::size_t precision = 8ul) const = 0;
	/**
	 * @brief import_element, read the element "idx" from an ascii istream
	 * @param idx
	 * @param i
	 */
	virtual void import_element(uint32 idx, std::istream& in) = 0;
	/**
	 * @brief element_ptr
	 * @return a generic pointer to the element of index idx.
	 * Use with caution. This method can't be used with ChunkArrayBool.
	 */
	virtual const void* element_ptr(uint32 idx) const = 0;

	/**
	 * @brief skip the data instead of loading
	 * @param fs input file stream
	 */
	static void skip(std::istream& fs)
	{
		std::size_t chunk_bytes;
		serialization::load(fs, &chunk_bytes, 1);
		uint32 nb_lines;
		serialization::load(fs, &nb_lines, 1);
		fs.ignore(std::streamsize(chunk_bytes), EOF);
	}

	/**
	 * @brief copy the chunk array source into this, allocation is done
	 * @param cag_src
	 */
	virtual void copy(const Self& cag_src) = 0;

	/**
	 * @brief copy the data of chunk array source into this. The allocation must have been done before.
	 * @param cag_src
	 */
	virtual void copy_data(const Self& cag_src) = 0;

};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CONTAINER_CHUNK_ARRAY_GEN_CPP_))
extern template class CGOGN_CORE_API ChunkArrayGen<CGOGN_CHUNK_SIZE>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CONTAINER_CHUNK_ARRAY_GEN_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CONTAINER_CHUNK_ARRAY_GEN_H_

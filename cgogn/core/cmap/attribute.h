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

#ifndef CGOGN_CORE_MAP_ATTRIBUTE_H_
#define CGOGN_CORE_MAP_ATTRIBUTE_H_

#include <cgogn/core/basic/cell.h>
#include <cgogn/core/utils/assert.h>
#include <cgogn/core/utils/serialization.h>
#include <cgogn/core/utils/logger.h>
#include <cgogn/core/container/chunk_array_container.h>
#include <cgogn/core/cmap/map_base_data.h>
#include <cgogn/core/cmap/map_traits.h>

namespace cgogn
{

/**
 * \brief Generic Attribute class
 * @TPARAM DATA_TRAITS storage traits (for MapBaseData ptr type)
 */
class AttributeGen
{
public:

	using Self = AttributeGen;

	using ChunkArrayGen = MapBaseData::ChunkArrayGen;
	using ChunkArrayContainer = MapBaseData::ChunkArrayContainer<uint32>;
	template <typename T>
	using ChunkArray = MapBaseData::ChunkArray<T>;

	static const uint32 CHUNK_SIZE = MapBaseData::CHUNK_SIZE;

protected:

	MapBaseData* map_;

public:

	inline AttributeGen(MapBaseData* const map) :
		map_(map)
	{}

	/**
	 * \brief copy constructor
	 * @param atthg
	 */
	inline AttributeGen(const Self& atthg) :
		map_(atthg.map_)
	{}

	/**
	 * \brief move constructor
	 * @param atthg
	 */
	inline AttributeGen(Self&& atthg) CGOGN_NOEXCEPT :
		map_(atthg.map_)
	{
		atthg.map_ = nullptr;
	}

	/**
	 * \brief operator =
	 * @param atthg
	 * @return
	 */
	inline AttributeGen& operator=(const Self& atthg)
	{
		this->map_ = atthg.map_;
		return *this;
	}

	/**
	 * \brief move operator =
	 * @param atthg
	 * @return
	 */
	inline AttributeGen& operator=(Self&& atthg) CGOGN_NOEXCEPT
	{
		this->map_ = atthg.map_;
		return *this;
	}

	virtual ~AttributeGen()
	{}

	inline bool is_linked_to(MapBaseData* m) const
	{
		return m == map_;
	}

	virtual const std::string&	name() const = 0;
	virtual const std::string&	type_name() const = 0;
	virtual bool				is_valid() const = 0;
};

/**
 * @brief The Attribute_T class
 * @TPARAM T the data type of the attribute to handlde
 * In this class we do not know the orbit of the Attribute.
 */
template <typename T>
class Attribute_T : public AttributeGen
{
public:

	using Inherit = AttributeGen;
	using Self = Attribute_T<T>;
	using value_type = T;

	using ChunkArrayGen = typename Inherit::ChunkArrayGen;
	using ChunkArrayContainer = typename Inherit::ChunkArrayContainer;
	using TChunkArray = typename Inherit::ChunkArray<T>;

	inline Attribute_T() :
		Inherit(nullptr),
		chunk_array_cont_(nullptr),
		chunk_array_(nullptr),
		orbit_(Orbit(NB_ORBITS))
	{}

	Attribute_T(MapBaseData* const map, TChunkArray* const ca, Orbit orbit) :
		Inherit(map),
		chunk_array_(ca),
		orbit_(orbit)
	{
		if (map != nullptr)
			chunk_array_cont_ = &map->const_attribute_container(orbit);
		if (chunk_array_ != nullptr)
			chunk_array_->add_external_ref(reinterpret_cast<ChunkArrayGen**>(&chunk_array_));
	}

	Attribute_T(const Self& att) :
		Inherit(att),
		chunk_array_cont_(att.chunk_array_cont_),
		chunk_array_(att.chunk_array_),
		orbit_(att.orbit_)
	{
		if (chunk_array_ != nullptr)
			chunk_array_->add_external_ref(reinterpret_cast<ChunkArrayGen**>(&chunk_array_));
	}

	inline Attribute_T(Self&& att) CGOGN_NOEXCEPT :
		Inherit(std::move(att)),
		chunk_array_cont_(att.chunk_array_cont_),
		chunk_array_(att.chunk_array_),
		orbit_(att.orbit_)
	{
		if (chunk_array_ != nullptr)
			chunk_array_->add_external_ref(reinterpret_cast<ChunkArrayGen**>(&chunk_array_));
	}

	inline Attribute_T& operator=(const Self& att)
	{
		Inherit::operator=(att);

		if (is_valid())
			chunk_array_->remove_external_ref(reinterpret_cast<ChunkArrayGen**>(&chunk_array_));

		chunk_array_cont_ = att.chunk_array_cont_;
		chunk_array_ = att.chunk_array_;
		orbit_ = att.orbit_;

		if (chunk_array_ != nullptr)
			chunk_array_->add_external_ref(reinterpret_cast<ChunkArrayGen**>(&chunk_array_));

		return *this;
	}

	Attribute_T& operator=(Self&& att)
	{
		Inherit::operator=(std::move(att));

		if (is_valid())
			chunk_array_->remove_external_ref(reinterpret_cast<ChunkArrayGen**>(&chunk_array_));

		chunk_array_cont_ = att.chunk_array_cont_;
		chunk_array_ = att.chunk_array_;
		orbit_ = att.orbit_;

		if (chunk_array_ != nullptr)
			chunk_array_->add_external_ref(reinterpret_cast<ChunkArrayGen**>(&chunk_array_));

		return *this;
	}

	virtual ~Attribute_T() override
	{
		if (is_valid())
			chunk_array_->remove_external_ref(reinterpret_cast<ChunkArrayGen**>(&chunk_array_));
	}

	/**
	 * \brief getDataVector
	 * @return
	 */
	TChunkArray const* data() const
	{
		return chunk_array_;
	}

	/**
	 * \brief affect a value to all elements of container (even holes)
	 * @param val value to affect
	 */
	inline void set_all_values(const T& val)
	{
		chunk_array_->set_all_values(val);
	}

	/**
	 * \brief operator[]
	 * @param i
	 * @return
	 */
	inline T& operator[](uint32 i)
	{
		cgogn_message_assert(is_valid(), "Invalid Attribute");
		return chunk_array_->operator[](i);
	}

	/**
	 * \brief const operator[]
	 * @param i
	 * @return
	 */
	inline const T& operator[](uint32 i) const
	{
		cgogn_message_assert(is_valid(), "Invalid Attribute");
		return chunk_array_->operator[](i);
	}

	inline T& operator[](Dart d)
	{
		cgogn_message_assert(this->is_valid(), "Invalid Attribute");
		return this->chunk_array_->operator[](this->map_->embedding(d, orbit_));
	}

	inline const T& operator[](Dart d) const
	{
		cgogn_message_assert(this->is_valid(), "Invalid Attribute");
		return this->chunk_array_->operator[](this->map_->embedding(d, orbit_));
	}

	virtual const std::string& name() const override
	{
		return chunk_array_->name();
	}

	virtual const std::string& type_name() const override
	{
		return chunk_array_->type_name();
	}

	virtual bool is_valid() const override
	{
		return chunk_array_ != nullptr;
	}

	class const_iterator
	{
	public:

		const Self* const ah_ptr_;
		uint32 index_;

		inline const_iterator(const Self* ah, uint32 i) :
			ah_ptr_(ah),
			index_(i)
		{}

		inline const_iterator(const const_iterator& it) :
			ah_ptr_(it.ah_ptr_),
			index_(it.index_)
		{}

		inline const_iterator& operator=(const const_iterator& it)
		{
			ah_ptr_ = it.ah_ptr_;
			index_ = it.index_;
			return *this;
		}

		inline const_iterator& operator++()
		{
			ah_ptr_->chunk_array_cont_->next(index_);
			return *this;
		}

		inline const T& operator*() const
		{
			return ah_ptr_->operator[](index_);
		}

		inline bool operator!=(const_iterator it) const
		{
			cgogn_assert(ah_ptr_ == it.ah_ptr_);
			return index_ != it.index_;
		}
	};

	inline const_iterator begin() const
	{
		return const_iterator(this, this->chunk_array_cont_->begin());
	}

	inline const_iterator end() const
	{
		return const_iterator(this, this->chunk_array_cont_->end());
	}

	class iterator
	{
	public:

		Self* const ah_ptr_;
		uint32 index_;

		inline iterator(Self* ah, uint32 i) :
			ah_ptr_(ah),
			index_(i)
		{}

		inline iterator(const iterator& it) :
			ah_ptr_(it.ah_ptr_),
			index_(it.index_)
		{}

		inline iterator& operator=(const iterator& it)
		{
			ah_ptr_ = it.ah_ptr_;
			index_ = it.index_;
			return *this;
		}

		inline iterator& operator++()
		{
			ah_ptr_->chunk_array_cont_->next(index_);
			return *this;
		}

		inline T& operator*()
		{
			return ah_ptr_->operator[](index_);
		}

		inline bool operator!=(iterator it) const
		{
			cgogn_assert(ah_ptr_ == it.ah_ptr_);
			return index_ != it.index_;
		}
	};

	inline iterator begin()
	{
		return iterator(this, this->chunk_array_cont_->begin());
	}

	inline iterator end()
	{
		return iterator(this, this->chunk_array_cont_->end());
	}

protected:

	ChunkArrayContainer const*	chunk_array_cont_;
	TChunkArray*				chunk_array_;
	Orbit						orbit_;
};

/**
 * \brief Attribute class
 * @TPARAM T the data type of the attribute to handlde
 */
template <typename T, Orbit ORBIT>
class Attribute : public Attribute_T<T>
{
public:

	using Inherit = Attribute_T<T>;
	using Self = Attribute<T, ORBIT>;

	using TChunkArray = typename Inherit::TChunkArray;
	using Inherit::operator[];

	/**
	 * \brief Default constructor
	 *
	 * Construct a non-valid Attribute (i.e. not linked to any attribute)
	 */
	inline Attribute() :
		Inherit(nullptr, nullptr, Orbit())
	{}

	inline Attribute(MapBaseData* const map, TChunkArray* const ca) :
		Inherit(map, ca, ORBIT)
	{}

	/**
	 * \brief Copy constructor
	 * @param att
	 */
	inline Attribute(const Self& att) :
		Inherit(att)
	{}

	/**
	 * \brief Move constructor
	 * @param att
	 */
	inline Attribute(Self&& att) CGOGN_NOEXCEPT :
		Inherit(std::move(att))
	{}

	/**
	 * \brief operator =
	 * @param att
	 * @return
	 */
	inline Attribute& operator=(const Self& att)
	{
		Inherit::operator=(att);
		return *this;
	}

	/**
	 * \brief move operator =
	 * @param att
	 * @return
	 */
	Attribute& operator=(Self&& att)
	{
		Inherit::operator=(std::move(att));
		return *this;
	}

	virtual ~Attribute() override
	{}

	/**
	 * \brief operator[]
	 * @param c
	 * @return
	 */
	inline T& operator[](Cell<ORBIT> c)
	{
		cgogn_message_assert(this->is_valid(), "Invalid Attribute");
		return this->chunk_array_->operator[](this->map_->embedding(c));
	}

	/**
	 * \brief operator[]
	 * @param c
	 * @return
	 */
	inline const T& operator[](Cell<ORBIT> c) const
	{
		cgogn_message_assert(this->is_valid(), "Invalid Attribute");
		return this->chunk_array_->operator[](this->map_->embedding(c));
	}

	inline Orbit orbit() const
	{
		return ORBIT;
	}
};

} // namespace cgogn

#endif // CGOGN_CORE_MAP_ATTRIBUTE_H_

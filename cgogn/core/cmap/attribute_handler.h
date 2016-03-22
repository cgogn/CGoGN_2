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

#ifndef CORE_MAP_ATTRIBUTE_HANDLER_H_
#define CORE_MAP_ATTRIBUTE_HANDLER_H_

#include <core/basic/cell.h>
#include <core/utils/assert.h>
#include <core/cmap/map_base_data.h>
namespace cgogn
{

/**
 * \brief Generic AttributeHandler class
 * @TPARAM DATA_TRAITS storage traits (for MapBaseData ptr type)
 */
template <typename DATA_TRAITS>
class AttributeHandlerGen
{
public:

	using Self = AttributeHandlerGen<DATA_TRAITS>;
	using MapData = MapBaseData<DATA_TRAITS>;

protected:

	MapData* map_;

public:

	inline AttributeHandlerGen(MapData* const map) :
		map_(map)
	{}

	/**
	 * \brief copy constructor
	 * @param atthg
	 */
	inline AttributeHandlerGen(const Self& atthg) :
		map_(atthg.map_)
	{}

	/**
	 * \brief move constructor
	 * @param atthg
	 */
	inline AttributeHandlerGen(Self&& atthg) CGOGN_NOEXCEPT :
		map_(atthg.map_)
	{
		atthg.map_ = nullptr;
	}

	/**
	 * \brief operator =
	 * @param atthg
	 * @return
	 */
	inline AttributeHandlerGen& operator=(const Self& atthg)
	{
		this->map_ = atthg.map_;
		return *this;
	}

	/**
	 * \brief move operator =
	 * @param atthg
	 * @return
	 */
	inline AttributeHandlerGen& operator=(Self&& atthg)
	{
		this->map_ = atthg.map_;
		return *this;
	}

	virtual ~AttributeHandlerGen()
	{}

	virtual bool is_valid() const = 0;

	virtual Orbit get_orbit() const = 0;
};


/**
 * \brief Generic AttributeHandler class with orbit parameter
 * @TPARAM ORBIT the orbit of the attribute to handlde
 */
template <typename DATA_TRAITS, Orbit ORBIT>
class AttributeHandlerOrbit : public AttributeHandlerGen<DATA_TRAITS>
{
public:

	using Inherit = AttributeHandlerGen<DATA_TRAITS>;
	using Self = AttributeHandlerOrbit<DATA_TRAITS, ORBIT>;
	using MapData = typename Inherit::MapData;

	static const uint32 CHUNKSIZE = MapData::CHUNKSIZE;
	static const Orbit orbit_value = ORBIT;

	template <typename T>
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNKSIZE, T>;
	template <typename T>
	using ChunkArray = cgogn::ChunkArray<CHUNKSIZE, T>;

protected:

	ChunkArrayContainer<uint32>* chunk_array_cont_;

public:

	inline AttributeHandlerOrbit(MapData* const map) :
		Inherit(map),
		chunk_array_cont_(nullptr)
	{
		if (map != nullptr)
			chunk_array_cont_ = &(map->template get_attribute_container<ORBIT>());
	}

	/**
	 * \brief copy constructor
	 * @param attho
	 */
	inline AttributeHandlerOrbit(const Self& attho) :
		Inherit(attho),
		chunk_array_cont_(attho.chunk_array_cont_)
	{}

	/**
	 * \brief move constructor
	 * @param attho
	 */
	inline AttributeHandlerOrbit(Self&& attho) CGOGN_NOEXCEPT :
		Inherit(std::move(attho)),
		chunk_array_cont_(attho.chunk_array_cont_)
	{}

	/**
	 * \brief operator =
	 * @param attho
	 * @return
	 */
	inline AttributeHandlerOrbit& operator=(const Self& attho)
	{
		Inherit::operator=(attho);
		chunk_array_cont_ = attho.chunk_array_cont_;
		return *this;
	}
	/**
	 * \brief move operator =
	 * @param attho
	 * @return
	 */
	inline AttributeHandlerOrbit& operator=(Self&& attho)
	{
		Inherit::operator=(std::move(attho));
		chunk_array_cont_ = attho.chunk_array_cont_;
		return *this;
	}

	virtual Orbit get_orbit() const override
	{
		return ORBIT;
	}

	virtual ~AttributeHandlerOrbit() override
	{}
};

/**
 * \brief AttributeHandler class
 * @TPARAM T the data type of the attribute to handlde
 */
template <typename DATA_TRAITS, typename T, Orbit ORBIT>
class AttributeHandler : public AttributeHandlerOrbit<DATA_TRAITS, ORBIT>
{
public:

	using Inherit = AttributeHandlerOrbit<DATA_TRAITS, ORBIT>;
	using Self = AttributeHandler<DATA_TRAITS, T, ORBIT>;
	using value_type =  T;
	using MapData =     typename Inherit::MapData;
	using TChunkArray = typename Inherit::template ChunkArray<T>;

protected:

	TChunkArray* chunk_array_;

public:

	/**
	 * \brief Default constructor
	 *
	 * Construct a non-valid AttributeHandler (i.e. not linked to any attribute)
	 */
	AttributeHandler() :
		Inherit(nullptr),
		chunk_array_(nullptr)
	{}

	/**
	 * \brief Constructor
	 * @param m the map the attribute belongs to
	 * @param ca ChunkArray pointer
	 */
	AttributeHandler(MapData* const m, TChunkArray* const ca) :
		Inherit(m),
		chunk_array_(ca)
	{
		if (chunk_array_ != nullptr)
		{
			TChunkArray** tmp = &chunk_array_;
			typename TChunkArray::Inherit** ref = reinterpret_cast<typename TChunkArray::Inherit**>(tmp);
			chunk_array_->add_external_ref(ref);
		}
	}

	/**
	 * \brief Copy constructor
	 * @param att
	 */
	AttributeHandler(const Self& att) :
		Inherit(att),
		chunk_array_(att.chunk_array_)
	{
		if (chunk_array_ != nullptr)
		{
			TChunkArray** tmp = &chunk_array_;
			typename TChunkArray::Inherit** ref = reinterpret_cast<typename TChunkArray::Inherit**>(tmp);
			chunk_array_->add_external_ref(ref);
		}
	}

	/**
	 * \brief Move constructor
	 * @param att
	 */
	AttributeHandler(Self&& att) CGOGN_NOEXCEPT :
		Inherit(std::move(att)),
		chunk_array_(att.chunk_array_)
	{
		if (chunk_array_ != nullptr)
		{
			TChunkArray** tmp = &chunk_array_;
			typename TChunkArray::Inherit** ref = reinterpret_cast<typename TChunkArray::Inherit**>(tmp);
			chunk_array_->add_external_ref(ref);
		}
	}

	/**
	 * \brief operator =
	 * @param att
	 * @return
	 */
	AttributeHandler& operator=(const Self& att)
	{
		Inherit::operator=(att);

		if (is_valid())
		{
			TChunkArray** tmp = &chunk_array_;
			typename TChunkArray::Inherit** ref = reinterpret_cast<typename TChunkArray::Inherit**>(tmp);
			chunk_array_->remove_external_ref(ref);
		}

		chunk_array_ = att.chunk_array_;

		if (chunk_array_ != nullptr)
		{
			TChunkArray** tmp = &chunk_array_;
			typename TChunkArray::Inherit** ref = reinterpret_cast<typename TChunkArray::Inherit**>(tmp);
			chunk_array_->add_external_ref(ref);
		}

		return *this;
	}

	/**
	 * \brief move operator =
	 * @param att
	 * @return
	 */
	AttributeHandler& operator=(Self&& att)
	{
		Inherit::operator=(std::move(att));

		if (is_valid())
		{
			TChunkArray** tmp = &chunk_array_;
			typename TChunkArray::Inherit** ref = reinterpret_cast<typename TChunkArray::Inherit**>(tmp);
			chunk_array_->remove_external_ref(ref);
		}

		chunk_array_ = att.chunk_array_;

		if (chunk_array_ != nullptr)
		{
			TChunkArray** tmp = &chunk_array_;
			typename TChunkArray::Inherit** ref = reinterpret_cast<typename TChunkArray::Inherit**>(tmp);
			chunk_array_->add_external_ref(ref);
		}

		return *this;
	}

	virtual ~AttributeHandler() override
	{
		if (is_valid())
		{
			TChunkArray** tmp = &chunk_array_;
			typename TChunkArray::Inherit** ref = reinterpret_cast<typename TChunkArray::Inherit**>(tmp);
			chunk_array_->remove_external_ref(ref);
		}
	}

	virtual bool is_valid() const override
	{
		return chunk_array_ != nullptr;
	}

	/**
	 * \brief getDataVector
	 * @return
	 */
	TChunkArray const* get_data() const
	{
		return chunk_array_;
	}

	/**
	 * \brief affect a value to all elements of container (even holes)
	 * @param val value to affect
	 */
	inline void set_all_container_values(const T& val)
	{
		chunk_array_->set_all_values(val);
	}

	/**
	 * \brief operator []
	 * @param c
	 * @return
	 */
	inline T& operator[](Cell<ORBIT> c)
	{
		cgogn_message_assert(is_valid(), "Invalid AttributeHandler");
		return chunk_array_->operator[](this->map_->get_embedding(c));
	}

	/**
	 * \brief operator []
	 * @param c
	 * @return
	 */
	inline const T& operator[](Cell<ORBIT> c) const
	{
		cgogn_message_assert(is_valid(), "Invalid AttributeHandler");
		return chunk_array_->operator[](this->map_->get_embedding(c));
	}

	/**
	 * \brief operator []
	 * @param i
	 * @return
	 */
	inline T& operator[](uint32 i)
	{
		cgogn_message_assert(is_valid(), "Invalid AttributeHandler");
		return chunk_array_->operator[](i);
	}

	/**
	 * \brief const operator []
	 * @param i
	 * @return
	 */
	inline const T& operator[](uint32 i) const
	{
		cgogn_message_assert(is_valid(), "Invalid AttributeHandler");
		return chunk_array_->operator[](i);
	}


	class const_iterator
	{
	public:
		const AttributeHandler<DATA_TRAITS, T, ORBIT>* const ah_ptr_;
		uint32 index_;

		inline const_iterator(const AttributeHandler<DATA_TRAITS, T, ORBIT>* ah, uint32 i) :
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
		AttributeHandler<DATA_TRAITS, T, ORBIT>* const ah_ptr_;
		uint32 index_;

		inline iterator(AttributeHandler<DATA_TRAITS, T, ORBIT>* ah, uint32 i) :
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
};

} // namespace cgogn

#endif // CORE_MAP_ATTRIBUTE_HANDLER_H_

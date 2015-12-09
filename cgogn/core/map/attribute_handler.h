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

#include <core/map/map_base.h>
#include <core/basic/cell.h>
#include <utils/assert.h>

///TODO ajouter enregistrement dans la map de la carte.

namespace cgogn
{

/**
 * \brief Generic AttributeHandler class
 * @TPARAM DATA_TRAITS storage traits (for MapBaseData ptr type)
 */
template<typename DATA_TRAITS>
class AttributeHandlerGen
{
public:

	typedef AttributeHandlerGen<DATA_TRAITS> Self;

	typedef MapBaseData<DATA_TRAITS> MapData;

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
	{}

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

	virtual unsigned int get_orbit() const = 0;
};


/**
 * \brief Generic AttributeHandler class with orbit parameter
 * @TPARAM ORBIT the orbit of the attribute to handlde
 */
template<typename DATA_TRAITS, unsigned int ORBIT>
class AttributeHandlerOrbit : public AttributeHandlerGen<DATA_TRAITS>
{
public:

	typedef AttributeHandlerGen<DATA_TRAITS>          Inherit;
	typedef AttributeHandlerOrbit<DATA_TRAITS, ORBIT> Self;

	typedef typename Inherit::MapData                 MapData;

	static const unsigned int CHUNKSIZE = MapData::CHUNKSIZE;

	template<typename T>
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNKSIZE, T>;
	template<typename T>
	using ChunkArray = cgogn::ChunkArray<CHUNKSIZE, T>;

protected:

	ChunkArrayContainer<unsigned int>* chunk_array_cont_;

public:

	inline AttributeHandlerOrbit(MapData* const map) :
		Inherit(map),
		chunk_array_cont_(nullptr)
	{
		if (map != nullptr)
			chunk_array_cont_ = &(map->get_attribute_container(ORBIT));
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

	virtual unsigned int get_orbit() const override
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
template<typename DATA_TRAITS, typename T, unsigned int ORBIT>
class AttributeHandler : public AttributeHandlerOrbit<DATA_TRAITS, ORBIT>
{
public:

	typedef AttributeHandlerOrbit<DATA_TRAITS, ORBIT> Inherit;
	typedef AttributeHandler<DATA_TRAITS, T, ORBIT>   Self;

	typedef T value_type;

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
	{}

	/**
	 * \brief Move constructor
	 * @param att
	 */
	AttributeHandler(Self&& att) CGOGN_NOEXCEPT :
		Inherit(std::move(att)),
		chunk_array_(att.chunk_array_)
	{}

	/**
	 * \brief operator =
	 * @param att
	 * @return
	 */
	AttributeHandler& operator=(const Self& att)
	{
		Inherit::operator=(att);
		chunk_array_ = att.chunk_array_;
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
		chunk_array_ = att.chunk_array_;
		return *this;
	}

	virtual ~AttributeHandler() override
	{
		if (chunk_array_ != nullptr)
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
	inline T& operator[](unsigned int i)
	{
		cgogn_message_assert(is_valid(), "Invalid AttributeHandler");
		return chunk_array_->operator[](i);
	}

	/**
	 * \brief const operator []
	 * @param i
	 * @return
	 */
	inline const T& operator[](unsigned int i) const
	{
		cgogn_message_assert(is_valid(), "Invalid AttributeHandler");
		return chunk_array_->operator[](i);
	}


	class const_iterator
	{
	public:
		const AttributeHandler<DATA_TRAITS, T, ORBIT>* const ah_ptr_;
		unsigned int index_;

		inline const_iterator(const AttributeHandler<DATA_TRAITS, T, ORBIT>* ah, unsigned int i) :
			ah_ptr_(ah),
			index_(i)
		{}

//		inline const_iterator& operator=(const const_iterator& it)
//		{
//			ah_ptr_ = it.ah_ptr_;
//			index_ = it.index_;
//			return *this;
//		}

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
		unsigned int index_;

		inline iterator(AttributeHandler<DATA_TRAITS, T, ORBIT>* ah, unsigned int i) :
			ah_ptr_(ah),
			index_(i)
		{}

//		inline iterator& operator=(const iterator& it)
//		{
//			ah_ptr_ = it.ah_ptr_;
//			index_ = it.index_;
//			return *this;
//		}

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

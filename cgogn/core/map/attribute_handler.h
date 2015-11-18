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

	typedef MapBaseData<DATA_TRAITS> MapData;

protected:

	MapData* map_;

	// boolean that states the validity of the handler
	bool valid_;

public:

	inline AttributeHandlerGen(MapData* const map) :
		map_(map)
		,valid_(false)
	{}

	/**
	 * \brief copy constructor
	 * @param atthg
	 */
	inline AttributeHandlerGen(const AttributeHandlerGen<DATA_TRAITS>& atthg) :
		map_(atthg.map_)
		,valid_(atthg.valid_)
	{}

	/**
	 * \brief move constructor
	 * @param atthg
	 */
	inline AttributeHandlerGen(AttributeHandlerGen<DATA_TRAITS>&& atthg) CGOGN_NOEXCEPT :
		map_(atthg.map_)
		,valid_(atthg.valid_)
	{}

	/**
	 * \brief operator =
	 * @param atthg
	 * @return
	 */
	inline AttributeHandlerGen& operator=(const AttributeHandlerGen<DATA_TRAITS>& atthg)
	{
		this->map_ = atthg.map_;
		this->valid_ = atthg.valid_;
		return *this;
	}

	/**
	 * \brief move operator =
	 * @param atthg
	 * @return
	 */
	inline AttributeHandlerGen& operator=(AttributeHandlerGen<DATA_TRAITS>&& atthg)
	{
		this->map_ = atthg.map_;
		this->valid_ = atthg.valid_;
		return *this;
	}

	virtual ~AttributeHandlerGen()
	{}

	inline bool isValid() const { return valid_; }

	virtual unsigned int getOrbit() const = 0;

protected:

	inline void setInvalid() { valid_ = false ; }

	inline void setValid() { valid_ = true ; }
};


/**
 * \brief Generic AttributeHandler class with orbit parameter
 * @TPARAM ORBIT the orbit of the attribute to handlde
 */
template<typename DATA_TRAITS, unsigned int ORBIT>
class AttributeHandlerOrbit : public AttributeHandlerGen<DATA_TRAITS>
{
public:

	typedef AttributeHandlerGen<DATA_TRAITS> Inherit;
	typedef typename Inherit::MapData        MapData;

protected:

	ChunkArrayContainer<DATA_TRAITS::CHUNK_SIZE, unsigned int>* chunk_array_cont_;

public:

	inline AttributeHandlerOrbit(MapData* const map) :
		Inherit(map)
		,chunk_array_cont_(nullptr)
	{
		if (map != nullptr)
		{
			chunk_array_cont_ =  &(map->getAttributeContainer(ORBIT));
		}
	}

	/**
	 * \brief copy constructor
	 * @param attho
	 */
	inline AttributeHandlerOrbit(const AttributeHandlerOrbit< DATA_TRAITS, ORBIT >& attho) :
		Inherit(attho)
		,chunk_array_cont_(attho.chunk_array_cont_)
	{}

	/**
	 * \brief move constructor
	 * @param attho
	 */
	inline AttributeHandlerOrbit(AttributeHandlerOrbit< DATA_TRAITS, ORBIT >&& attho) CGOGN_NOEXCEPT :
		Inherit(std::move(attho))
		,chunk_array_cont_(attho.chunk_array_cont_)
	{}

	/**
	 * \brief operator =
	 * @param attho
	 * @return
	 */
	inline AttributeHandlerOrbit& operator=(const AttributeHandlerOrbit< DATA_TRAITS, ORBIT >& attho)
	{
		Inherit::operator =(attho);
		chunk_array_cont_ = attho.chunk_array_cont_;
		return *this;
	}
	/**
	 * \brief move operator =
	 * @param attho
	 * @return
	 */
	inline AttributeHandlerOrbit& operator=(AttributeHandlerOrbit< DATA_TRAITS, ORBIT >&& attho)
	{
		Inherit::operator =(std::move(attho));
		chunk_array_cont_ = attho.chunk_array_cont_;
		return *this;
	}

	virtual unsigned int getOrbit() const override { return ORBIT; }
	virtual ~AttributeHandlerOrbit() override {}
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
	typedef ChunkArray<DATA_TRAITS::CHUNK_SIZE, T>    TChunkArray;
	typedef typename Inherit::MapData                 MapData;

protected:

	TChunkArray* chunk_array_;

public:

	/**
	 * \brief Default constructor
	 *
	 * Construct a non-valid AttributeHandler (i.e. not linked to any attribute)
	 */
	AttributeHandler():
		Inherit(nullptr)
	{}

	/**
	 * \brief Constructor
	 * @param m the map which belong attribute
	 * @param attributeName name of attribute
	 */
	AttributeHandler(MapData* const m, const std::string& attributeName) :
		Inherit(m)
	{
		cgogn_assert(this->chunk_array_cont_ != nullptr);
		chunk_array_ = this->chunk_array_cont_->getAttribute(attributeName);
		if (chunk_array_ == nullptr)
		{
			this->setInvalid();
		} else {
			this->setValid();
		}
	}

	AttributeHandler(MapData* const m, TChunkArray* const ca) :
		Inherit(m)
		,chunk_array_(ca)
	{
		if (chunk_array_ == nullptr)
		{
			this->setInvalid();
		} else {
			this->setValid();
		}
	}

	/**
	 * \brief Copy constructor
	 * @param att
	 */
	AttributeHandler(const AttributeHandler<DATA_TRAITS, T, ORBIT>& att) :
		Inherit(att)
		,chunk_array_(att.chunk_array_)
	{}

	/**
	 * \brief Move constructor
	 * @param att
	 */
	AttributeHandler(AttributeHandler<DATA_TRAITS, T, ORBIT>&& att) CGOGN_NOEXCEPT :
		Inherit(std::move(att))
		,chunk_array_(att.chunk_array_)
	{}

	/**
	 * \brief operator =
	 * @param att
	 * @return
	 */
	AttributeHandler& operator=(const AttributeHandler<DATA_TRAITS, T, ORBIT>& att)
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
	AttributeHandler& operator=(AttributeHandler<DATA_TRAITS, T, ORBIT>&& att)
	{
		Inherit::operator=(std::move(att));
		chunk_array_ = att.chunk_array_;
		return *this;
	}

	virtual ~AttributeHandler() override
	{}

	/**
	 * \brief getDataVector
	 * @return
	 */
	TChunkArray const * getData() const
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
		cgogn_message_assert(this->valid_, "Invalid AttributeHandler") ;
		return chunk_array_->operator[]( this->map_->getEmbedding(c) ) ;
	}

	/**
	 * \brief operator []
	 * @param c
	 * @return
	 */
	inline const T& operator[](Cell<ORBIT> c) const
	{
		cgogn_message_assert(this->valid_, "Invalid AttributeHandler") ;
		return chunk_array_->operator[]( this->map_->getEmbedding(c) ) ;
	}

	/**
	 * \brief operator []
	 * @param i
	 * @return
	 */
	inline T& operator[](unsigned int i)
	{
		cgogn_message_assert(this->valid_, "Invalid AttributeHandler") ;
		return chunk_array_->operator[](i) ;
	}

	/**
	 * \brief const operator []
	 * @param i
	 * @return
	 */
	inline const T& operator[](unsigned int i) const
	{
		cgogn_message_assert(this->valid_, "Invalid AttributeHandler") ;
		return chunk_array_->operator[](i) ;
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
			ah_ptr_(ah), index_(i)
		{}

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

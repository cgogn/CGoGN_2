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

#ifndef __CORE_MAP_ATTRIBUTE_HANDLER_H__
#define __CORE_MAP_ATTRIBUTE_HANDLER_H__

#include "core/map/map_base.h"
#include "core/basic/cell.h"

///TODO ajouter enregistrement dans la map de la carte.

namespace cgogn
{

/**
 * @brief Generic AttributeHandler class
 * @TPARAM DATA_TRAITS storage traits (for MapBaseData ptr type)
 */
template<typename DATA_TRAITS>
class AttributeHandlerGen
{
protected:

	MapBaseData<DATA_TRAITS>* map_;

	// boolean that states the validity of the handler
	bool valid_;

public:

	AttributeHandlerGen(bool v) :
		map_(NULL), valid_(v)
	{}

	virtual ~AttributeHandlerGen()
	{}

	inline bool isValid() const { return valid_; }

	virtual unsigned int getOrbit() const = 0;

protected:

	inline void setInvalid() { valid_ = false ; }

	inline void setValid() { valid_ = true ; }
};


/**
 * @brief Generic AttributeHandler class with orbit parameter
 * @TPARAM ORBIT the orbit of the attribute to handlde
 */
template<typename DATA_TRAITS, unsigned int ORBIT>
class AttributeHandlerOrbit : public AttributeHandlerGen<DATA_TRAITS>
{
protected:

	ChunkArrayContainer<DATA_TRAITS::CHUNK_SIZE, unsigned int>* chunk_array_cont_;

public:

	AttributeHandlerOrbit(MapBaseData<DATA_TRAITS>* map) :
		AttributeHandlerGen<DATA_TRAITS>(map)
	{
		chunk_array_cont_ = &(map->getAttributeContainer(ORBIT));
	}

	unsigned int getOrbit() const { return ORBIT; }
};

/**
 * @brief AttributeHandler class
 * @TPARAM T the data type of the attribute to handlde
 */
template<typename DATA_TRAITS, typename T, unsigned int ORBIT>
class AttributeHandler : public AttributeHandlerOrbit<DATA_TRAITS, ORBIT>
{
protected:

	ChunkArray<DATA_TRAITS::CHUNK_SIZE, T>* chunk_array_;

public:

	/**
	 * @brief Default constructor
	 *
	 * Construct a non-valid AttributeHandler (i.e. not linked to any attribute)
	 */
	AttributeHandler():
		AttributeHandlerGen<DATA_TRAITS>(false)
	{}

	/**
	 * @brief Constructor
	 * @param m the map which belong attribute
	 * @param attributeName name of attribute
	 */
	AttributeHandler(MapBaseData<DATA_TRAITS>* m, const std::string& attributeName):
		AttributeHandlerOrbit<DATA_TRAITS, ORBIT>(m)
	{
		chunk_array_ = this->chunk_array_cont_->getAttribute(attributeName);
		if (chunk_array_ == NULL)
		{
			this->setInvalid();
		}
	}

	AttributeHandler(MapBaseData<DATA_TRAITS>* m, ChunkArray<DATA_TRAITS::CHUNK_SIZE, T>* ca):
		AttributeHandlerOrbit<DATA_TRAITS, ORBIT>(m),
		chunk_array_(ca)
	{
		if (chunk_array_ == NULL)
		{
			this->setInvalid();
		}
	}

	/**
	 * @brief Copy constructor
	 * @param att
	 */
	AttributeHandler(const AttributeHandler<DATA_TRAITS, T, ORBIT>& att):
		AttributeHandlerOrbit<DATA_TRAITS, ORBIT>(att.map_)/*,
		chunk_array_(att.chunk_array_)*/
	{
		this->chunk_array_= att.chunk_array_;
		this->valid_ = att.valid_;
	}

	/**
	 * @brief operator =
	 * @param att
	 * @return
	 */
	AttributeHandler<DATA_TRAITS,T,ORBIT>& operator=(const AttributeHandler<DATA_TRAITS, T, ORBIT>& att)
	{
		this->valid_ = att.valid_;
		this->map_ = att.map_;
		this->chunk_array_cont_ = att.chunk_array_cont_;
		this->chunck_array_ = att.chunck_array_;
	}

	/**
	 * @brief getDataVector
	 * @return
	 */
	ChunkArray<DATA_TRAITS::CHUNK_SIZE, T>* getData() const
	{
		return chunk_array_;
	}

	/**
	 * @brief operator []
	 * @param c
	 * @return
	 */
	T& operator[](Cell<ORBIT> c)
	{
		assert(this->valid || !"Invalid AttributeHandler") ;
		unsigned int i = this->map_->getEmbedding(c) ;
		return chunk_array_->operator[](i) ;
	}

	/**
	 * @brief operator []
	 * @param c
	 * @return
	 */
	const T& operator[](Cell<ORBIT> c) const
	{
		assert(this->valid || !"Invalid AttributeHandler") ;
		unsigned int i = this->map_->getEmbedding(c) ;
		return chunk_array_->operator[](i) ;
	}

	/**
	 * @brief operator []
	 * @param i
	 * @return
	 */
	T& operator[](unsigned int i)
	{
		assert(this->valid_ || !"Invalid AttributeHandler") ;
		return chunk_array_->operator[](i) ;
	}

	/**
	 * @brief const operator []
	 * @param i
	 * @return
	 */
	const T& operator[](unsigned int i) const
	{
		assert(this->valid_ || !"Invalid AttributeHandler") ;
		return chunk_array_->operator[](i) ;
	}


	class const_iterator
	{
	public:
		const AttributeHandler<DATA_TRAITS, T,ORBIT>* ah_ptr_;
		unsigned int index_;

		inline const_iterator(const AttributeHandler<DATA_TRAITS, T, ORBIT>* ah, unsigned int i) :
			ah_ptr_(ah), index_(i)
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
			assert(ah_ptr_ == it.ah_ptr_);
			return index_ != it.index_;
		}
	};

	inline const_iterator begin() const
	{
		return const_iterator(this,this->chunk_array_cont_->begin());
	}

	inline const_iterator end() const
	{
		return const_iterator(this,this->chunk_array_cont_->end());
	}


	class iterator
	{
	public:
		AttributeHandler<DATA_TRAITS, T,ORBIT>* ah_ptr_;
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
			assert(ah_ptr_ == it.ah_ptr_);
			return index_ != it.index_;
		}
	};

	inline iterator begin()
	{
		return iterator(this,this->chunk_array_cont_->begin());
	}

	inline iterator end()
	{
		return iterator(this,this->chunk_array_cont_->end());
	}
};

} // namespace cgogn

#endif // __CORE_MAP_ATTRIBUTE_HANDLER_H__

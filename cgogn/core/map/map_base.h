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

#ifndef CORE_MAP_MAP_BASE_H_
#define CORE_MAP_MAP_BASE_H_

#include <core/map/map_base_data.h>
#include <core/map/attribute_handler.h>

#include <sstream>

namespace cgogn
{

template<typename DATA_TRAITS, typename TOPO_TRAITS>
class MapBase : public MapBaseData<DATA_TRAITS>
{
public:
	typedef MapBaseData<DATA_TRAITS> Inherit;
	typedef MapBase<DATA_TRAITS, TOPO_TRAITS> Super;

	using typename Inherit::ChunkArrayGen;
	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	using AttributeHandlerGen = cgogn::AttributeHandlerGen<DATA_TRAITS>;
	template<typename T, unsigned int ORBIT>
	using AttributeHandler = cgogn::AttributeHandler<DATA_TRAITS, T, ORBIT>;

	MapBase() : Inherit()
	{}

	~MapBase()
	{}

	/*******************************************************************************
	 * Attributes management
	 *******************************************************************************/

	/**
	 * \brief add an attribute
	 * @param attribute_name the name of the attribute to create
	 * @return a handler to the created attribute
	 */
	template <typename T, unsigned int ORBIT>
	inline AttributeHandler<T, ORBIT> add_attribute(const std::string& attribute_name = "")
	{
		if (this->embeddings_[ORBIT] == nullptr)
		{
			std::ostringstream oss;
			oss << "EMB_" << orbit_name(ORBIT);
			ChunkArray<unsigned int>* idx = this->topology_.template add_attribute<unsigned int>(oss.str());
			this->embeddings_[ORBIT] = idx;
			for (unsigned int i = this->topology_.begin(); i != this->topology_.end(); this->topology_.next(i))
				(*idx)[i] = EMBNULL;
		}

		ChunkArray<T>* ca = this->attributes_[ORBIT].template add_attribute<T>(attribute_name);
		return AttributeHandler<T, ORBIT>(this, ca);
	}

	/**
	 * \brief remove an attribute
	 * @param ah a handler to the attribute to remove
	 * @return true if remove succeed else false
	 */
	template <typename T, unsigned int ORBIT>
	inline bool remove_attribute(AttributeHandler<T, ORBIT>& ah)
	{
		ChunkArray<T>* ca = ah.getData();

		if (this->attributes_[ORBIT].remove_attribute(ca))
		{
			typedef typename std::multimap<ChunkArrayGen*, AttributeHandlerGen*>::iterator IT;
			std::pair<IT, IT> bounds = attribute_handlers_.equal_range(ca);
			for(IT i = bounds.first; i != bounds.second; ++i)
				(*i).second->set_invalid();
			attribute_handlers_.erase(bounds.first, bounds.second);
			return true;
		}
		return false;
	}

	/**
	* \brief search an attribute for a given orbit
	* @param nameAttr attribute name
	* @return an AttributeHandler
	*/
	template <typename T, unsigned int ORBIT>
	inline AttributeHandler< T, ORBIT> get_attribute(const std::string& attribute_name)
	{
		ChunkArray<T>* ca = this->attributes_[ORBIT].template get_attribute<T>(attribute_name);
		return AttributeHandler<T, ORBIT>(this, ca);
	}

protected:

	std::multimap<ChunkArrayGen*, AttributeHandlerGen*> attribute_handlers_;

	/*******************************************************************************
	 * Basic traversals
	 *******************************************************************************/

//private:

//	/**
//	 * \brief Map begin
//	 * @return the first dart of the map
//	 */
//	inline Dart begin() const
//	{
//		return Dart(this->topology_.begin());
//	}

//	/**
//	 * \brief Map end
//	 * @return the dart after the last dart of the map
//	 */
//	inline Dart end() const
//	{
//		return Dart(this->topology_.end());
//	}

//	/**
//	 * \brief next dart in the map
//	 * @param d reference to the dart to be modified
//	 */
//	inline void next(Dart& d) const
//	{
//		this->topology_.next(d.index);
//	}

public:

	class iterator
	{
	public:
		MapBase* const map_;
		Dart dart_;

		inline iterator(MapBase* map, Dart d) :
			map_(map),
			dart_(d)
		{}

		inline iterator& operator++()
		{
			map_->topology_.next(dart_.index);
			return *this;
		}

		inline Dart& operator*()
		{
			return dart_;
		}

		inline bool operator!=(iterator it) const
		{
			cgogn_assert(map_ == it.map_);
			return dart_ != it.dart_;
		}
	};

	inline iterator begin()
	{
		return iterator(this, Dart(this->topology_.begin()));
	}

	inline iterator end()
	{
		return iterator(this, Dart(this->topology_.end()));
	}

	class const_iterator
	{
	public:
		const MapBase* const map_;
		Dart dart_;

		inline const_iterator(const MapBase* map, Dart d) :
			map_(map),
			dart_(d)
		{}

		inline const_iterator& operator++()
		{
			map_->topology_.next(dart_.index);
			return *this;
		}

		inline const Dart& operator*() const
		{
			return dart_;
		}

		inline bool operator!=(iterator it) const
		{
			cgogn_assert(map_ == it.map_);
			return dart_ != it.dart_;
		}
	};

	inline const_iterator begin() const
	{
		return const_iterator(this, Dart(this->topology_.begin()));
	}

	inline const_iterator end() const
	{
		return const_iterator(this, Dart(this->topology_.end()));
	}

	/**
	 * \brief apply a function on each dart of the map
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <typename FUNC>
	inline void foreach_dart(FUNC f)
	{
		for (Dart d : *this)
			f(d);
	}

	/**
	 * \brief apply a function on each dart of the map
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <typename FUNC>
	inline void foreach_dart(FUNC& f)
	{
		for (Dart d : *this)
			f(d);
	}
};

} // namespace cgogn

#endif // CORE_MAP_MAP_BASE_H_

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
#include <core/traversal/global.h>

namespace cgogn
{

template<typename DATA_TRAITS, typename TOPO_TRAITS>
class MapBase : public MapBaseData<DATA_TRAITS>
{
public:

	typedef MapBaseData<DATA_TRAITS> Inherit;
	typedef MapBase<DATA_TRAITS, TOPO_TRAITS> Self;

	using typename Inherit::ChunkArrayGen;
	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	using AttributeHandlerGen = cgogn::AttributeHandlerGen<DATA_TRAITS>;
	template<typename T, unsigned int ORBIT>
	using AttributeHandler = cgogn::AttributeHandler<DATA_TRAITS, T, ORBIT>;

protected:

	std::multimap<ChunkArrayGen*, AttributeHandlerGen*> attribute_handlers_;

public:

	MapBase() : Inherit()
	{}

	~MapBase()
	{}

	/*******************************************************************************
	 * Embedding management
	 *******************************************************************************/

	template <unsigned int ORBIT>
	inline unsigned int get_embedding(Cell<ORBIT> c) const
	{
		cgogn_message_assert(this->template is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		return (*this->embeddings_[ORBIT])[c.dart.index];
	}

	inline unsigned int get_embedding(Dart d, unsigned int orbit) const
	{
		cgogn_message_assert(this->template is_orbit_embedded(orbit), "Invalid parameter: orbit not embedded");

		return (*this->embeddings_[orbit])[d.index];
	}

	inline void init_embedding(Dart d, unsigned int orbit, unsigned int emb)
	{
		cgogn_message_assert(this->embeddings_[orbit] != nullptr, "Invalid parameter: orbit not embedded");

		this->attributes_[orbit].ref_line(emb);     // ref the new emb
		(*this->embeddings_[orbit])[d.index] = emb; // affect the embedding to the dart
	}

	template <unsigned int ORBIT>
	inline void set_embedding(Dart d, unsigned int emb)
	{
		cgogn_message_assert(this->template is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		unsigned int old = get_embedding<ORBIT>(d);

		if (old == emb)	return;

		this->attributes_[ORBIT].unref_line(old); // unref the old emb
		this->attributes_[ORBIT].ref_line(emb);   // ref the new emb

		(*this->embeddings_[ORBIT])[d.index] = emb; // affect the embedding to the dart
	}

	inline void set_embedding(Dart d, unsigned int orbit, unsigned int emb)
	{
		cgogn_message_assert(this->embeddings_[orbit] != nullptr, "Invalid parameter: orbit not embedded");

		unsigned int old = get_embedding(d, orbit);

		if (old == emb)	return;

		this->attributes_[orbit].unref_line(old); // unref the old emb
		this->attributes_[orbit].ref_line(emb);   // ref the new emb

		(*this->embeddings_[orbit])[d.index] = emb; // affect the embedding to the dart
	}

	/*******************************************************************************
	 * Attributes management
	 *******************************************************************************/

public:

	/**
	 * \brief add an attribute
	 * @param attribute_name the name of the attribute to create
	 * @return a handler to the created attribute
	 */
	template <typename T, unsigned int ORBIT>
	inline AttributeHandler<T, ORBIT> add_attribute(const std::string& attribute_name = "")
	{
		if (!this->template is_orbit_embedded<ORBIT>())
			this->create_embedding(ORBIT);
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

	/*******************************************************************************
	 * Basic traversals
	 *******************************************************************************/

	class const_iterator
	{
	public:

		const Self& map_;
		Dart dart_;

		inline const_iterator(const Self& map, Dart d) :
			map_(map),
			dart_(d)
		{}

		inline const_iterator& operator++()
		{
			map_.topology_.next(dart_.index);
			return *this;
		}

		inline const Dart& operator*() const
		{
			return dart_;
		}

		inline bool operator!=(const_iterator it) const
		{
			cgogn_assert(&map_ == &(it.map_));
			return dart_ != it.dart_;
		}
	};

	inline const_iterator begin() const
	{
		return const_iterator(*this, Dart(this->topology_.begin()));
	}

	inline const_iterator end() const
	{
		return const_iterator(*this, Dart(this->topology_.end()));
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

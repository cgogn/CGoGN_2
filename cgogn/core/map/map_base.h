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

#include <core/basic/cell.h>
#include <core/basic/dart_marker.h>
#include <core/basic/cell_marker.h>

namespace cgogn
{

enum TraversalStrategy
{
	AUTO = 0,
	FORCE_DART_MARKING,
	FORCE_CELL_MARKING,
	FORCE_TOPO_CACHE
};

template <typename MAP_TRAITS, typename MAP_TYPE>
class MapBase : public MapBaseData<MAP_TRAITS>
{
public:

	typedef MapBaseData<MAP_TRAITS> Inherit;
	typedef MapBase<MAP_TRAITS, MAP_TYPE> Self;

	template <typename MAP> friend class DartMarker_T;
	template <typename MAP, Orbit ORBIT> friend class CellMarker_T;

	using typename Inherit::ChunkArrayGen;
	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	using AttributeHandlerGen = cgogn::AttributeHandlerGen<MAP_TRAITS>;
	template<typename T, Orbit ORBIT>
	using AttributeHandler = cgogn::AttributeHandler<MAP_TRAITS, T, ORBIT>;

	using ConcreteMap = typename MAP_TYPE::TYPE;

	using DartMarker = cgogn::DartMarker<ConcreteMap>;
	using DartMarkerStore = cgogn::DartMarkerStore<ConcreteMap>;

	template<Orbit ORBIT>
	using CellMarker = cgogn::CellMarker<ConcreteMap, ORBIT>;

	MapBase() : Inherit()
	{}

	~MapBase()
	{}

	MapBase(Self const&) = delete;
	MapBase(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

	/**
	 * @brief clear : clear the topology (empty the dart attributes including embeddings) leaving the other attributes unmodified
	 */
	inline void clear()
	{
		this->topology_.clear_attributes();

		for (unsigned int i = 0u; i < NB_ORBITS; ++i)
			this->attributes_[i].clear_attributes();
	}

	/**
	 * @brief clear_and_remove_attributes : clear the topology and delete all the attributes.
	 */
	inline void clear_and_remove_attributes()
	{
		this->topology_.clear_attributes();

		for (auto& mark_att_topo : this->mark_attributes_topology_)
			mark_att_topo.clear();

		for (auto& att : this->attributes_)
			att.remove_attributes();

		for (std::size_t i = 0u; i < NB_ORBITS; ++i)
		{
			if (this->embeddings_[i] != nullptr)
			{
				this->topology_.remove_attribute(this->embeddings_[i]);
				this->embeddings_[i] = nullptr;
				this->global_topo_cache_[i] = nullptr;
			}

			for (auto& mark_attr : this->mark_attributes_[i])
				mark_attr.clear();
		}
	}

protected:

	inline ConcreteMap* to_concrete()
	{
		return static_cast<ConcreteMap*>(this);
	}

	inline const ConcreteMap* to_concrete() const
	{
		return static_cast<const ConcreteMap*>(this);
	}

	/*******************************************************************************
	 * Container elements management
	 *******************************************************************************/

	inline unsigned int add_topology_element()
	{
		unsigned int idx = this->topology_.template insert_lines<ConcreteMap::PRIM_SIZE>();
		this->topology_.init_markers_of_line(idx);
		return idx;
	}

	template <Orbit ORBIT>
	inline unsigned int add_attribute_element()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		unsigned int idx = this->attributes_[ORBIT].template insert_lines<1>();
		this->attributes_[ORBIT].init_markers_of_line(idx);
		return idx;
	}

public:

	/*******************************************************************************
	 * Attributes management
	 *******************************************************************************/

	/**
	 * \brief add an attribute
	 * @param attribute_name the name of the attribute to create
	 * @return a handler to the created attribute
	 */
	template <typename T, Orbit ORBIT>
	inline AttributeHandler<T, ORBIT> add_attribute(const std::string& attribute_name = "")
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		if (!this->template is_orbit_embedded<ORBIT>())
			create_embedding<ORBIT>();
		ChunkArray<T>* ca = this->attributes_[ORBIT].template add_attribute<T>(attribute_name);
		return AttributeHandler<T, ORBIT>(this, ca);
	}

	/**
	 * \brief remove an attribute
	 * @param ah a handler to the attribute to remove
	 * @return true if remove succeed else false
	 */
	template <typename T, Orbit ORBIT>
	inline bool remove_attribute(AttributeHandler<T, ORBIT>& ah)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		const ChunkArray<T>* ca = ah.get_data();
		return this->attributes_[ORBIT].remove_attribute(ca);
	}

	/**
	* \brief search an attribute for a given orbit
	* @param attribute_name attribute name
	* @return an AttributeHandler
	*/
	template <typename T, Orbit ORBIT>
	inline AttributeHandler<T, ORBIT> get_attribute(const std::string& attribute_name)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		ChunkArray<T>* ca = this->attributes_[ORBIT].template get_attribute<T>(attribute_name);
		return AttributeHandler<T, ORBIT>(this, ca);
	}

	/**
	* \brief search an attribute for a given orbit and change its type (if size is compatible). First template arg is asked type, second is real type.
	* @param attribute_name attribute name
	* @return an AttributeHandler
	*/
	template <typename T_ASK, typename T_ATT, Orbit ORBIT>
	inline AttributeHandler<T_ASK, ORBIT> get_attribute_force_type(const std::string& attribute_name)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		static_assert(sizeof(T_ASK) == sizeof(T_ATT), "Incompatible casting operation between attributes, sizes are differents");

		ChunkArray<T_ASK>* ca = reinterpret_cast<ChunkArray<T_ASK>*>(this->attributes_[ORBIT].template get_attribute<T_ATT>(attribute_name));
		return AttributeHandler<T_ASK, ORBIT>(this, ca);
	}

protected:

	/*******************************************************************************
	 * Marking attributes management
	 *******************************************************************************/

	/**
	* \brief get a mark attribute on the given ORBIT attribute container (from pool or created)
	* @return a mark attribute on the topology container
	*/
	template <Orbit ORBIT>
	inline ChunkArray<bool>* get_mark_attribute()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		unsigned int thread = this->get_current_thread_index();
		if (!this->mark_attributes_[ORBIT][thread].empty())
		{
			ChunkArray<bool>* ca = this->mark_attributes_[ORBIT][thread].back();
			this->mark_attributes_[ORBIT][thread].pop_back();
			return ca;
		}
		else
		{
			std::lock_guard<std::mutex> lock(this->mark_attributes_mutex_[ORBIT]);
			if (!this->template is_orbit_embedded<ORBIT>())
				create_embedding<ORBIT>();
			ChunkArray<bool>* ca = this->attributes_[ORBIT].add_marker_attribute();
			return ca;
		}
	}

	/**
	* \brief release a mark attribute on the given ORBIT attribute container
	* @param the mark attribute to release
	*/
	template <Orbit ORBIT>
	inline void release_mark_attribute(ChunkArray<bool>* ca)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(this->template is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		this->mark_attributes_[ORBIT][this->get_current_thread_index()].push_back(ca);
	}

	/*******************************************************************************
	 * Embedding management
	 *******************************************************************************/

	/**
	 * \brief initialize a new orbit embedding
	 */
	template <Orbit ORBIT>
	inline void create_embedding()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(!this->template is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit is already embedded");

		std::ostringstream oss;
		oss << "EMB_" << orbit_name(ORBIT);

		// create the topology attribute that stores the orbit indices
		ChunkArray<unsigned int>* ca = this->topology_.template add_attribute<unsigned int>(oss.str());
		this->embeddings_[ORBIT] = ca;

		// initialize the indices of the existing orbits
		ConcreteMap* cmap = to_concrete();
		foreach_cell<ORBIT, FORCE_DART_MARKING>([cmap] (Cell<ORBIT> c)
		{
			cmap->init_orbit_embedding(c, cmap->template add_attribute_element<ORBIT>());
		});
	}

public:

	/**
	 * \brief make sure that all given orbits are uniquely embedded (indexed)
	 */
	template <Orbit ORBIT>
	void unique_orbit_embedding()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(this->template is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		AttributeHandler<unsigned int, ORBIT> counter = add_attribute<unsigned int, ORBIT>("tmp_counter");
		for (unsigned int& i : counter) i = 0;

		ConcreteMap* cmap = to_concrete();
		foreach_cell<ORBIT, FORCE_DART_MARKING>([cmap, &counter] (Cell<ORBIT> c)
		{
			if (counter[c] > 0)
				cmap->set_orbit_embedding(c, cmap->template add_attribute_element<ORBIT>());
			counter[c]++;
		});

		remove_attribute(counter) ;
	}

	/*******************************************************************************
	 * Topo caches management
	 *******************************************************************************/

	template <Orbit ORBIT>
	bool is_topo_cache_enabled()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		return this->global_topo_cache_[ORBIT] != nullptr;
	}

	template <Orbit ORBIT>
	void enable_topo_cache()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(!is_topo_cache_enabled<ORBIT>(), "Trying to enable an enabled global topo cache");

		this->global_topo_cache_[ORBIT] = this->attributes_[ORBIT].template add_attribute<Dart>("global_topo_cache");;
		update_topo_cache<ORBIT>();
	}

	template <Orbit ORBIT>
	void update_topo_cache()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_topo_cache_enabled<ORBIT>(), "Trying to update a disabled global topo cache");

		foreach_cell<ORBIT, FORCE_CELL_MARKING>([this] (Cell<ORBIT> c)
		{
			(*this->global_topo_cache_[ORBIT])[this->get_embedding(c)] = c.dart;
		});
	}

	template <Orbit ORBIT>
	void disable_topo_cache()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(is_topo_cache_enabled<ORBIT>(), "Trying to disable a disabled global topo cache");

		this->topology_.remove_attribute(this->global_topo_cache_[ORBIT]);
		this->global_topo_cache_[ORBIT] = nullptr;
	}

	/*******************************************************************************
	 * Traversals
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

		inline const_iterator(const const_iterator& it) :
			map_(it.map_),
			dart_(it.dart_)
		{}

		inline const_iterator& operator=(const const_iterator& it)
		{
			map_ = it.map_;
			dart_ = it.dart_;
			return *this;
		}

		inline const_iterator& operator++()
		{
			map_.topology_.next(dart_.index);
			return *this;
		}

		inline const Dart& operator*() const
		{
			return dart_;
		}

		inline bool operator!=(const const_iterator& it) const
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
	inline void foreach_dart(const FUNC& f)
	{
		for (Dart d = Dart(this->topology_.begin()), end = Dart(this->topology_.end());
			 d != end;
			 this->topology_.next(d.index))
		{
			f(d);
		}
	}

	/**
	 * \brief apply a function on each dart of the map and stops when the function returns false
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <typename FUNC>
	inline void foreach_dart_until(const FUNC& f)
	{
		for (Dart d = Dart(this->topology_.begin()), end = Dart(this->topology_.end());
			 d != end;
			 this->topology_.next(d.index))
		{
			if(!f(d))
				break;
		}
	}

	/**
	 * \brief apply a function on each orbit of the map
	 * @tparam ORBIT orbit to traverse
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <Orbit ORBIT, TraversalStrategy STRATEGY = TraversalStrategy::AUTO, typename FUNC>
	inline void foreach_cell(const FUNC& f)
	{
		static_assert(check_func_parameter_type(FUNC, Cell<ORBIT>), "Wrong function cell parameter type");

		switch (STRATEGY)
		{
			case FORCE_DART_MARKING :
				foreach_cell_dart_marking<ORBIT>(f);
				break;
			case FORCE_CELL_MARKING :
				foreach_cell_cell_marking<ORBIT>(f);
				break;
			case FORCE_TOPO_CACHE :
				foreach_cell_topo_cache<ORBIT>(f);
				break;
			case AUTO :
				if (is_topo_cache_enabled<ORBIT>())
					foreach_cell_topo_cache<ORBIT>(f);
				else if (this->template is_orbit_embedded<ORBIT>())
					foreach_cell_cell_marking<ORBIT>(f);
				else
					foreach_cell_dart_marking<ORBIT>(f);
				break;
		}
	}


	/**
	 * \brief apply a function on each orbit of the map and stops when the function returns false
	 * @tparam ORBIT orbit to traverse
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <Orbit ORBIT, TraversalStrategy STRATEGY = TraversalStrategy::AUTO, typename FUNC>
	void foreach_cell_until(const FUNC& f)
	{
		static_assert(check_func_parameter_type(FUNC, Cell<ORBIT>), "Wrong function cell parameter type");
		static_assert(check_func_return_type(FUNC, bool), "Wrong function return type");

		switch (STRATEGY)
		{
			case FORCE_DART_MARKING :
				foreach_cell_until_dart_marking<ORBIT>(f);
				break;
			case FORCE_CELL_MARKING :
				foreach_cell_until_cell_marking<ORBIT>(f);
				break;
			case FORCE_TOPO_CACHE :
				foreach_cell_topo_cache<ORBIT>(f);
				break;
			case AUTO :
				if (is_topo_cache_enabled<ORBIT>())
					foreach_cell_topo_cache<ORBIT>(f);
				else if (this->template is_orbit_embedded<ORBIT>())
					foreach_cell_until_cell_marking<ORBIT>(f);
				else
					foreach_cell_until_dart_marking<ORBIT>(f);
				break;
		}
	}

protected:

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_cell_dart_marking(const FUNC& f)
	{
		DartMarker dm(*to_concrete());
		for (Dart d = Dart(this->topology_.begin()), end = Dart(this->topology_.end());
			 d != end;
			 this->topology_.next(d.index))
		{
			if (!dm.is_marked(d))
			{
				dm.template mark_orbit<ORBIT>(d);
				f(d);
			}
		}
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_cell_cell_marking(const FUNC& f)
	{
		CellMarker<ORBIT> cm(*to_concrete());
		for (Dart d = Dart(this->topology_.begin()), end = Dart(this->topology_.end());
			 d != end;
			 this->topology_.next(d.index))
		{
			if (!cm.is_marked(d))
			{
				cm.mark(d);
				f(d);
			}
		}
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_cell_topo_cache(const FUNC& f)
	{
		for (unsigned int i = this->attributes_[ORBIT].begin(), end = this->attributes_[ORBIT].end();
			 i != end;
			 this->attributes_[ORBIT].next(i))
		{
			f((*this->global_topo_cache_[ORBIT])[i]);
		}
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_cell_until_dart_marking(const FUNC& f)
	{
		DartMarker dm(*to_concrete());
		for (Dart d = Dart(this->topology_.begin()), end = Dart(this->topology_.end());
			 d != end;
			 this->topology_.next(d.index))
		{
			if (!dm.is_marked(d))
			{
				dm.template mark_orbit<ORBIT>(d);
				if(!f(d))
					break;
			}
		}
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_cell_until_cell_marking(const FUNC& f)
	{
		CellMarker<ORBIT> cm(*to_concrete());
		for (Dart d = Dart(this->topology_.begin()), end = Dart(this->topology_.end());
			 d != end;
			 this->topology_.next(d.index))
		{
			if (!cm.is_marked(d))
			{
				cm.mark(d);
				if(!f(d))
					break;
			}
		}
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_cell_until_topo_cache(const FUNC& f)
	{
		for (unsigned int i = this->attributes_[ORBIT].begin(), end = this->attributes_[ORBIT].end();
			 i != end;
			 this->attributes_[ORBIT].next(i))
		{
			if(!f((*this->global_topo_cache_[ORBIT])[i]))
				break;
		}
	}
};

} // namespace cgogn

#endif // CORE_MAP_MAP_BASE_H_

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

template <typename DATA_TRAITS, typename TOPO_TRAITS>
class MapBase : public MapBaseData<DATA_TRAITS>
{
public:

	typedef MapBaseData<DATA_TRAITS> Inherit;
	typedef MapBase<DATA_TRAITS, TOPO_TRAITS> Self;

	template <typename MAP> friend class cgogn::DartMarkerT;
	template <typename MAP, Orbit ORBIT> friend class cgogn::CellMarkerT;

	using typename Inherit::ChunkArrayGen;
	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	using AttributeHandlerGen = cgogn::AttributeHandlerGen<DATA_TRAITS>;
	template<typename T, Orbit ORBIT>
	using AttributeHandler = cgogn::AttributeHandler<DATA_TRAITS, T, ORBIT>;

	using ConcreteMap = typename TOPO_TRAITS::CONCRETE;

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

	void clear(bool remove_attributes = false)
	{
		this->topology_.clear(false);

		for (unsigned int i = 0; i < Orbit::NB_ORBITS; ++i)
			this->attributes_[i].clear(remove_attributes);

		if (remove_attributes)
		{
			for (unsigned int i = 0; i < Orbit::NB_ORBITS; ++i)
			{
				if (this->embeddings_[i] != nullptr)
				{
					this->topology_.remove_attribute(this->embeddings_[i]);
					this->embeddings_[i] = nullptr;
				}

				for (unsigned int j = 0; j < NB_THREADS; ++j)
					this->mark_attributes_[i][j].clear();
			}
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
		unsigned int idx = this->topology_.template insert_lines<TOPO_TRAITS::PRIM_SIZE>();
		this->topology_.init_markers_of_line(idx);
		return idx;
	}

public:

	template <Orbit ORBIT>
	inline unsigned int add_attribute_element()
	{
		static_assert(ORBIT < Orbit::NB_ORBITS, "Unknown orbit parameter");

		unsigned int idx = this->attributes_[ORBIT].template insert_lines<1>();
		this->attributes_[ORBIT].init_markers_of_line(idx);
		return idx;
	}

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
		static_assert(ORBIT < Orbit::NB_ORBITS, "Unknown orbit parameter");

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
		static_assert(ORBIT < Orbit::NB_ORBITS, "Unknown orbit parameter");

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
		static_assert(ORBIT < Orbit::NB_ORBITS, "Unknown orbit parameter");

		ChunkArray<T>* ca = this->attributes_[ORBIT].template get_attribute<T>(attribute_name);
		return AttributeHandler<T, ORBIT>(this, ca);
	}

	/*******************************************************************************
	 * Marking attributes management
	 *******************************************************************************/

protected:

	/**
	* \brief get a mark attribute on the topology container (from pool or created)
	* @return a mark attribute on the topology container
	*/
	inline ChunkArray<bool>* get_topology_mark_attribute()
	{
		unsigned int thread = this->get_current_thread_index();
		if (!this->mark_attributes_topology_[thread].empty())
		{
			ChunkArray<bool>* ca = this->mark_attributes_topology_[thread].back();
			this->mark_attributes_topology_[thread].pop_back();
			return ca;
		}
		else
		{
			std::lock_guard<std::mutex> lock(this->mark_attributes_topology_mutex_);
			ChunkArray<bool>* ca = this->topology_.add_marker_attribute();
			return ca;
		}
	}

	/**
	* \brief release a mark attribute on the topology container
	* @param the mark attribute to release
	*/
	inline void release_topology_mark_attribute(ChunkArray<bool>* ca)
	{
		unsigned int thread = this->get_current_thread_index();
		this->mark_attributes_topology_[thread].push_back(ca);
	}

	/**
	* \brief get a mark attribute on the given ORBIT attribute container (from pool or created)
	* @return a mark attribute on the topology container
	*/
	template <Orbit ORBIT>
	inline ChunkArray<bool>* get_mark_attribute()
	{
		static_assert(ORBIT < Orbit::NB_ORBITS, "Unknown orbit parameter");

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
		static_assert(ORBIT < Orbit::NB_ORBITS, "Unknown orbit parameter");

		cgogn_message_assert(this->template is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		this->mark_attributes_[ORBIT][this->get_current_thread_index()].push_back(ca);
	}

	template <Orbit ORBIT>
	inline void create_embedding()
	{
		static_assert(ORBIT < Orbit::NB_ORBITS, "Unknown orbit parameter");

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

	/*******************************************************************************
	 * Traversals
	 *******************************************************************************/

public:

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
		switch (STRATEGY)
		{
			case FORCE_DART_MARKING :
				foreach_cell_dart_marking<ORBIT>(f);
				break;
			case FORCE_CELL_MARKING :
				foreach_cell_cell_marking<ORBIT>(f);
				break;
			case FORCE_TOPO_CACHE :
				cgogn_assert_not_reached("FORCE_TOPO_CACHE not implemented yet");
				break;
			case AUTO :
				if (this->template is_orbit_embedded<ORBIT>())
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
		switch (STRATEGY)
		{
			case FORCE_DART_MARKING :
				foreach_cell_until_dart_marking<ORBIT>(f);
				break;
			case FORCE_CELL_MARKING :
				foreach_cell_until_cell_marking<ORBIT>(f);
				break;
			case FORCE_TOPO_CACHE :
				cgogn_assert_not_reached("FORCE_TOPO_CACHE not implemented yet");
				break;
			case AUTO :
				if (this->template is_orbit_embedded<ORBIT>())
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
};

} // namespace cgogn

#endif // CORE_MAP_MAP_BASE_H_

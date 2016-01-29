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

#ifndef CORE_CMAP_MAP_BASE_H_
#define CORE_CMAP_MAP_BASE_H_

#include <vector>
#include <memory>

#include <core/cmap/map_base_data.h>
#include <core/cmap/attribute_handler.h>

#include <core/basic/cell.h>
#include <core/basic/dart_marker.h>
#include <core/basic/cell_marker.h>

#include <core/utils/thread_barrier.h>

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
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	using AttributeHandlerGen = cgogn::AttributeHandlerGen<MAP_TRAITS>;
	template <typename T, Orbit ORBIT>
	using AttributeHandler = cgogn::AttributeHandler<MAP_TRAITS, T, ORBIT>;

	using ConcreteMap = typename MAP_TYPE::TYPE;

	using DartMarker = cgogn::DartMarker<ConcreteMap>;
	using DartMarkerStore = cgogn::DartMarkerStore<ConcreteMap>;

	template <Orbit ORBIT>
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

	/**
	 * \brief Removes a topological element of PRIM_SIZE 
	 * from the topology container
	 * \details Removing a topological element consists in
	 * removing PRIM_SIZE lines of the topological container starting
	 * from index 
	 * 
	 * \param int [description]
	 */
	inline void remove_topology_element(unsigned int index)
	{
		this->topology_.template remove_lines<ConcreteMap::PRIM_SIZE>(index);
	}

	template <Orbit ORBIT>
	inline unsigned int add_attribute_element()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		unsigned int idx = this->attributes_[ORBIT].template insert_lines<1>();
		this->attributes_[ORBIT].init_markers_of_line(idx);
		return idx;
	}

	template <Orbit ORBIT>
	inline void remove_attribute_element(unsigned int index)
	{
		static_assert(ORBIT < NB_ORBITS,  "Unknown orbit parameter");

		this->attributes_[ORBIT].template remove_lines<1>(index);
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
	void enforce_unique_orbit_embedding()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(this->template is_orbit_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		AttributeHandler<unsigned int, ORBIT> counter = add_attribute<unsigned int, ORBIT>("__tmp_counter");
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

	template <Orbit ORBIT>
	bool is_well_embedded(Cell<ORBIT> c) const
	{
		const ConcreteMap* cmap = to_concrete();
		unsigned int index = this->get_embedding(c);
		bool result = true;

		std::map<unsigned int, Dart> emb_set;
		cmap->foreach_dart_of_orbit(c, [&] (Dart d)
		{
			emb_set.insert(std::pair<unsigned int, Dart>(this->template get_embedding<ORBIT>(d), d));
		});

		if(emb_set.size() > 1)
		{
			std::cout << "Orbit is not well embedded: " << std::endl;

			result = false;
			std::map<unsigned int, Dart>::iterator it;
			for (auto const& de : emb_set)
				std::cout << "\t dart #" << de.second << " has embed index #" << de.first << std::endl;
			std::cout << std::endl;
		}

		return result;
	}

	/*******************************************************************************
	 * Topo caches management
	 *******************************************************************************/

	template <Orbit ORBIT>
	bool is_topo_cache_enabled() const
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
	 * Topological information
	 *******************************************************************************/

	/**
	 * \brief return true if c1 and c2 represent the same cell, i.e. contain darts of the same orbit
	 * @tparam ORBIT considered orbit
	 * @param c1 first cell to compare
	 * @param c2 second cell to compare
	 */
	template <Orbit ORBIT>
	bool same_cell(Cell<ORBIT> c1, Cell<ORBIT> c2) const
	{
		const ConcreteMap* cmap = to_concrete();
		bool result = false;
		cmap->template foreach_dart_of_orbit_until<ORBIT>(c1, [&] (Dart d) -> bool
		{
			if (d == c2.dart)
			{
				result = true;
				return false;
			}
			return true;
		});
		return result;
	}

	/**
	 * \brief return the number of darts in the map
	 * @return
	 */
	unsigned int nb_darts() const
	{
		return this->topology_.size();
	}

	/**
	 * \brief return the number of cells of the given orbit in the map
	 */
	template <Orbit ORBIT>
	unsigned int nb_cells() const
	{
		if (this->template is_orbit_embedded<ORBIT>())
			return this->attributes_[ORBIT].size();
		else
		{
			unsigned int result = 0;
			foreach_cell<ORBIT>([&result] (Cell<ORBIT>) { ++result; });
			return result;
		}
	}

	/**
	 * \brief return the number of darts in the given cell
	 */
	template <Orbit ORBIT>
	unsigned int nb_darts(Cell<ORBIT> c) const
	{
		const ConcreteMap* cmap = to_concrete();
		unsigned int result = 0u;
		cmap->foreach_dart_of_orbit(c, [&result] (Dart) { ++result; });
		return result;
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
	inline void foreach_dart(const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Dart), "Wrong function parameter type");

		for (Dart d = Dart(this->topology_.begin()), end = Dart(this->topology_.end());
			 d != end;
			 this->topology_.next(d.index))
		{
			f(d);
		}
	}

	template <typename FUNC>
	inline void parallel_foreach_dart(const FUNC& f, unsigned int nb_threads = NB_THREADS - 1) const
	{
		static_assert(check_func_ith_parameter_type(FUNC, 0, Dart), "Wrong function first parameter type");
		static_assert(check_func_ith_parameter_type(FUNC, 1, unsigned int), "Wrong function second parameter type");

		std::vector<unsigned int> thread_indices(nb_threads);
		std::vector<std::thread::id*> thread_id_pointers(nb_threads);

		// get new thread local indices on this map
		// and get pointers to the corresponding thread_id locations
		// so that the threads will be able to store their id once created
		for (unsigned int i = 0; i < nb_threads; ++i)
		{
			unsigned int index = this->get_new_thread_index();
			thread_indices[i] = index;
			thread_id_pointers[i] = this->get_thread_id_pointer(index);
		}

		// these vectors will contain elements to be processed by the threads
		// the first ones are passed to the threads
		// the second ones are filled by this thread then swapped with the first ones
		std::vector<std::vector<Dart>> vd1(nb_threads);
		std::vector<std::vector<Dart>> vd2(nb_threads);
		for (unsigned int i = 0; i < nb_threads; ++i)
		{
			vd1[i].reserve(PARALLEL_BUFFER_SIZE);
			vd2[i].reserve(PARALLEL_BUFFER_SIZE);
		}

		bool finished = false;

		// creation of threads
		Barrier sync1(nb_threads + 1);
		Barrier sync2(nb_threads + 1);

		auto thread_deleter = [] (std::thread* th) { th->join(); delete th; };
		auto tfs_deleter = [this] (ThreadFunction<Dart, FUNC>* tf)
		{
			this->remove_thread(tf->get_thread_index());
			delete tf;
		};

		using thread_ptr = std::unique_ptr<std::thread, decltype(thread_deleter)>;
		using tfs_ptr = std::unique_ptr<ThreadFunction<Dart, FUNC>, decltype(tfs_deleter)>;

		std::vector<thread_ptr> threads;
		std::vector<tfs_ptr> tfs;
		threads.reserve(nb_threads);
		tfs.reserve(nb_threads);
		for (unsigned int i = 0; i < nb_threads; ++i)
		{
			tfs.emplace_back(tfs_ptr(new ThreadFunction<Dart, FUNC>(f, vd1[i], sync1, sync2, finished, i, thread_indices[i], thread_id_pointers[i]), tfs_deleter));
			threads.emplace_back(thread_ptr(new std::thread(std::ref( *(tfs[i]) )), thread_deleter));
		}

		Dart it = Dart(this->topology_.begin());
		Dart end = Dart(this->topology_.end());
		while (it != end)
		{
			for (unsigned int i = 0; i < nb_threads; ++i)
				vd2[i].clear();

			// fill vd2 vectors
			unsigned int nb = 0;
			while (it != end && nb < nb_threads * PARALLEL_BUFFER_SIZE)
			{
				vd2[nb % nb_threads].push_back(it);
				++nb;
				this->topology_.next(it.index);
			}

			for (unsigned int i = 0; i < nb_threads; ++i)
				vd1[i].swap(vd2[i]);

			sync2.wait(); // vectors are ready for threads to process
			sync1.wait(); // wait for all threads to finish their vector
		}

		finished = true; // say finish to all threads
		sync2.wait(); // last barrier wait
	}

	/**
	 * \brief apply a function on each dart of the map and stops when the function returns false
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <typename FUNC>
	inline void foreach_dart_until(const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Dart), "Wrong function parameter type");
		static_assert(check_func_return_type(FUNC, bool), "Wrong function return type");

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
	inline void foreach_cell(const FUNC& f) const
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

	template <Orbit ORBIT, TraversalStrategy STRATEGY = TraversalStrategy::AUTO, typename FUNC>
	inline void parallel_foreach_cell(const FUNC& f, unsigned int nb_threads = NB_THREADS - 1) const
	{
		static_assert(check_func_ith_parameter_type(FUNC, 0, Cell<ORBIT>), "Wrong function first parameter type");
		static_assert(check_func_ith_parameter_type(FUNC, 1, unsigned int), "Wrong function second parameter type");

		switch (STRATEGY)
		{
			case FORCE_DART_MARKING :
				parallel_foreach_cell_dart_marking<ORBIT>(f, nb_threads);
				break;
			case FORCE_CELL_MARKING :
				parallel_foreach_cell_cell_marking<ORBIT>(f, nb_threads);
				break;
			case FORCE_TOPO_CACHE :
				parallel_foreach_cell_topo_cache<ORBIT>(f, nb_threads);
				break;
			case AUTO :
				if (is_topo_cache_enabled<ORBIT>())
					parallel_foreach_cell_topo_cache<ORBIT>(f, nb_threads);
				else if (this->template is_orbit_embedded<ORBIT>())
					parallel_foreach_cell_cell_marking<ORBIT>(f, nb_threads);
				else
					parallel_foreach_cell_dart_marking<ORBIT>(f, nb_threads);
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
	void foreach_cell_until(const FUNC& f) const
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
				foreach_cell_until_topo_cache<ORBIT>(f);
				break;
			case AUTO :
				if (is_topo_cache_enabled<ORBIT>())
					foreach_cell_until_topo_cache<ORBIT>(f);
				else if (this->template is_orbit_embedded<ORBIT>())
					foreach_cell_until_cell_marking<ORBIT>(f);
				else
					foreach_cell_until_dart_marking<ORBIT>(f);
				break;
		}
	}

protected:

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_cell_dart_marking(const FUNC& f) const
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
	inline void parallel_foreach_cell_dart_marking(const FUNC& f, unsigned int nb_threads) const
	{
		std::vector<unsigned int> thread_indices(nb_threads);
		std::vector<std::thread::id*> thread_id_pointers(nb_threads);

		// get new thread local indices on this map
		// and get pointers to the corresponding thread_id locations
		// so that the threads will be able to store their id once created
		for (unsigned int i = 0; i < nb_threads; ++i)
		{
			unsigned int index = this->get_new_thread_index();
			thread_indices[i] = index;
			thread_id_pointers[i] = this->get_thread_id_pointer(index);
		}

		// these vectors will contain elements to be processed by the threads
		// the first ones are passed to the threads
		// the second ones are filled by this thread while the other are processed
		std::vector<std::vector<Cell<ORBIT>>> vd1(nb_threads);
		std::vector<std::vector<Cell<ORBIT>>> vd2(nb_threads);
		for (unsigned int i = 0; i < nb_threads; ++i)
		{
			vd1[i].reserve(PARALLEL_BUFFER_SIZE);
			vd2[i].reserve(PARALLEL_BUFFER_SIZE);
		}

		bool finished = false;

		// creation of threads
		Barrier sync1(nb_threads + 1);
		Barrier sync2(nb_threads + 1);

		auto thread_deleter = [] (std::thread* th) { th->join(); delete th; };
		auto tfs_deleter = [this] (ThreadFunction<Cell<ORBIT>, FUNC>* tf)
		{
			this->remove_thread(tf->get_thread_index());
			delete tf;
		};

		using thread_ptr = std::unique_ptr<std::thread, decltype(thread_deleter)>;
		using tfs_ptr = std::unique_ptr<ThreadFunction<Cell<ORBIT>, FUNC>, decltype(tfs_deleter)>;

		std::vector<thread_ptr> threads;
		std::vector<tfs_ptr> tfs;
		threads.reserve(nb_threads);
		tfs.reserve(nb_threads);
		for (unsigned int i = 0u; i < nb_threads; ++i)
		{
			tfs.emplace_back(tfs_ptr(new ThreadFunction<Cell<ORBIT>, FUNC>(f, vd1[i], sync1, sync2, finished, i, thread_indices[i], thread_id_pointers[i]), tfs_deleter));
			threads.emplace_back(thread_ptr(new std::thread(std::ref( *(tfs[i]) )), thread_deleter));
		}

		DartMarker dm(*to_concrete());
		Dart it = Dart(this->topology_.begin());
		Dart end = Dart(this->topology_.end());
		while (it != end)
		{
			for (unsigned int i = 0; i < nb_threads; ++i)
				vd2[i].clear();

			// fill vd2 vectors
			unsigned int nb = 0;
			while (it != end && nb < nb_threads * PARALLEL_BUFFER_SIZE)
			{
				if (!dm.is_marked(it))
				{
					dm.template mark_orbit<ORBIT>(it);
					vd2[nb % nb_threads].push_back(Cell<ORBIT>(it));
					++nb;
				}
				this->topology_.next(it.index);
			}

			for (unsigned int i = 0; i < nb_threads; ++i)
				vd1[i].swap(vd2[i]);

			sync2.wait(); // vectors are ready for threads to process
			sync1.wait(); // wait for all threads to finish their vector
		}

		finished = true; // say finish to all threads
		sync2.wait(); // last barrier wait
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_cell_cell_marking(const FUNC& f) const
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
	inline void parallel_foreach_cell_cell_marking(const FUNC& f, unsigned int nb_threads) const
	{
		std::vector<unsigned int> thread_indices(nb_threads);
		std::vector<std::thread::id*> thread_id_pointers(nb_threads);

		// get new thread local indices on this map
		// and get pointers to the corresponding thread_id locations
		// so that the threads will be able to store their id once created
		for (unsigned int i = 0; i < nb_threads; ++i)
		{
			unsigned int index = this->get_new_thread_index();
			thread_indices[i] = index;
			thread_id_pointers[i] = this->get_thread_id_pointer(index);
		}

		// these vectors will contain elements to be processed by the threads
		// the first ones are passed to the threads
		// the second ones are filled by this thread while the other are processed
		std::vector<std::vector<Cell<ORBIT>>> vd1(nb_threads);
		std::vector<std::vector<Cell<ORBIT>>> vd2(nb_threads);
		for (unsigned int i = 0; i < nb_threads; ++i)
		{
			vd1[i].reserve(PARALLEL_BUFFER_SIZE);
			vd2[i].reserve(PARALLEL_BUFFER_SIZE);
		}

		bool finished = false;

		// creation of threads
		Barrier sync1(nb_threads + 1);
		Barrier sync2(nb_threads + 1);

		auto thread_deleter = [] (std::thread* th) { th->join(); delete th; };
		auto tfs_deleter = [this] (ThreadFunction<Cell<ORBIT>, FUNC>* tf)
		{
			this->remove_thread(tf->get_thread_index());
			delete tf;
		};

		using thread_ptr = std::unique_ptr<std::thread, decltype(thread_deleter)>;
		using tfs_ptr = std::unique_ptr<ThreadFunction<Cell<ORBIT>, FUNC>, decltype(tfs_deleter)>;

		std::vector<thread_ptr> threads;
		std::vector<tfs_ptr> tfs;
		threads.reserve(nb_threads);
		tfs.reserve(nb_threads);
		for (unsigned int i = 0; i < nb_threads; ++i)
		{
			tfs.emplace_back(tfs_ptr(new ThreadFunction<Cell<ORBIT>, FUNC>(f, vd1[i], sync1, sync2, finished, i, thread_indices[i], thread_id_pointers[i]), tfs_deleter));
			threads.emplace_back(thread_ptr(new std::thread(std::ref( *(tfs[i]) )), thread_deleter));
		}

		CellMarker<ORBIT> cm(*to_concrete());
		Dart it = Dart(this->topology_.begin());
		Dart end = Dart(this->topology_.end());
		while (it != end)
		{
			for (unsigned int i = 0; i < nb_threads; ++i)
				vd2[i].clear();

			// fill vd2 vectors
			unsigned int nb = 0;
			while (it != end && nb < nb_threads * PARALLEL_BUFFER_SIZE)
			{
				if (!cm.is_marked(it))
				{
					cm.mark(it);
					vd2[nb % nb_threads].push_back(Cell<ORBIT>(it));
					++nb;
				}
				this->topology_.next(it.index);
			}

			for (unsigned int i = 0; i < nb_threads; ++i)
				vd1[i].swap(vd2[i]);

			sync2.wait(); // vectors are ready for threads to process
			sync1.wait(); // wait for all threads to finish their vector
		}

		finished = true; // say finish to all threads
		sync2.wait(); // last barrier wait
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_cell_topo_cache(const FUNC& f) const
	{
		for (unsigned int i = this->attributes_[ORBIT].begin(), end = this->attributes_[ORBIT].end();
			 i != end;
			 this->attributes_[ORBIT].next(i))
		{
			f((*this->global_topo_cache_[ORBIT])[i]);
		}
	}

	template <Orbit ORBIT, typename FUNC>
	inline void parallel_foreach_cell_topo_cache(const FUNC& f, unsigned int nb_threads) const
	{
		std::vector<unsigned int> thread_indices(nb_threads);
		std::vector<std::thread::id*> thread_id_pointers(nb_threads);

		// get new thread local indices on this map
		// and get pointers to the corresponding thread_id locations
		// so that the threads will be able to store their id once created
		for (unsigned int i = 0; i < nb_threads; ++i)
		{
			unsigned int index = this->get_new_thread_index();
			thread_indices[i] = index;
			thread_id_pointers[i] = this->get_thread_id_pointer(index);
		}

		// these vectors will contain elements to be processed by the threads
		// the first ones are passed to the threads
		// the second ones are filled by this thread while the other are processed
		std::vector<std::vector<Cell<ORBIT>>> vd1(nb_threads);
		std::vector<std::vector<Cell<ORBIT>>> vd2(nb_threads);
		for (unsigned int i = 0; i < nb_threads; ++i)
		{
			vd1[i].reserve(PARALLEL_BUFFER_SIZE);
			vd2[i].reserve(PARALLEL_BUFFER_SIZE);
		}

		bool finished = false;

		// creation of threads
		Barrier sync1(nb_threads + 1);
		Barrier sync2(nb_threads + 1);

		auto thread_deleter = [] (std::thread* th) { th->join(); delete th; };
		auto tfs_deleter = [this] (ThreadFunction<Cell<ORBIT>, FUNC>* tf)
		{
			this->remove_thread(tf->get_thread_index());
			delete tf;
		};

		using thread_ptr = std::unique_ptr<std::thread, decltype(thread_deleter)>;
		using tfs_ptr = std::unique_ptr<ThreadFunction<Cell<ORBIT>, FUNC>, decltype(tfs_deleter)>;

		std::vector<thread_ptr> threads;
		std::vector<tfs_ptr> tfs;
		threads.reserve(nb_threads);
		tfs.reserve(nb_threads);
		for (unsigned int i = 0; i < nb_threads; ++i)
		{
			tfs.emplace_back(tfs_ptr(new ThreadFunction<Cell<ORBIT>, FUNC>(f, vd1[i], sync1, sync2, finished, i, thread_indices[i], thread_id_pointers[i]), tfs_deleter));
			threads.emplace_back(thread_ptr(new std::thread(std::ref( *(tfs[i]) )), thread_deleter));
		}

		unsigned int it = this->attributes_[ORBIT].begin();
		unsigned int end = this->attributes_[ORBIT].end();
		while (it != end)
		{
			for (unsigned int i = 0; i < nb_threads; ++i)
				vd2[i].clear();

			// fill vd2 vectors
			unsigned int nb = 0;
			while (it != end && nb < nb_threads * PARALLEL_BUFFER_SIZE)
			{
				vd2[nb % nb_threads].push_back(Cell<ORBIT>((*this->global_topo_cache_[ORBIT])[it]));
				++nb;
				this->attributes_[ORBIT].next(it);
			}

			for (unsigned int i = 0; i < nb_threads; ++i)
				vd1[i].swap(vd2[i]);

			sync2.wait(); // vectors are ready for threads to process
			sync1.wait(); // wait for all threads to finish their vector
		}

		finished = true; // say finish to all threads
		sync2.wait(); // last barrier wait
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_cell_until_dart_marking(const FUNC& f) const
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
	inline void foreach_cell_until_cell_marking(const FUNC& f) const
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
	inline void foreach_cell_until_topo_cache(const FUNC& f) const
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

#endif // CORE_CMAP_MAP_BASE_H_

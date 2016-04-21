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

#ifndef CGOGN_CORE_CMAP_MAP_BASE_H_
#define CGOGN_CORE_CMAP_MAP_BASE_H_

#include <vector>
#include <memory>

#include <cgogn/core/cmap/map_base_data.h>
#include <cgogn/core/cmap/attribute_handler.h>

#include <cgogn/core/basic/cell.h>
#include <cgogn/core/basic/dart_marker.h>
#include <cgogn/core/basic/cell_marker.h>

#include <cgogn/core/utils/masks.h>
#include <cgogn/core/utils/logger.h>
#include <cgogn/core/utils/thread_barrier.h>
#include <cgogn/core/utils/unique_ptr.h>

namespace cgogn
{

enum TraversalStrategy
{
	AUTO = 0,
	FORCE_DART_MARKING,
	FORCE_CELL_MARKING
};

template <typename MAP_TRAITS, typename MAP_TYPE>
class MapBase : public MapBaseData<MAP_TRAITS>
{
public:

	using Inherit = MapBaseData<MAP_TRAITS>;
	using Self = MapBase<MAP_TRAITS, MAP_TYPE>;

	template <typename MAP> friend class DartMarker_T;
	template <typename MAP, Orbit ORBIT> friend class CellMarker_T;

	using typename Inherit::ChunkArrayGen;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	using AttributeGen = cgogn::AttributeGen<MAP_TRAITS>;
	template <typename T, Orbit ORBIT>
	using Attribute = cgogn::Attribute<MAP_TRAITS, T, ORBIT>;

	using ConcreteMap = typename MAP_TYPE::TYPE;

	using DartMarker = cgogn::DartMarker<ConcreteMap>;
	using DartMarkerStore = cgogn::DartMarkerStore<ConcreteMap>;

	template <Orbit ORBIT>
	using CellMarker = cgogn::CellMarker<ConcreteMap, ORBIT>;

	MapBase() :
		Inherit()
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MapBase);

	~MapBase()
	{}

	/**
	 * @brief clear : clear the topology (empty the dart attributes including embeddings) leaving the other attributes unmodified
	 */
	inline void clear()
	{
		this->topology_.clear_attributes();

		for (uint32 i = 0u; i < NB_ORBITS; ++i)
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

	inline Dart add_dart()
	{
		Dart d(add_topology_element());
		to_concrete()->init_dart(d);
		return d;
	}

	inline void remove_dart(Dart d)
	{
		this->remove_topology_element(d.index);
	}

	/*******************************************************************************
	 * Container elements management
	 *******************************************************************************/

	/**
	 * \brief Adds a topological element of PRIM_SIZE to the topology container
	 * \return the index of the added element
	 * Adding a topological element consists in adding PRIM_SIZE lines
	 * to the topological container starting from index
	 */
	inline uint32 add_topology_element()
	{
		const uint32 idx = this->topology_.template insert_lines<ConcreteMap::PRIM_SIZE>();
		this->topology_.init_markers_of_line(idx);
		for (uint32 orbit = 0u; orbit < NB_ORBITS; ++orbit)
		{
			if (this->embeddings_[orbit])
				(*this->embeddings_[orbit])[idx] = EMBNULL;
		}
		return idx;
	}

	/**
	 * \brief Removes a topological element of PRIM_SIZE from the topology container
	 * \param index the index of the element to remove
	 * Removing a topological element consists in removing PRIM_SIZE lines
	 * of the topological container starting from index
	 */
	inline void remove_topology_element(uint32 index)
	{
		this->topology_.template remove_lines<ConcreteMap::PRIM_SIZE>(index);

		for(uint32 orbit = 0; orbit < NB_ORBITS; ++orbit)
		{
			if(this->embeddings_[orbit])
			{
				uint32 emb = (*this->embeddings_[orbit])[index];
				if (emb != EMBNULL)
					this->attributes_[orbit].unref_line(emb);
			}
		}
	}

	template <Orbit ORBIT>
	inline uint32 add_attribute_element()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		uint32 idx = this->attributes_[ORBIT].template insert_lines<1>();
		this->attributes_[ORBIT].init_markers_of_line(idx);
		return idx;
	}

	template <Orbit ORBIT>
	inline void remove_attribute_element(uint32 index)
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
	inline Attribute<T, ORBIT> add_attribute(const std::string& attribute_name = "")
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		if (!this->template is_embedded<ORBIT>())
			create_embedding<ORBIT>();
		ChunkArray<T>* ca = this->attributes_[ORBIT].template add_attribute<T>(attribute_name);
		return Attribute<T, ORBIT>(this, ca);
	}

	/**
	 * \brief remove an attribute
	 * @param ah a handler to the attribute to remove
	 * @return true if remove succeed else false
	 */
	template <typename T, Orbit ORBIT>
	inline bool remove_attribute(Attribute<T, ORBIT>& ah)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		const ChunkArray<T>* ca = ah.get_data();
		return this->attributes_[ORBIT].remove_attribute(ca);
	}

	/**
	* \brief search an attribute for a given orbit
	* @param attribute_name attribute name
	* @return an Attribute
	*/
	template <typename T, Orbit ORBIT>
	inline Attribute<T, ORBIT> get_attribute(const std::string& attribute_name)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		ChunkArray<T>* ca = this->attributes_[ORBIT].template get_attribute<T>(attribute_name);
		return Attribute<T, ORBIT>(this, ca);
	}

	/**
	* \brief search an attribute for a given orbit and change its type (if size is compatible). First template arg is asked type, second is real type.
	* @param attribute_name attribute name
	* @return an Attribute
	*/
	template <typename T_ASK, typename T_ATT, Orbit ORBIT>
	inline Attribute<T_ASK, ORBIT> get_attribute_force_type(const std::string& attribute_name)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		static_assert(sizeof(T_ASK) == sizeof(T_ATT), "Incompatible casting operation between attributes, sizes are differents");

		ChunkArray<T_ASK>* ca = reinterpret_cast<ChunkArray<T_ASK>*>(this->attributes_[ORBIT].template get_attribute<T_ATT>(attribute_name));
		return Attribute<T_ASK, ORBIT>(this, ca);
	}


	template <typename T, Orbit ORBIT>
	inline void swap_attributes(Attribute<T, ORBIT>& ah1, Attribute<T, ORBIT>& ah2)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		cgogn_message_assert(ah1.is_linked_to(this), "swap_attributes: wrong map");
		cgogn_message_assert(ah2.is_linked_to(this), "swap_attributes: wrong map");

		this->attributes_[ORBIT].swap_data_attributes(ah1.get_data(), ah2.get_data());
	}

	template <typename T, Orbit ORBIT>
	inline void copy_attribute(Attribute<T, ORBIT>& dest, Attribute<T, ORBIT>& src)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		cgogn_message_assert(dest.is_linked_to(this), "copy_attribute: wrong map");
		cgogn_message_assert(src.is_linked_to(this), "copy_attribute: wrong map");

		this->attributes_[ORBIT].copy_data_attribute(dest.get_data(), src.get_data());
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

		std::size_t thread = this->get_current_thread_index();
		if (!this->mark_attributes_[ORBIT][thread].empty())
		{
			ChunkArray<bool>* ca = this->mark_attributes_[ORBIT][thread].back();
			this->mark_attributes_[ORBIT][thread].pop_back();
			return ca;
		}
		else
		{
			std::lock_guard<std::mutex> lock(this->mark_attributes_mutex_[ORBIT]);
			if (!this->template is_embedded<ORBIT>())
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
		cgogn_message_assert(this->template is_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

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
		cgogn_message_assert(!this->template is_embedded<ORBIT>(), "Invalid parameter: orbit is already embedded");

		std::ostringstream oss;
		oss << "EMB_" << orbit_name(ORBIT);

		// create the topology attribute that stores the orbit indices
		ChunkArray<uint32>* ca = this->topology_.template add_attribute<uint32>(oss.str());
		this->embeddings_[ORBIT] = ca;

		// initialize all darts indices to EMBNULL for this ORBIT
		foreach_dart([ca] (Dart d) { (*ca)[d.index] = EMBNULL; });

		// initialize the indices of the existing orbits
		foreach_cell<FORCE_DART_MARKING>([this] (Cell<ORBIT> c) { this->new_orbit_embedding(c); });

		cgogn_assert(this->template is_well_embedded<Cell<ORBIT>>());
	}

	template <Orbit ORBIT>
	inline uint32 new_orbit_embedding(Cell<ORBIT> c)
	{
		using CellType = Cell<ORBIT>;

		const uint32 emb = add_attribute_element<ORBIT>();
		to_concrete()->foreach_dart_of_orbit(c, [this, emb] (Dart d)
		{
			this->template set_embedding<CellType>(d, emb);
		});
		return emb;
	}

public:

	/**
	 * \brief make sure that all given orbits are uniquely embedded (indexed)
	 */
	template <Orbit ORBIT>
	void enforce_unique_orbit_embedding()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(this->template is_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		Attribute<uint32, ORBIT> counter = add_attribute<uint32, ORBIT>("__tmp_counter");
		for (uint32& i : counter) i = 0;

		foreach_cell<FORCE_DART_MARKING>([this, &counter] (Cell<ORBIT> c)
		{
			if (counter[c] > 0)
			{
				const uint32 old_emb = this->get_embedding(c);
				const uint32 new_emb = this->new_orbit_embedding(c);
				cgogn_log_warning("enforce_unique_orbit_embedding") << "Warning: enforce_unique_orbit_embedding: duplicating orbit #" << old_emb << " in orbit " << orbit_name(ORBIT);
				this->template get_attribute_container<ORBIT>().copy_line(new_emb, old_emb, false, false);
			}

			counter[c]++;
		});

		remove_attribute(counter);
	}

	/**
	 * \brief Tests if all \p ORBIT orbits are well embedded
	 * \details An orbit is well embedded if all its darts
	 * have the same embedding (index)
	 *
	 * \tparam ORBIT [description]
	 * \return [description]
	 */
	template <typename CellType>
	bool is_well_embedded()
	{
		static const Orbit ORBIT = CellType::ORBIT;
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(this->template is_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");

		const ConcreteMap* cmap = to_concrete();
		Attribute<std::vector<CellType>, ORBIT> counter = add_attribute<std::vector<CellType>, ORBIT>("__tmp_dart_per_emb");
		bool result = true;

		const typename Inherit::template ChunkArrayContainer<uint32>& container = this->attributes_[ORBIT];

		// Check that the indexation of cells is correct
		foreach_cell<FORCE_DART_MARKING>([&] (CellType c)
		{
			const uint32 idx = this->get_embedding(c);
			// check used indices are valid
			if (idx == EMBNULL)
			{
				result = false;
				cgogn_log_error("is_well_embedded") << "EMBNULL found for dart " << c << " in orbit " << orbit_name(ORBIT);
				return;
			}
			counter[idx].push_back(c);
			// check all darts of the cell use the same index (distinct to EMBNULL)
			cmap->foreach_dart_of_orbit(c, [&] (Dart d)
			{
				const uint32 emb_d = this->get_embedding(CellType(d));
				if (emb_d != idx)
					cgogn_log_error("is_well_embedded") << "Different indices (" << idx << " and " << emb_d << ") in orbit " << orbit_name(ORBIT);
			});
		});
		// check that all cells present in the attribute handler are used
		for (uint32 i = container.begin(), end = container.end(); i != end; container.next(i))
		{
			if (counter[i].empty())
			{
				result =false;
				cgogn_log_error("is_well_embedded") << "Cell #" << i << " is not used in orbit " << orbit_name(ORBIT);
			} else
			{
				const std::size_t size = counter[i].size();
				if (size >= 2ul)
				{
					result =false;
					cgogn_log_error("is_well_embedded") << size << " cells with same index \"" << i << "\" in orbit " << orbit_name(ORBIT);
				}
			}
		}

		remove_attribute(counter);
		return result;
	}

	bool check_map_integrity()
	{
		ConcreteMap* cmap = to_concrete();
		bool result = true;

		// check the integrity of topological relations or the correct sewing of darts
		foreach_dart_until([&cmap, &result] (Dart d)
		{
			result = cmap->check_integrity(d);
			return result;
		});
		if (!result)
		{
			cgogn_log_error("check_map_integrity") << "Integrity of the topology is broken";
			return false;
		}

		// check the embedding indexation for the concrete map
		result = cmap->check_embedding_integrity();
		if (!result)
		{
			cgogn_log_error("check_map_integrity") << "Integrity of the embeddings is broken";
			return false;
		}
		return true;
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
		if (this->template is_embedded<ORBIT>())
			return this->get_embedding(c1) == this->get_embedding(c2);

		bool result = false;
		to_concrete()->foreach_dart_of_orbit_until(c1, [&] (Dart d) -> bool
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
	uint32 nb_darts() const
	{
		return this->topology_.size();
	}

	/**
	 * \brief return the number of cells of the given orbit in the map
	 */
	template <Orbit ORBIT>
	uint32 nb_cells() const
	{
		if (this->template is_embedded<ORBIT>())
			return this->attributes_[ORBIT].size();
		else
		{
			uint32 result = 0;
			foreach_cell([&result] (Cell<ORBIT>) { ++result; });
			return result;
		}
	}

	template <Orbit ORBIT, typename MASK>
	uint32 nb_cells(const MASK& mask) const
	{
		uint32 result = 0;
		foreach_cell([&result] (Cell<ORBIT>) { ++result; }, mask);
		return result;
	}

	/**
	 * \brief return the number of darts in the given cell
	 */
	template <Orbit ORBIT>
	uint32 nb_darts_of_orbit(Cell<ORBIT> c) const
	{
		uint32 result = 0u;
		to_concrete()->foreach_dart_of_orbit(c, [&result] (Dart) { ++result; });
		return result;
	}

	inline bool is_boundary(Dart d) const
	{
		return (*this->boundary_marker_)[d.index];
	}

	inline void set_boundary(Dart d, bool b)
	{
		this->boundary_marker_->set_value(d.index, b);
	}

	template <Orbit ORBIT>
	inline void fill_hole(Cell<ORBIT> c)
	{
		to_concrete()->foreach_dart_of_orbit(c, [this] (Dart d)
		{
			set_boundary(d, false);
		});
	}

	template <Orbit ORBIT>
	inline void create_hole(Cell<ORBIT> c)
	{
		to_concrete()->foreach_dart_of_orbit(c, [this] (Dart d)
		{
			set_boundary(d, true);
		});
	}

	template <typename CellType>
	bool is_incident_to_boundary(CellType c) const
	{
		static_assert(!std::is_same<CellType, typename ConcreteMap::Boundary>::value, "is_incident_to_boundary is not defined for boundary cells");
		bool result = false;
		to_concrete()->foreach_dart_of_orbit_until(c, [this, &result] (Dart d)
		{
			if (is_boundary(d)) { result = true; return false; }
			return true;
		});
		return result;
	}

	template <typename CellType>
	Dart find_incident_to_boundary(CellType c) const
	{
		static_assert(!std::is_same<CellType, typename ConcreteMap::Boundary>::value, "find_incident_to_boundary is not defined for boundary cells");
		Dart result;
		to_concrete()->foreach_dart_of_orbit_until(c, [this, &result] (Dart d)
		{
			if (is_boundary(d)) { result = d; return false; }
			return true;
		});
		return result;
	}


	/*******************************************************************************
	 * Traversals
	 *******************************************************************************/

public:

	/**
	 * \brief apply a function on each dart of the map (including boundary darts)
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <typename FUNC>
	inline void foreach_dart(const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Dart), "Wrong function parameter type");

		for (Dart it = Dart(this->topology_.begin()), last = Dart(this->topology_.end()); it != last; this->topology_.next(it.index))
			f(it);
	}

	template <typename FUNC>
	inline void parallel_foreach_dart(const FUNC& f) const
	{
		static_assert(check_func_ith_parameter_type(FUNC, 0, Dart), "Wrong function first parameter type");
		static_assert(check_func_ith_parameter_type(FUNC, 1, uint32), "Wrong function second parameter type");

		using Future = std::future<typename std::result_of<FUNC(Dart, uint32)>::type>;
		using VecDarts = std::vector<Dart>;

		ThreadPool* thread_pool = cgogn::get_thread_pool();
		const std::size_t nb_threads_pool = thread_pool->get_nb_threads();

		std::array<std::vector<VecDarts*>, 2> dart_buffers;
		std::array<std::vector<Future>, 2> futures;
		dart_buffers[0].reserve(nb_threads_pool);
		dart_buffers[1].reserve(nb_threads_pool);
		futures[0].reserve(nb_threads_pool);
		futures[1].reserve(nb_threads_pool);

		Buffers<Dart>* dbuffs = cgogn::get_dart_buffers();

		Dart it = Dart(this->topology_.begin());
		Dart last = Dart(this->topology_.end());

		while (it != last)
		{
			for (uint32 i = 0u; i < 2u; ++i)
			{
				for (uint32 j = 0u; j < nb_threads_pool && it.index < last.index; ++j)
				{
					dart_buffers[i].push_back(dbuffs->get_buffer());
					cgogn_assert(dart_buffers[i].size() <= nb_threads_pool);
					std::vector<Dart>& darts = *dart_buffers[i].back();
					darts.reserve(PARALLEL_BUFFER_SIZE);
					for (unsigned k = 0u; k < PARALLEL_BUFFER_SIZE && it.index < last.index; ++k)
					{
						darts.push_back(it);
						this->topology_.next(it.index);
					}

					futures[i].push_back(thread_pool->enqueue([&darts, &f] (uint32 th_id)
					{
						for (auto d : darts)
							f(d, th_id);
					}));
				}

				const uint32 id = (i+1u) % 2u;

				for (auto& fu : futures[id])
					fu.wait();
				for (auto& b : dart_buffers[id])
					dbuffs->release_cell_buffer(b);

				futures[id].clear();
				dart_buffers[id].clear();

				// if we reach the end of the map while filling buffers from the second set we need to clean them too.
				if (it.index >= last.index && i == 1u)
				{
					for (auto& fu : futures[1u])
						fu.wait();
					for (auto &b : dart_buffers[1u])
						dbuffs->release_buffer(b);
				}
			}
		}
	}

	/**
	 * \brief apply a function on each dart of the map (including boundary darts) and stops when the function returns false
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <typename FUNC>
	inline void foreach_dart_until(const FUNC& f) const
	{
		static_assert(check_func_parameter_type(FUNC, Dart), "Wrong function parameter type");
		static_assert(check_func_return_type(FUNC, bool), "Wrong function return type");

		for (Dart it = Dart(this->topology_.begin()), last = Dart(this->topology_.end()); it != last; this->topology_.next(it.index))
		{
			if (!f(it))
				break;
		}
	}

protected:

	/*!
	 * \Brief Methods to iterate over darts.
	 * These functions skip boundary darts.
	 */
	inline Dart begin() const
	{
		Dart d = Dart(this->topology_.begin());
		Dart end = Dart(this->topology_.end());

		while (d != end && is_boundary(d))
			this->topology_.next(d.index);

		return d;
	}

	inline void next(Dart& d) const
	{
		Dart end = Dart(this->topology_.end());

		do {
			this->topology_.next(d.index);
		} while (d != end && is_boundary(d));
	}

	inline Dart end() const
	{
		return Dart(this->topology_.end());
	}

public:

	/**
	 * \brief apply a function on each cell of the map (boundary cells excluded)
	 * (the dimension of the traversed cells is determined based on the parameter of the given callable)
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <TraversalStrategy STRATEGY = TraversalStrategy::AUTO, typename FUNC>
	inline void foreach_cell(const FUNC& f) const
	{
		using CellType = func_parameter_type(FUNC);

		foreach_cell<STRATEGY>(f, [] (CellType) { return true; });
	}

	template <TraversalStrategy STRATEGY = TraversalStrategy::AUTO, typename FUNC>
	inline void parallel_foreach_cell(const FUNC& f) const
	{
		using CellType = func_parameter_type(FUNC);

		parallel_foreach_cell<STRATEGY>(f, [] (CellType) { return true; });
	}

	template <TraversalStrategy STRATEGY = TraversalStrategy::AUTO, typename FUNC>
	inline void foreach_cell_until(const FUNC& f) const
	{
		foreach_cell_until<STRATEGY>(f, [] (Dart) { return true; });
	}

	/**
	 * \brief apply a function on each cell of the map (boundary cells excluded)
	 * that is selected by the given FilterFunction (CellType -> bool)
	 * (the dimension of the traversed cells is determined based on the parameter of the given callable)
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <TraversalStrategy STRATEGY = TraversalStrategy::AUTO,
			  typename FUNC,
			  typename FilterFunction,
			  typename std::enable_if<check_func_return_type(FilterFunction, bool) && check_func_parameter_type(FilterFunction, func_parameter_type(FUNC))>::type* = nullptr>
	inline void foreach_cell(const FUNC& f, const FilterFunction& filter) const
	{
		using CellType = func_parameter_type(FUNC);
		static const Orbit ORBIT = CellType::ORBIT;

		switch (STRATEGY)
		{
			case FORCE_DART_MARKING :
				foreach_cell_dart_marking(f, filter);
				break;
			case FORCE_CELL_MARKING :
				foreach_cell_cell_marking(f, filter);
				break;
			case AUTO :
				if (this->template is_embedded<CellType>())
					foreach_cell_cell_marking(f, filter);
				else
					foreach_cell_dart_marking(f, filter);
				break;
		}
	}

	template <TraversalStrategy STRATEGY = TraversalStrategy::AUTO,
			  typename FUNC,
			  typename FilterFunction,
			  typename std::enable_if<check_func_return_type(FilterFunction, bool) && check_func_parameter_type(FilterFunction, func_parameter_type(FUNC))>::type* = nullptr>
	inline void parallel_foreach_cell(const FUNC& f, const FilterFunction& filter) const
	{
		static_assert(check_func_ith_parameter_type(FUNC, 1, uint32), "Wrong function second parameter type");

		using CellType = func_parameter_type(FUNC);
		static const Orbit ORBIT = CellType::ORBIT;

		switch (STRATEGY)
		{
			case FORCE_DART_MARKING :
				parallel_foreach_cell_dart_marking(f, filter);
				break;
			case FORCE_CELL_MARKING :
				parallel_foreach_cell_cell_marking(f, filter);
				break;
			case AUTO :
				if (this->template is_embedded<CellType>())
					parallel_foreach_cell_cell_marking(f, filter);
				else
					parallel_foreach_cell_dart_marking(f, filter);
				break;
		}
	}

	template <TraversalStrategy STRATEGY = TraversalStrategy::AUTO,
			  typename FUNC,
			  typename FilterFunction,
			  typename std::enable_if<check_func_return_type(FilterFunction, bool) && check_func_parameter_type(FilterFunction, func_parameter_type(FUNC))>::type* = nullptr>
	void foreach_cell_until(const FUNC& f, const FilterFunction& filter) const
	{
		using CellType = func_parameter_type(FUNC);

		switch (STRATEGY)
		{
			case FORCE_DART_MARKING :
				foreach_cell_until_dart_marking(f, filter);
				break;
			case FORCE_CELL_MARKING :
				foreach_cell_until_cell_marking(f, filter);
				break;
			case AUTO :
				if (this->template is_embedded<CellType>())
					foreach_cell_until_cell_marking(f, filter);
				else
					foreach_cell_until_dart_marking(f, filter);
				break;
		}
	}

	/**
	 * \brief apply a function on each cell of the map (boundary cells excluded)
	 * that is selected by the filter function of the corresponding CellType within the given Filters object
	 * (the dimension of the traversed cells is determined based on the parameter of the given callable)
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <typename FUNC,
			  typename Filters,
			  typename std::enable_if<std::is_base_of<CellFilters, Filters>::value>::type* = nullptr>
	inline void foreach_cell(const FUNC& f, const Filters& filters) const
	{
		using CellType = func_parameter_type(FUNC);

		foreach_cell(f, [&filters] (CellType c) { return filters.filter(c); });
	}

	template <typename FUNC,
			  typename Filters,
			  typename std::enable_if<std::is_base_of<CellFilters, Filters>::value>::type* = nullptr>
	inline void parallel_foreach_cell(const FUNC& f, const Filters& filters) const
	{
		using CellType = func_parameter_type(FUNC);

		parallel_foreach_cell(f, [&filters] (CellType c) { return filters.filter(c); });
	}

	template <typename FUNC,
			  typename Filters,
			  typename std::enable_if<std::is_base_of<CellFilters, Filters>::value>::type* = nullptr>
	inline void foreach_cell_until(const FUNC& f, const Filters& filters) const
	{
		using CellType = func_parameter_type(FUNC);

		foreach_cell_until(f, [&filters] (CellType c) { return filters.filter(c); });
	}

	/**
	 * \brief apply a function on each cell of the map that is provided by the given Traversor object
	 * (the dimension of the traversed cells is determined based on the parameter of the given callable)
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <typename FUNC,
			  typename Traversor,
			  typename std::enable_if<std::is_base_of<CellTraversor, Traversor>::value>::type* = nullptr>
	inline void foreach_cell(const FUNC& f, const Traversor& t) const
	{
		using CellType = func_parameter_type(FUNC);

		for (CellType it = t.template begin<CellType>(); !t.template end<CellType>(); it = t.template next<CellType>())
			f(it);
	}

	template <typename FUNC,
			  typename Traversor,
			  typename std::enable_if<std::is_base_of<CellTraversor, Traversor>::value>::type* = nullptr>
	inline void parallel_foreach_cell(const FUNC& f, const Traversor& t) const
	{
		static_assert(check_func_ith_parameter_type(FUNC, 1, uint32), "Wrong function second parameter type");

		using CellType = func_parameter_type(FUNC);

		using VecCell = std::vector<CellType>;
		using Future = std::future<typename std::result_of<FUNC(CellType, uint32)>::type>;

		ThreadPool* thread_pool = cgogn::get_thread_pool();
		const std::size_t nb_threads_pool = thread_pool->get_nb_threads();

		std::array<std::vector<VecCell*>, 2> cells_buffers;
		std::array<std::vector<Future>, 2> futures;
		cells_buffers[0].reserve(nb_threads_pool);
		cells_buffers[1].reserve(nb_threads_pool);
		futures[0].reserve(nb_threads_pool);
		futures[1].reserve(nb_threads_pool);

		Buffers<Dart>* dbuffs = cgogn::get_dart_buffers();

		CellType it = t.template begin<CellType>();

		uint32 i = 0u; // buffer id (0/1)
		uint32 j = 0u; // thread id (0..nb_threads_pool)
		while (!t.template end<CellType>())
		{
			// fill buffer
			cells_buffers[i].push_back(dbuffs->template get_cell_buffer<CellType>());
			VecCell& cells = *cells_buffers[i].back();
			cells.reserve(PARALLEL_BUFFER_SIZE);
			for (unsigned k = 0u; k < PARALLEL_BUFFER_SIZE && !t.template end<CellType>(); ++k)
			{
				CellType c(it);
				cells.push_back(c);
				it = t.template next<CellType>();
			}
			// launch thread
			futures[i].push_back(thread_pool->enqueue([&cells, &f] (uint32 th_id)
			{
				for (auto c : cells)
					f(c, th_id);
			}));
			// next thread
			if (++j == nb_threads_pool)
			{	// again from 0 & change buffer
				j = 0;
				const uint32 id = (i+1u) % 2u;
				for (auto& fu : futures[id])
					fu.wait();
				for (auto& b : cells_buffers[id])
					dbuffs->release_cell_buffer(b);
				futures[id].clear();
				cells_buffers[id].clear();
			}
		}

		// clean all at end
		for (auto& fu : futures[0u])
			fu.wait();
		for (auto& b : cells_buffers[0u])
			dbuffs->release_cell_buffer(b);
		for (auto& fu : futures[1u])
			fu.wait();
		for (auto& b : cells_buffers[1u])
			dbuffs->release_cell_buffer(b);
	}

	template <typename FUNC,
			  typename Traversor,
			  typename std::enable_if<std::is_base_of<CellTraversor, Traversor>::value>::type* = nullptr>
	inline void foreach_cell_until(const FUNC& f, const Traversor& t) const
	{
		using CellType = func_parameter_type(FUNC);

		for (CellType it = t.template begin<CellType>(); !t.template end<CellType>(); it = t.template next<CellType>())
			if (!f(it))
				break;
	}

protected:

	template <typename FUNC, typename FilterFunction>
	inline void foreach_cell_dart_marking(const FUNC& f, const FilterFunction& filter) const
	{
		using CellType = func_parameter_type(FUNC);

		const ConcreteMap* cmap = to_concrete();
		DartMarker dm(*cmap);
		for (Dart it = cmap->begin(), last = cmap->end(); it.index < last.index; cmap->next(it))
		{
			if (!dm.is_marked(it))
			{
				CellType c(it);
				dm.mark_orbit(c);
				if (filter(c))
					f(c);
			}
		}
	}

	template <typename FUNC, typename FilterFunction>
	inline void parallel_foreach_cell_dart_marking(const FUNC& f, const FilterFunction& filter) const
	{
		using CellType = func_parameter_type(FUNC);

		using VecCell = std::vector<CellType>;
		using Future = std::future<typename std::result_of<FUNC(CellType, uint32)>::type>;

		ThreadPool* thread_pool = cgogn::get_thread_pool();
		const std::size_t nb_threads_pool = thread_pool->get_nb_threads();

		std::array<std::vector<VecCell*>, 2> cells_buffers;
		std::array<std::vector<Future>, 2> futures;
		cells_buffers[0].reserve(nb_threads_pool);
		cells_buffers[1].reserve(nb_threads_pool);
		futures[0].reserve(nb_threads_pool);
		futures[1].reserve(nb_threads_pool);

		Buffers<Dart>* dbuffs = cgogn::get_dart_buffers();

		const ConcreteMap* cmap = to_concrete();
		DartMarker dm(*cmap);
		Dart it = cmap->begin();
		Dart last = cmap->end();

		uint32 i = 0u; // buffer id (0/1)
		uint32 j = 0u; // thread id (0..nb_threads_pool)
		while (it.index < last.index)
		{
			// fill buffer
			cells_buffers[i].push_back(dbuffs->template get_cell_buffer<CellType>());
			VecCell& cells = *cells_buffers[i].back();
			cells.reserve(PARALLEL_BUFFER_SIZE);
			for (unsigned k = 0u; k < PARALLEL_BUFFER_SIZE && it.index < last.index; )
			{
				if (!dm.is_marked(it))
				{
					CellType c(it);
					dm.mark_orbit(c);
					if (filter(c))
					{
						cells.push_back(c);
						++k;
					}
				}
				cmap->next(it);
			}
			//launch thread
			futures[i].push_back(thread_pool->enqueue([&cells, &f] (uint32 th_id)
			{
				for (auto c : cells)
					f(c, th_id);
			}));
			// next thread
			if (++j == nb_threads_pool)
			{	// again from 0 & change buffer
				j = 0;
				const uint32 id = (i+1u) % 2u;
				for (auto& fu : futures[id])
					fu.wait();
				for (auto& b : cells_buffers[id])
					dbuffs->release_cell_buffer(b);
				futures[id].clear();
				cells_buffers[id].clear();
			}
		}

		// clean all at end
		for (auto& fu : futures[0u])
			fu.wait();
		for (auto &b : cells_buffers[0u])
			dbuffs->release_cell_buffer(b);
		for (auto& fu : futures[1u])
			fu.wait();
		for (auto &b : cells_buffers[1u])
			dbuffs->release_cell_buffer(b);
	}

	template <typename FUNC, typename FilterFunction>
	inline void foreach_cell_cell_marking(const FUNC& f, const FilterFunction& filter) const
	{
		using CellType = func_parameter_type(FUNC);
		static const Orbit ORBIT = CellType::ORBIT;

		const ConcreteMap* cmap = to_concrete();
		CellMarker<ORBIT> cm(*cmap);
		for (Dart it = cmap->begin(), last = cmap->end(); it.index < last.index; cmap->next(it))
		{
			CellType c(it);
			if (!cm.is_marked(c))
			{
				cm.mark(c);
				if (filter(c))
					f(c);
			}
		}
	}

	template <typename FUNC, typename FilterFunction>
	inline void parallel_foreach_cell_cell_marking(const FUNC& f, const FilterFunction& filter) const
	{
		using CellType = func_parameter_type(FUNC);
		static const Orbit ORBIT = CellType::ORBIT;

		using VecCell = std::vector<CellType>;
		using Future = std::future<typename std::result_of<FUNC(CellType, uint32)>::type>;

		ThreadPool* thread_pool = cgogn::get_thread_pool();
		const std::size_t nb_threads_pool = thread_pool->get_nb_threads();

		std::array<std::vector<VecCell*>, 2> cells_buffers;
		std::array<std::vector<Future>, 2> futures;
		cells_buffers[0].reserve(nb_threads_pool);
		cells_buffers[1].reserve(nb_threads_pool);
		futures[0].reserve(nb_threads_pool);
		futures[1].reserve(nb_threads_pool);

		Buffers<Dart>* dbuffs = cgogn::get_dart_buffers();

		const ConcreteMap* cmap = to_concrete();
		CellMarker<ORBIT> cm(*cmap);
		Dart it = cmap->begin();
		Dart last = cmap->end();

		uint32 i = 0u; // buffer id (0/1)
		uint32 j = 0u; // thread id (0..nb_threads_pool)
		while (it.index < last.index)
		{
			// fill buffer
			cells_buffers[i].push_back(dbuffs->template get_cell_buffer<CellType>());
			VecCell& cells = *cells_buffers[i].back();
			cells.reserve(PARALLEL_BUFFER_SIZE);
			for (unsigned k = 0u; k < PARALLEL_BUFFER_SIZE && it.index < last.index; )
			{
				CellType c(it);
				if (!cm.is_marked(c))
				{
					cm.mark(c);
					if (filter(c))
					{
						cells.push_back(c);
						++k;
					}
				}
				cmap->next(it);
			}
			// launch thread
			futures[i].push_back(thread_pool->enqueue([&cells, &f] (uint32 th_id)
			{
				for (auto c : cells)
					f(c, th_id);
			}));
			// next thread
			if (++j == nb_threads_pool)
			{	// again from 0 & change buffer
				j = 0;
				const uint32 id = (i+1u) % 2u;
				for (auto& fu : futures[id])
					fu.wait();
				for (auto& b : cells_buffers[id])
					dbuffs->release_cell_buffer(b);
				futures[id].clear();
				cells_buffers[id].clear();
			}
		}

		// clean all at end
		for (auto& fu : futures[0u])
			fu.wait();
		for (auto& b : cells_buffers[0u])
			dbuffs->release_cell_buffer(b);
		for (auto& fu : futures[1u])
			fu.wait();
		for (auto& b : cells_buffers[1u])
			dbuffs->release_cell_buffer(b);
	}

	template <typename FUNC, typename FilterFunction>
	inline void foreach_cell_until_dart_marking(const FUNC& f, const FilterFunction& filter) const
	{
		using CellType = func_parameter_type(FUNC);

		const ConcreteMap* cmap = to_concrete();
		DartMarker dm(*cmap);
		for (Dart it = cmap->begin(), last = cmap->end(); it.index < last.index; cmap->next(it))
		{
			if (!dm.is_marked(it))
			{
				CellType c(it);
				dm.mark_orbit(c);
				if(filter(c))
					if(!f(c))
						break;
			}
		}
	}

	template <typename FUNC, typename FilterFunction>
	inline void foreach_cell_until_cell_marking(const FUNC& f, const FilterFunction& filter) const
	{
		using CellType = func_parameter_type(FUNC);
		static const Orbit ORBIT = CellType::ORBIT;

		const ConcreteMap* cmap = to_concrete();
		CellMarker<ORBIT> cm(*cmap);
		for (Dart it = cmap->begin(), last = cmap->end(); it.index < last.index; cmap->next(it))
		{
			CellType c(it);
			if (!cm.is_marked(c))
			{
				cm.mark(c);
				if(filter(c))
					if(!f(c))
						break;
			}
		}
	}
};

} // namespace cgogn

#endif // CGOGN_CORE_CMAP_MAP_BASE_H_

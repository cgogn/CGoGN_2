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

#include <cgogn/core/utils/masks.h>
#include <cgogn/core/utils/logger.h>
#include <cgogn/core/utils/unique_ptr.h>
#include <cgogn/core/utils/type_traits.h>

#include <cgogn/core/basic/cell.h>
#include <cgogn/core/basic/dart_marker.h>
#include <cgogn/core/basic/cell_marker.h>

#include <cgogn/core/cmap/map_base_data.h>
#include <cgogn/core/cmap/attribute.h>

namespace cgogn
{

enum TraversalStrategy
{
	AUTO = 0,
	FORCE_DART_MARKING,
	FORCE_CELL_MARKING
};

template <typename MAP_TYPE>
class MapBase : public MapBaseData
{
public:

	using Inherit = MapBaseData;
	using Self = MapBase<MAP_TYPE>;

	template <typename MAP> friend class DartMarker_T;
	template <typename MAP, Orbit ORBIT> friend class CellMarker_T;

	using typename Inherit::ChunkArrayGen;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;
	using typename Inherit::ChunkArrayBool;
	template <typename T_REF>
	using ChunkArrayContainer = typename Inherit::template ChunkArrayContainer<T_REF>;

	using ConcreteMap = typename MAP_TYPE::TYPE;

	using DartMarker = cgogn::DartMarker<ConcreteMap>;
	using DartMarkerStore = cgogn::DartMarkerStore<ConcreteMap>;

	template <Orbit ORBIT>
	using CellMarker = cgogn::CellMarker<ConcreteMap, ORBIT>;
	template <Orbit ORBIT>
	using CellMarkerStore = cgogn::CellMarkerStore<ConcreteMap, ORBIT>;
	template <Orbit ORBIT>
	using CellMarkerNoUnmark = typename cgogn::CellMarkerNoUnmark<ConcreteMap, ORBIT>;

	MapBase() :	Inherit() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MapBase);
	~MapBase() {}

	inline uint8 dimension_concrete_map() const
	{
		return ConcreteMap::DIMENSION;
	}

	/**
	 * @brief clear : clear the topology (empty the dart attributes including embeddings) leaving the other attributes unmodified
	 */
	inline void clear()
	{
		this->topology_.clear_chunk_arrays();

		for (uint32 i = 0u; i < NB_ORBITS; ++i)
			this->attributes_[i].clear_chunk_arrays();
	}

	/**
	 * @brief clear_and_remove_attributes : clear the topology and delete all the attributes.
	 */
	inline void clear_and_remove_attributes()
	{
		// 1st step : some cleaning
		this->topology_.clear_chunk_arrays();

		for (auto& att : this->attributes_)
			att.remove_chunk_arrays();


		// 2nd step : updating internal data structures.
		{
			std::lock_guard<std::mutex> lock(this->mark_attributes_topology_mutex_);
			for (ChunkArrayBool* cab : this->topology_.marker_arrays())
				cab->clear();
		}

		for (std::size_t i = 0u; i < NB_ORBITS; ++i)
		{
			if (this->embeddings_[i] != nullptr)
			{
				this->topology_.remove_chunk_array(this->embeddings_[i]);
				this->embeddings_[i] = nullptr;
			}

			std::lock_guard<std::mutex> lock(this->mark_attributes_mutex_[i]);
			for (ChunkArrayBool* cab : this->attributes_[i].marker_arrays())
				cab->clear();
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

	/**
	 * \brief Adds a topological element of PRIM_SIZE to the topology container
	 * \return the index of the added element
	 * Adding a topological element consists in adding PRIM_SIZE lines
	 * to the topological container starting from index
	 */
	inline Dart add_topology_element()
	{
		const uint32 idx = this->topology_.template insert_lines<ConcreteMap::PRIM_SIZE>();
		for (uint32 jdx = idx; jdx < idx + ConcreteMap::PRIM_SIZE; ++jdx)
		{
			this->topology_.init_markers_of_line(jdx);
			for (uint32 orbit = 0u; orbit < NB_ORBITS; ++orbit)
			{
				if (this->embeddings_[orbit])
					(*this->embeddings_[orbit])[jdx] = INVALID_INDEX;
			}
			to_concrete()->init_dart(Dart(jdx));
		}
		return Dart(idx);
	}

	/**
	 * \brief Removes a topological element of PRIM_SIZE from the topology container
	 * \param d the element to remove ( or one them if PRIM_SIZE >1)
	 * Removing a topological element consists in removing PRIM_SIZE lines
	 * of the topological container starting from index
	 */
	inline void remove_topology_element(Dart d)
	{
		uint32 index = d.index;
		this->topology_.template remove_lines<ConcreteMap::PRIM_SIZE>(index);

		for (uint32 orbit = 0; orbit < NB_ORBITS; ++orbit)
		{
			if (this->embeddings_[orbit])
			{
				for (uint32 jdx = index; jdx < index + ConcreteMap::PRIM_SIZE; ++jdx)
				{
					uint32 emb = (*this->embeddings_[orbit])[jdx];
					if (emb != INVALID_INDEX)
						this->attributes_[orbit].unref_line(emb);
				}
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

	//	template <Orbit ORBIT>
	//	inline void compact_orbit_container()
	//	{
	//		if (!this->template is_embedded<ORBIT>())
	//			return;

	//		auto& cac = this->template attribute_container<ORBIT>();
	//		const std::vector<unsigned int>& map_old_new = cac.template compact<ConcreteMap::PRIM_SIZE>();
	//		this->parallel_foreach_dart([&map_old_new,this](Dart d, uint32)
	//		{
	//			uint32& old_idx = this->embeddings_[ORBIT]->operator[](d);
	//			const uint32 new_idx = map_old_new[old_idx];
	//			if (new_idx != UINT32_MAX)
	//				old_idx = new_idx;
	//		});
	//	}

public:

	/*******************************************************************************
	 * Attributes management
	 *******************************************************************************/

	inline bool has_attribute(Orbit orbit, const std::string& att_name)
	{
		cgogn_message_assert(orbit < NB_ORBITS, "Unknown orbit parameter");
		return this->attributes_[orbit].has_array(att_name);
	}

	/**
	 * \brief add an attribute
	 * @param attribute_name the name of the attribute to create
	 * @return a handler to the created attribute
	 */
	template <typename T, Orbit ORBIT>
	inline Attribute<T, ORBIT> add_attribute(const std::string& attribute_name)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		if (!this->template is_embedded<ORBIT>())
			create_embedding<ORBIT>();
		ChunkArray<T>* ca = this->attributes_[ORBIT].template add_chunk_array<T>(attribute_name);
		return Attribute<T, ORBIT>(this, ca);
	}

	/**
	* \brief search an attribute for a given orbit
	* @param attribute_name attribute name
	* @return an Attribute
	*/
	template <typename T, Orbit ORBIT>
	inline Attribute<T, ORBIT> get_attribute(const std::string& attribute_name) const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		ChunkArray<T>* ca = const_cast<Self*>(this)->attributes_[ORBIT].template get_chunk_array<T>(attribute_name);
		return Attribute<T, ORBIT>(const_cast<Self*>(this), ca);
	}

	/**
	 * \brief remove an attribute
	 * @param ah a handler to the attribute to remove
	 * @return true if remove succeed else false
	 */
	template <typename T>
	inline bool remove_attribute(const Attribute_T<T>& ah)
	{
		return this->attributes_[ah.orbit()].remove_chunk_array(ah.data());
	}

	/**
	 * \brief remove_attribute
	 * @param orbit, the attribute orbit
	 * @param att_name attribute name
	 * @return true if remove succeed else false
	 */
	inline bool remove_attribute(Orbit orbit, const std::string& att_name)
	{
		cgogn_message_assert(orbit < NB_ORBITS, "Unknown orbit parameter");
		return this->attributes_[orbit].remove_chunk_array(att_name);
	}

	/**
	 * \brief Second version of add_attribute taking a Cell as template paramter instead of an Orbit
	 */
	template <typename T, typename CellType>
	inline Attribute<T, CellType::ORBIT> add_attribute(const std::string& attribute_name)
	{
		return this->add_attribute<T, CellType::ORBIT>(attribute_name);
	}

	/**
	 * \brief Second version of get_attribute taking a Cell as template paramter instead of an Orbit
	 */
	template <typename T, typename CellType>
	inline Attribute<T, CellType::ORBIT> get_attribute(const std::string& attribute_name) const
	{
		return this->get_attribute<T, CellType::ORBIT>(attribute_name);
	}

	/**
	 * \brief Third version of get_attribute taking a single type template parameter T and returning an Attribute_T<T>
	 */
	template <typename T>
	inline Attribute_T<T> get_attribute(Orbit orbit, const std::string& attribute_name) const
	{
		cgogn_message_assert(orbit < NB_ORBITS, "Unknown orbit parameter");

		ChunkArray<T>* ca = const_cast<Self*>(this)->attributes_[orbit].template get_chunk_array<T>(attribute_name);
		return Attribute_T<T>(const_cast<Self*>(this), ca, orbit);
	}

	/**
	* \brief search an attribute for a given orbit and change its type (if size is compatible). First template arg is asked type, second is real type.
	* @param attribute_name attribute name
	* @return an Attribute
	*/
	template <typename T_ASK, typename T_ATT, Orbit ORBIT>
	inline Attribute<T_ASK, ORBIT> get_attribute_force_type(const std::string& attribute_name) const
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		static_assert(sizeof(T_ASK) == sizeof(T_ATT), "Incompatible casting operation between attributes, sizes are differents");

		const ChunkArray<T_ASK>* ca = reinterpret_cast<const ChunkArray<T_ASK>*>(this->attributes_[ORBIT].template get_chunk_array<T_ATT>(attribute_name));
		return Attribute<T_ASK, ORBIT>(const_cast<Self*>(this), const_cast<ChunkArray<T_ASK>*>(ca));
	}

	template <typename T, Orbit ORBIT>
	inline void swap_attributes(Attribute<T, ORBIT>& ah1, Attribute<T, ORBIT>& ah2)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		cgogn_message_assert(ah1.is_linked_to(this), "swap_attributes: wrong map");
		cgogn_message_assert(ah2.is_linked_to(this), "swap_attributes: wrong map");

		this->attributes_[ORBIT].swap_chunk_arrays(ah1.data(), ah2.data());
	}

	template <typename T, Orbit ORBIT>
	inline void copy_attribute(Attribute<T, ORBIT>& dest, Attribute<T, ORBIT>& src)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		cgogn_message_assert(dest.is_linked_to(this), "copy_attribute: wrong map");
		cgogn_message_assert(src.is_linked_to(this), "copy_attribute: wrong map");

		this->attributes_[ORBIT].copy_chunk_array_data(dest.data(), src.data());
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
	inline ChunkArrayBool* mark_attribute()
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");

		const std::size_t thread = cgogn::current_thread_marker_index();
		cgogn_assert(thread < mark_attributes_[ORBIT].size());

		if (!this->mark_attributes_[ORBIT][thread].empty())
		{
			ChunkArrayBool* ca = this->mark_attributes_[ORBIT][thread].back();
			this->mark_attributes_[ORBIT][thread].pop_back();
			return ca;
		}
		else
		{
			std::lock_guard<std::mutex> lock(this->mark_attributes_mutex_[ORBIT]);
			if (!this->template is_embedded<ORBIT>())
				create_embedding<ORBIT>();
			ChunkArrayBool* ca = this->attributes_[ORBIT].add_marker_attribute();
			return ca;
		}
	}

	/**
	* \brief release a mark attribute on the given ORBIT attribute container
	* @param the mark attribute to release
	*/
	template <Orbit ORBIT>
	inline void release_mark_attribute(ChunkArrayBool* ca)
	{
		static_assert(ORBIT < NB_ORBITS, "Unknown orbit parameter");
		cgogn_message_assert(this->template is_embedded<ORBIT>(), "Invalid parameter: orbit not embedded");
		cgogn_assert(cgogn::current_thread_marker_index() < mark_attributes_[ORBIT].size());

		this->mark_attributes_[ORBIT][cgogn::current_thread_marker_index()].push_back(ca);
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
		ChunkArray<uint32>* ca = this->topology_.template add_chunk_array<uint32>(oss.str());
		this->embeddings_[ORBIT] = ca;

		// initialize all darts indices to INVALID_INDEX for this ORBIT
		foreach_dart([ca] (Dart d) { (*ca)[d.index] = INVALID_INDEX; });

		// initialize the indices of the existing orbits
		foreach_cell<FORCE_DART_MARKING>([this] (Cell<ORBIT> c) { this->new_orbit_embedding(c); });

//		cgogn_assert(this->template is_well_embedded<Cell<ORBIT>>());
	}

	/**
	 * \brief the darts of the given cell are indexed for CellType with the given embedding
	 */
	template <typename CellType, Orbit ORBIT>
	void set_orbit_embedding(Cell<ORBIT> c, uint32 emb)
	{
		to_concrete()->foreach_dart_of_orbit(c, [this, emb] (Dart d)
		{
			this->template set_embedding<CellType>(d, emb);
		});
	}

	/**
	 * \brief creates a new embedding and set it to the darts of the given cell
	 * \return the new index
	 */
	template <Orbit ORBIT>
	inline uint32 new_orbit_embedding(Cell<ORBIT> c)
	{
		const uint32 emb = add_attribute_element<ORBIT>();
		set_orbit_embedding<Cell<ORBIT>>(c, emb);
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
				const uint32 old_emb = this->embedding(c);
				const uint32 new_emb = this->new_orbit_embedding(c);
				cgogn_log_warning("enforce_unique_orbit_embedding") << "Warning: enforce_unique_orbit_embedding: duplicating orbit #" << old_emb << " in orbit " << orbit_name(ORBIT);
				this->attributes_[ORBIT].copy_line(new_emb, old_emb, false, false);
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

		const ChunkArrayContainer<uint32>& container = this->attributes_[ORBIT];

		// Check that the indexation of cells is correct
		foreach_cell<FORCE_DART_MARKING>([&] (CellType c)
		{
			const uint32 idx = this->embedding(c);
			// check used indices are valid
			if (idx == INVALID_INDEX)
			{
				result = false;
				cgogn_log_error("is_well_embedded") << "INVALID_INDEX found for dart " << c << " in orbit " << orbit_name(ORBIT);
				return;
			}
			counter[idx].push_back(c);
			uint32 refs = 1;
			// check all darts of the cell use the same index (distinct to INVALID_INDEX)
			cmap->foreach_dart_of_orbit(c, [&] (Dart d)
			{
				const uint32 emb_d = this->embedding(CellType(d));
				if (emb_d != idx)
				{
					cgogn_log_error("is_well_embedded") << "Different indices (" << idx << " and " << emb_d << ") in orbit " << orbit_name(ORBIT);
					result = false;
				}
				refs++;
			});
			if (refs != container.nb_refs(this->embedding(c)))
			{
				cgogn_log_error("is_well_embedded") << "Wrong reference number of embedding " << this->embedding(c) << " in orbit " << orbit_name(ORBIT);
				result = false;
			}

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
		foreach_dart([&cmap, &result] (Dart d) -> bool
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
	 * \brief return true if c1 and c2 represent the same cell
	 * Comparison is done using exclusively the topological information (darts)
	 * @tparam ORBIT considered orbit
	 * @param c1 first cell to compare
	 * @param c2 second cell to compare
	 */
	template <Orbit ORBIT>
	bool same_orbit(Cell<ORBIT> c1, Cell<ORBIT> c2) const
	{
		bool result = false;
		to_concrete()->foreach_dart_of_orbit(c1, [&] (Dart d) -> bool
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
	 * \brief return true if c1 and c2 represent the same cell
	 * If the orbit is embedded, the comparison is done on the indices, otherwise it is done using the darts
	 * @tparam ORBIT considered orbit
	 * @param c1 first cell to compare
	 * @param c2 second cell to compare
	 */
	template <Orbit ORBIT>
	bool same_cell(Cell<ORBIT> c1, Cell<ORBIT> c2) const
	{
		if (this->template is_embedded<ORBIT>() && !is_boundary_cell(c1) && !is_boundary_cell(c2))
			return this->embedding(c1) == this->embedding(c2);
		else
			return same_orbit(c1, c2);
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
			uint32 result = 0u;
			foreach_cell([&result] (Cell<ORBIT>) { ++result; });
			return result;
		}
	}

	template <typename CellType>
	uint32 nb_cells() const
	{
		return nb_cells<CellType::ORBIT>();
	}

	template <Orbit ORBIT, typename MASK>
	uint32 nb_cells(const MASK& mask) const
	{
		uint32 result = 0u;
		foreach_cell([&result] (Cell<ORBIT>) { ++result; }, mask);
		return result;
	}

	template <typename CellType, typename MASK>
	uint32 nb_cells() const
	{
		return nb_cells<CellType::ORBIT, MASK>();
	}

	/**
	 * \brief return the number of boundaries of the map
	 */
	uint32 nb_boundaries() const
	{
		uint32 result = 0u;
		DartMarker m(*to_concrete());
#if defined(_MSC_VER) && _MSC_VER < 1900 // MSVC 2013 fix
		using Boundary = ConcreteMap::Boundary;
#else
		using Boundary = typename ConcreteMap::Boundary;
#endif
		foreach_dart([&m, &result, this] (Dart d)
		{
			if (!m.is_marked(d))
			{
				Boundary c(d);
				m.mark_orbit(c);
				if (this->is_boundary_cell(c)) ++result;
			}
		});
		return result;
	}

	/**
	 * \brief return the number of connected components of the map
	 */
	uint32 nb_connected_components() const
	{
		return nb_cells<ConcreteMap::ConnectedComponent::ORBIT>();
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

#pragma warning(push)
#pragma warning(disable:4717)
	template <Orbit ORBIT>
	bool is_boundary_cell(Cell<ORBIT> c) const
	{
		return to_concrete()->is_boundary_cell(c);
	}
#pragma warning(pop)

	template <Orbit ORBIT>
	bool is_incident_to_boundary(Cell<ORBIT> c) const
	{
		static_assert(!std::is_same<Cell<ORBIT>, typename ConcreteMap::Boundary>::value, "is_incident_to_boundary is not defined for cells of boundary dimension");
		bool result = false;
		to_concrete()->foreach_dart_of_orbit(c, [this, &result] (Dart d) -> bool
		{
			if (is_boundary(d)) { result = true; return false; }
			return true;
		});
		return result;
	}

	template <Orbit ORBIT>
	Dart boundary_dart(Cell<ORBIT> c) const
	{
		static_assert(!std::is_same<Cell<ORBIT>, typename ConcreteMap::Boundary>::value, "boundary_dart is not defined for boundary cells");
		Dart result;
		to_concrete()->foreach_dart_of_orbit(c, [this, &result] (Dart d) -> bool
		{
			if (is_boundary(d)) { result = d; return false; }
			return true;
		});
		return result;
	}

protected:

	template <Orbit ORBIT>
	void boundary_mark(Cell<ORBIT> c)
	{
		static_assert(std::is_same<Cell<ORBIT>, typename ConcreteMap::Boundary>::value, "Cell is not defined as boundary");
		to_concrete()->foreach_dart_of_orbit(c, [this] (Dart d)
		{
			set_boundary(d, true);
		});
	}

	template <Orbit ORBIT>
	void boundary_unmark(Cell<ORBIT> c)
	{
		static_assert(std::is_same<Cell<ORBIT>, typename ConcreteMap::Boundary>::value, "Cell is not defined as boundary");
		to_concrete()->foreach_dart_of_orbit(c, [this] (Dart d)
		{
			set_boundary(d, false);
		});
	}

	/*******************************************************************************
	 * Traversals
	 *******************************************************************************/

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

	/*!
	 * \Brief Methods to iterate over darts.
	 * These functions traverse all darts.
	 */
	inline Dart all_begin() const
	{
		return Dart(this->topology_.begin());
	}

	inline void all_next(Dart& d) const
	{
		this->topology_.next(d.index);
	}

	inline Dart all_end() const
	{
		return Dart(this->topology_.end());
	}

public:

	/**
	 * \brief apply a function on each dart of the map (including boundary darts)
	 * if the function returns a boolean, the traversal stops when it first returns false
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <typename FUNC>
	inline void foreach_dart(const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Dart>::value, "foreach_dart: given function should take a Dart as parameter");

		const ConcreteMap* cmap = to_concrete();
		for (Dart it = cmap->all_begin(), last = cmap->all_end(); it != last; cmap->all_next(it))
		{
			if (!internal::void_to_true_binder(f, it))
				break;
		}
	}

	/**
	 * \brief apply a function in parallel on each dart of the map (including boundary darts)
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <typename FUNC>
	inline void parallel_foreach_dart(const FUNC& f) const
	{
		static_assert(is_func_parameter_same<FUNC, Dart>::value, "parallel_foreach_dart: given function should take a Dart as parameter");

		using Future = std::future<typename std::result_of<FUNC(Dart)>::type>;
		using VecDarts = std::vector<Dart>;

		ThreadPool* thread_pool = cgogn::thread_pool();
		uint32 nb_workers = thread_pool->nb_workers();
		if (nb_workers == 0)
			return foreach_dart(f);

		std::array<std::vector<VecDarts*>, 2> dart_buffers;
		std::array<std::vector<Future>, 2> futures;
		dart_buffers[0].reserve(nb_workers);
		dart_buffers[1].reserve(nb_workers);
		futures[0].reserve(nb_workers);
		futures[1].reserve(nb_workers);

		Buffers<Dart>* dbuffs = cgogn::dart_buffers();
		const ConcreteMap* cmap = to_concrete();

		uint32 i = 0u; // buffer id (0/1)
		uint32 j = 0u; // thread id (0..nb_workers)
		Dart it = cmap->all_begin();
		Dart last = cmap->all_end();

		while (it != last)
		{
			dart_buffers[i].push_back(dbuffs->buffer());
			cgogn_assert(dart_buffers[i].size() <= nb_workers);
			std::vector<Dart>& darts = *dart_buffers[i].back();
			darts.reserve(PARALLEL_BUFFER_SIZE);
			for (unsigned k = 0u; k < PARALLEL_BUFFER_SIZE && it.index < last.index; ++k)
			{
				darts.push_back(it);
				cmap->all_next(it);
			}

			futures[i].push_back(thread_pool->enqueue([&darts, &f] ()
			{
				for (auto d : darts)
					f(d);
			}));

			// next thread
			if (++j == nb_workers)
			{	// again from 0 & change buffer
				j = 0;
				i = (i+1u) % 2u;
				for (auto& fu : futures[i])
					fu.wait();
				for (auto& b : dart_buffers[i])
					dbuffs->release_buffer(b);
				futures[i].clear();
				dart_buffers[i].clear();
			}
		}

		// clean all at end
		for (auto& fu : futures[0u])
			fu.wait();
		for (auto& b : dart_buffers[0u])
			dbuffs->release_buffer(b);
		for (auto& fu : futures[1u])
			fu.wait();
		for (auto& b : dart_buffers[1u])
			dbuffs->release_buffer(b);
	}

	/**
	 * \brief apply a function on each cell of the map (boundary cells excluded)
	 * the dimension of the traversed cells is determined based on the parameter of the given callable
	 * if the function returns a boolean, the traversal stops when it first returns false
	 * if the function takes a second integer parameter, it is given the cell index along with each cell
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <TraversalStrategy STRATEGY = TraversalStrategy::AUTO, typename FUNC>
	inline void foreach_cell(const FUNC& f) const
	{
		using CellType = func_parameter_type<FUNC>;

		foreach_cell<STRATEGY>(f, [] (CellType) { return true; });
	}

	/**
	 * \brief apply a function in parallel on each cell of the map (boundary cells excluded)
	 * the dimension of the traversed cells is determined based on the parameter of the given callable
	 * if the function takes a second integer parameter, it is given the cell index along with each cell
	 * @tparam FUNC type of the callable
	 * @param f a callable
	 */
	template <TraversalStrategy STRATEGY = TraversalStrategy::AUTO, typename FUNC>
	inline void parallel_foreach_cell(const FUNC& f) const
	{
		using CellType = func_parameter_type<FUNC>;

		parallel_foreach_cell<STRATEGY>(f, [] (CellType) { return true; });
	}

	/**
	 * \brief apply a function on each cell of the map (boundary cells excluded)
	 * the dimension of the traversed cells is determined based on the parameter of the given callable
	 * only cells selected by the given FilterFunction (CellType -> bool) are processed
	 * if the function returns a boolean, the traversal stops when it first returns false
	 * if the function takes a second integer parameter, it is given the cell index along with each cell
	 * @tparam FUNC type of the callable
	 * @tparam FilterFunction type of the cell filtering function (CellType -> bool)
	 * @param f a callable
	 * @param filter a cell filtering function
	 */
	template <TraversalStrategy STRATEGY = TraversalStrategy::AUTO, typename FUNC, typename FilterFunction>
	inline auto foreach_cell(const FUNC& f, const FilterFunction& filter) const
		-> typename std::enable_if<
			is_func_return_same<FilterFunction, bool>::value  &&
			is_func_parameter_same<FilterFunction, func_parameter_type<FUNC>>::value
		   >::type
	{
		using CellType = func_parameter_type<FUNC>;

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

	/**
	 * \brief apply a function in parallel on each cell of the map (boundary cells excluded)
	 * the dimension of the traversed cells is determined based on the parameter of the given callable
	 * only cells selected by the given FilterFunction (CellType -> bool) are processed
	 * if the function takes a second integer parameter, it is given the cell index along with each cell
	 * @tparam FUNC type of the callable
	 * @tparam FilterFunction type of the cell filtering function (CellType -> bool)
	 * @param f a callable
	 * @param filter a cell filtering function
	 */
	template <TraversalStrategy STRATEGY = TraversalStrategy::AUTO, typename FUNC, typename FilterFunction>
	inline auto parallel_foreach_cell(const FUNC& f, const FilterFunction& filter) const
		-> typename std::enable_if<
			is_func_return_same<FilterFunction, bool>::value &&
			is_func_parameter_same<FilterFunction, func_parameter_type<FUNC>>::value
		   >::type
	{
		using CellType = func_parameter_type<FUNC>;

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

	/**
	 * \brief apply a function on each cell of the map (boundary cells excluded)
	 * the dimension of the traversed cells is determined based on the parameter of the given callable
	 * only cells selected by the filter function of the corresponding CellType within the given Filters object are processed
	 * if the function returns a boolean, the traversal stops when it first returns false
	 * if the function takes a second integer parameter, it is given the cell index along with each cell
	 * @tparam FUNC type of the callable
	 * @tparam Filters type of the CellFilters object (inherits from CellFilters)
	 * @param f a callable
	 * @param filters a CellFilters object (contains a filtering function for each CellType)
	 */
	template <TraversalStrategy STRATEGY = TraversalStrategy::AUTO, typename FUNC, typename Filters>
	inline auto foreach_cell(const FUNC& f, const Filters& filters) const
		-> typename std::enable_if<std::is_base_of<CellFilters, Filters>::value>::type
	{
		using CellType = func_parameter_type<FUNC>;

		if ((filters.filtered_cells() & orbit_mask<CellType>()) == 0u)
			cgogn_log_warning("foreach_cell") << "Using a CellFilter for a non-filtered CellType";

		foreach_cell<STRATEGY>(f, [&filters] (CellType c) { return filters.filter(c); });
	}

	/**
	 * \brief apply a function in parallel on each cell of the map (boundary cells excluded)
	 * the dimension of the traversed cells is determined based on the parameter of the given callable
	 * only cells selected by the filter function of the corresponding CellType within the given Filters object are processed
	 * if the function takes a second integer parameter, it is given the cell index along with each cell
	 * @tparam FUNC type of the callable
	 * @tparam Filters type of the CellFilters object (inherits from CellFilters)
	 * @param f a callable
	 * @param filters a CellFilters object (contains a filtering function for each CellType)
	 */
	template <TraversalStrategy STRATEGY = TraversalStrategy::AUTO, typename FUNC, typename Filters>
	inline auto parallel_foreach_cell(const FUNC& f, const Filters& filters) const
		-> typename std::enable_if<std::is_base_of<CellFilters, Filters>::value>::type
	{
		using CellType = func_parameter_type<FUNC>;

		if ((filters.filtered_cells() & orbit_mask<CellType>()) == 0u)
			cgogn_log_warning("foreach_cell") << "Using a CellFilter for a non-filtered CellType";

		parallel_foreach_cell<STRATEGY>(f, [&filters] (CellType c) { return filters.filter(c); });
	}

	/**
	 * \brief apply a function on each cell of the map that is provided by the given Traversor object
	 * the dimension of the traversed cells is determined based on the parameter of the given callable
	 * if the function returns a boolean, the traversal stops when it first returns false
	 * @tparam FUNC type of the callable
	 * @tparam Traversor type of the CellTraversor object (inherits from CellTraversor)
	 * @param f a callable
	 * @param t a Traversor object
	 */
	template <typename FUNC, typename Traversor>
	inline auto foreach_cell(const FUNC& f, const Traversor& t) const
		-> typename std::enable_if<std::is_base_of<CellTraversor, Traversor>::value>::type
	{
		using CellType = func_parameter_type<FUNC>;

		if (!t.template is_traversed<CellType>())
			cgogn_log_warning("foreach_cell") << "Using a CellTraversor for a non-traversed CellType";

		for (typename Traversor::const_iterator it = t.template begin<CellType>(), end = t.template end<CellType>(); it != end; ++it)
			if (!internal::void_to_true_binder(f, CellType(*it)))
				break;
	}

	/**
	 * \brief apply a function in parallel on each cell of the map that is provided by the given Traversor object
	 * the dimension of the traversed cells is determined based on the parameter of the given callable
	 * @tparam FUNC type of the callable
	 * @tparam Traversor type of the CellTraversor object (inherits from CellTraversor)
	 * @param f a callable
	 * @param t a Traversor object
	 */
	template <typename FUNC, typename Traversor>
	inline auto parallel_foreach_cell(const FUNC& f, const Traversor& t) const
		-> typename std::enable_if<std::is_base_of<CellTraversor, Traversor>::value>::type
	{
		using CellType = func_parameter_type<FUNC>;

		using VecCell = std::vector<CellType>;
		using Future = std::future<typename std::result_of<FUNC(CellType)>::type>;

		if (!t.template is_traversed<CellType>())
			cgogn_log_warning("foreach_cell") << "Using a CellTraversor for a non-traversed CellType";

		ThreadPool* thread_pool = cgogn::thread_pool();
		uint32 nb_workers = thread_pool->nb_workers();
		if (nb_workers == 0)
			return foreach_cell(f, t);

		std::array<std::vector<VecCell*>, 2> cells_buffers;
		std::array<std::vector<Future>, 2> futures;
		cells_buffers[0].reserve(nb_workers);
		cells_buffers[1].reserve(nb_workers);
		futures[0].reserve(nb_workers);
		futures[1].reserve(nb_workers);

		Buffers<Dart>* dbuffs = cgogn::dart_buffers();

		uint32 i = 0u; // buffer id (0/1)
		uint32 j = 0u; // thread id (0..nb_workers)
		auto it = t.template begin<CellType>();
		const auto it_end = t.template end<CellType>();
		while(it != it_end)
		{
			// fill buffer
			cells_buffers[i].push_back(dbuffs->template cell_buffer<CellType>());
			VecCell& cells = *cells_buffers[i].back();
			cells.reserve(PARALLEL_BUFFER_SIZE);
			for (unsigned k = 0u; k < PARALLEL_BUFFER_SIZE && (it != it_end); ++k)
			{
				cells.push_back(CellType(*it));
				++it;
			}
			// launch thread
			futures[i].push_back(thread_pool->enqueue([&cells, &f] ()
			{
				for (auto c : cells)
					f(c);
			}));
			// next thread
			if (++j == nb_workers)
			{	// again from 0 & change buffer
				j = 0;
				i = (i+1u) % 2u;
				for (auto& fu : futures[i])
					fu.wait();
				for (auto& b : cells_buffers[i])
					dbuffs->release_cell_buffer(b);
				futures[i].clear();
				cells_buffers[i].clear();
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

protected:

	/**
	 * \brief apply a function on each cell of the map (boundary cells excluded) using a DartMarker
	 * the dimension of the traversed cells is determined based on the parameter of the given callable
	 * only cells selected by the given FilterFunction (CellType -> bool) are processed
	 * if the function returns a boolean, the traversal stops when it first returns false
	 * @tparam FUNC type of the callable
	 * @tparam FilterFunction type of the cell filtering function (CellType -> bool)
	 * @param f a callable
	 * @param filter a cell filtering function
	 */
	template <typename FUNC, typename FilterFunction>
	inline void foreach_cell_dart_marking(const FUNC& f, const FilterFunction& filter) const
	{
		using CellType = func_parameter_type<FUNC>;

		const ConcreteMap* cmap = to_concrete();
		DartMarker dm(*cmap);
		for (Dart it = cmap->begin(), last = cmap->end(); it.index < last.index; cmap->next(it))
		{
			if (!dm.is_marked(it))
			{
				const CellType c(it);
				dm.mark_orbit(c);
				if (filter(c) && !internal::void_to_true_binder(f, c))
					break;
			}
		}
	}

	/**
	 * \brief apply a function on each cell of the map (boundary cells excluded) using a DartMarker
	 * the dimension of the traversed cells is determined based on the parameter of the given callable
	 * only cells selected by the given FilterFunction (CellType -> bool) are processed
	 * @tparam FUNC type of the callable
	 * @tparam FilterFunction type of the cell filtering function (CellType -> bool)
	 * @param f a callable
	 * @param filter a cell filtering function
	 */
	template <typename FUNC, typename FilterFunction>
	inline void parallel_foreach_cell_dart_marking(const FUNC& f, const FilterFunction& filter) const
	{
		using CellType = func_parameter_type<FUNC>;

		using VecCell = std::vector<CellType>;
		using Future = std::future<typename std::result_of<FUNC(CellType)>::type>;

		ThreadPool* thread_pool = cgogn::thread_pool();
		uint32 nb_workers = thread_pool->nb_workers();
		if (nb_workers == 0)
			return foreach_cell_dart_marking(f, filter);

		std::array<std::vector<VecCell*>, 2> cells_buffers;
		std::array<std::vector<Future>, 2> futures;
		cells_buffers[0].reserve(nb_workers);
		cells_buffers[1].reserve(nb_workers);
		futures[0].reserve(nb_workers);
		futures[1].reserve(nb_workers);

		Buffers<Dart>* dbuffs = cgogn::dart_buffers();

		const ConcreteMap* cmap = to_concrete();
		DartMarker dm(*cmap);
		Dart it = cmap->begin();
		Dart last = cmap->end();

		uint32 i = 0u; // buffer id (0/1)
		uint32 j = 0u; // thread id (0..nb_workers)
		while (it.index < last.index)
		{
			// fill buffer
			cells_buffers[i].push_back(dbuffs->template cell_buffer<CellType>());
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
			futures[i].push_back(thread_pool->enqueue([&cells, &f] ()
			{
				for (auto c : cells)
					f(c);
			}));
			// next thread
			if (++j == nb_workers)
			{	// again from 0 & change buffer
				j = 0;
				i = (i+1u) % 2u;
				for (auto& fu : futures[i])
					fu.wait();
				for (auto& b : cells_buffers[i])
					dbuffs->release_cell_buffer(b);
				futures[i].clear();
				cells_buffers[i].clear();
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

	/**
	 * \brief apply a function on each cell of the map (boundary cells excluded) using a CellMarker
	 * the dimension of the traversed cells is determined based on the parameter of the given callable
	 * only cells selected by the given FilterFunction (CellType -> bool) are processed
	 * if the function returns a boolean, the traversal stops when it first returns false
	 * @tparam FUNC type of the callable
	 * @tparam FilterFunction type of the cell filtering function (CellType -> bool)
	 * @param f a callable
	 * @param filter a cell filtering function
	 */
	template <typename FUNC, typename FilterFunction>
	inline void foreach_cell_cell_marking(const FUNC& f, const FilterFunction& filter) const
	{
		using CellType = func_parameter_type<FUNC>;

		const ConcreteMap* cmap = to_concrete();
		CellMarker<CellType::ORBIT> cm(*cmap);
		for (Dart it = cmap->begin(), last = cmap->end(); it.index < last.index; cmap->next(it))
		{
			const CellType c(it);
			if (!cm.is_marked(c))
			{
				cm.mark(c);
				if (filter(c) && !internal::void_to_true_binder(f, c))
					break;
			}
		}
	}

	/**
	 * \brief apply a function in parallel on each cell of the map (boundary cells excluded) using a CellMarker
	 * the dimension of the traversed cells is determined based on the parameter of the given callable
	 * only cells selected by the given FilterFunction (CellType -> bool) are processed
	 * @tparam FUNC type of the callable
	 * @tparam FilterFunction type of the cell filtering function (CellType -> bool)
	 * @param f a callable
	 * @param filter a cell filtering function
	 */
	template <typename FUNC, typename FilterFunction>
	inline void parallel_foreach_cell_cell_marking(const FUNC& f, const FilterFunction& filter) const
	{
		using CellType = func_parameter_type<FUNC>;
		static const Orbit ORBIT = CellType::ORBIT;

		using VecCell = std::vector<CellType>;
		using Future = std::future<typename std::result_of<FUNC(CellType)>::type>;

		ThreadPool* thread_pool = cgogn::thread_pool();
		uint32 nb_workers = thread_pool->nb_workers();
		if (nb_workers == 0)
			return foreach_cell_cell_marking(f, filter);

		std::array<std::vector<VecCell*>, 2> cells_buffers;
		std::array<std::vector<Future>, 2> futures;
		cells_buffers[0].reserve(nb_workers);
		cells_buffers[1].reserve(nb_workers);
		futures[0].reserve(nb_workers);
		futures[1].reserve(nb_workers);

		Buffers<Dart>* dbuffs = cgogn::dart_buffers();

		const ConcreteMap* cmap = to_concrete();
		CellMarker<ORBIT> cm(*cmap);
		Dart it = cmap->begin();
		Dart last = cmap->end();

		uint32 i = 0u; // buffer id (0/1)
		uint32 j = 0u; // thread id (0..nb_workers)
		while (it.index < last.index)
		{
			// fill buffer
			cells_buffers[i].push_back(dbuffs->template cell_buffer<CellType>());
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
			futures[i].push_back(thread_pool->enqueue([&cells, &f] ()
			{
				for (auto c : cells)
					f(c);
			}));
			// next thread
			if (++j == nb_workers)
			{	// again from 0 & change buffer
				j = 0;
				i = (i+1u) % 2u;
				for (auto& fu : futures[i])
					fu.wait();
				for (auto& b : cells_buffers[i])
					dbuffs->release_cell_buffer(b);
				futures[i].clear();
				cells_buffers[i].clear();
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

public:

	/*******************************************************************************
	 * compacting
	 *******************************************************************************/
	/**
	 * @brief compact an embedding orbit
	 * @param orbit to compact
	 */
	void compact_embedding(uint32 orbit)
	{
		ChunkArray<uint32>* embedding = this->embeddings_[orbit];
		if (embedding != nullptr)
		{
			std::vector<uint32> old_new = this->attributes_[orbit].template compact<1>();
			if (!old_new.empty())
			{
				for (uint32 i=this->topology_.begin(); i!= this->topology_.end(); this->topology_.next(i))
				{
					uint32& emb = (*embedding)[i];
					if ((emb != std::numeric_limits<uint32>::max())
						&& (old_new[emb] != std::numeric_limits<uint32>::max()))
						emb = old_new[emb];
				}
			}
		}
	}

	void compact_topo()
	{
		std::vector<uint32> old_new = this->topology_.template compact<ConcreteMap::PRIM_SIZE>();

		if (old_new.empty())
			return;			// already compact nothing to do with relationss

		for (ChunkArrayGen* ptr: this->topology_.chunk_arrays())
		{
			ChunkArray<Dart>* ca = dynamic_cast<ChunkArray<Dart>*>(ptr);
			if (ca)
			{
				for (uint32 i=this->topology_.begin(); i!= this->topology_.end(); this->topology_.next(i))
				{
					Dart& d = (*ca)[i];
					uint32 idx = d.index;
					if (old_new[idx] != std::numeric_limits<uint32>::max())
						d = Dart(old_new[idx]);
				}
			}
		}
	}

	/**
	 * @brief compact this map
	 */
	void compact()
	{
		compact_topo();
		for (uint32 orbit = 0; orbit < NB_ORBITS; ++orbit)
			compact_embedding(orbit); // checking if embedding used done inside
	}

	/**
	 * @brief merge map in this map
	 * @param map must be of same type than map
	 * @param newdarts a DartMarker in which the new imported darts are marked
	 * @return false if the merge can not be done (incompatible attributes), true otherwise
	 */
	bool merge(const ConcreteMap& map, DartMarker& newdarts)
	{
		// check attributes compatibility
		for(uint32 i = 0; i < NB_ORBITS; ++i)
		{
			if (this->embeddings_[i] != nullptr)
			{
				if (!this->attributes_[i].check_before_merge(map.attributes_[i]))
					return false;
			}
		}

		// compact topology container
		this->compact_topo();
		uint32 first = this->topology_.size();

		// ensure that orbits that are embedded in given map are also embedded in this map
		ConcreteMap* concrete = to_concrete();
		concrete->merge_check_embedding(map);

		// store index of copied darts
		std::vector<uint32> old_new_topo = this->topology_.template merge<ConcreteMap::PRIM_SIZE>(map.topology_);

		// mark new darts with the given dartmarker
		newdarts.unmark_all();
		map.foreach_dart([&] (Dart d) { newdarts.mark(Dart(old_new_topo[d.index])); });

		// change topo relations of copied darts
		for (ChunkArrayGen* ptr : this->topology_.chunk_arrays())
		{
			ChunkArray<Dart>* cad = dynamic_cast<ChunkArray<Dart>*>(ptr);
			if (cad)
			{
				for (uint32 i = first; i != this->topology_.end(); this->topology_.next(i))
				{
					Dart& d = (*cad)[i];
					uint32 idx = d.index;
					if (old_new_topo[idx] != INVALID_INDEX)
						d = Dart(old_new_topo[idx]);
				}
			}
		}

		// set boundary of copied darts
		map.foreach_dart([&] (Dart d)
		{
			if (map.is_boundary(d))
				this->set_boundary(Dart(old_new_topo[d.index]), true);
		});

		// change embedding indices of moved lines
		for(uint32 i = 0; i < NB_ORBITS; ++i)
		{
			ChunkArray<uint32>* emb = this->embeddings_[i];
			if (emb != nullptr)
			{
				if (map.embeddings_[i] == nullptr) // set embedding to INVALID for further easy detection
				{
					for (uint32 j = first; j != this->topology_.end(); this->topology_.next(j))
						(*emb)[j] = INVALID_INDEX;
				}
				else
				{
					std::vector<uint32> old_new = this->attributes_[i].template merge<1>(map.attributes_[i]);
					for (uint32 j = first; j != this->topology_.end(); this->topology_.next(j))
					{
						uint32& e = (*emb)[j];
						if (e != INVALID_INDEX)
						{
							if (old_new[e] != INVALID_INDEX)
								e = old_new[e];
						}
					}
				}
			}
		}

		// embed remaining cells
		concrete->merge_finish_embedding(first);

		// ok
		return true;
	}
};

} // namespace cgogn

#endif // CGOGN_CORE_CMAP_MAP_BASE_H_

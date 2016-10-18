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

#ifndef CGOGN_CORE_CMAP_CMAP0_H_
#define CGOGN_CORE_CMAP_CMAP0_H_

#include <cgogn/core/cmap/map_base.h>

namespace cgogn
{

template <typename MAP_TYPE>
class CMap0_T : public MapBase<MAP_TYPE>
{
public:

	static const uint8 DIMENSION = 0;

	static const uint8 PRIM_SIZE = 1;

	using MapType = MAP_TYPE;
	using Inherit = MapBase<MAP_TYPE>;
	using Self = CMap0_T<MAP_TYPE>;

	friend class MapBase<MAP_TYPE>;
	friend class DartMarker_T<Self>;
	friend class cgogn::DartMarkerStore<Self>;

	using Vertex = Cell<Orbit::DART>;

	using Boundary = Vertex;  // just for compilation
	using ConnectedComponent = Vertex;

	template <typename T>
	using ChunkArrayContainer = typename Inherit::template ChunkArrayContainer<T>;
	using typename Inherit::ChunkArrayGen;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	template <typename T>
	using VertexAttribute = Attribute<T, Vertex::ORBIT>;

	using DartMarker = typename cgogn::DartMarker<Self>;
	using DartMarkerStore = typename cgogn::DartMarkerStore<Self>;

	template <Orbit ORBIT>
	using CellMarker = typename cgogn::CellMarker<Self, ORBIT>;
	template <Orbit ORBIT>
	using CellMarkerNoUnmark = typename cgogn::CellMarkerNoUnmark<Self, ORBIT>;
	template <Orbit ORBIT>
	using CellMarkerStore = typename cgogn::CellMarkerStore<Self, ORBIT>;

public:

	CMap0_T() : Inherit()
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap0_T);

	~CMap0_T() override
	{}

	/*!
	 * \brief Check the integrity of embedding information
	 */
	inline bool check_embedding_integrity()
	{
		bool result = true;

		if (this->template is_embedded<Vertex>())
			result = result && this->template is_well_embedded<Vertex>();

		return result;
	}

	/*******************************************************************************
	 * Low-level topological operations
	 *******************************************************************************/

protected:

	/*!
	* \brief Init an newly added dart.
	*/
	inline void init_dart(Dart)
	{
	}

	/**
	 * @brief Check the integrity of a dart
	 * @param d the dart to check
	 * @return true if the integrity constraints are locally statisfied
	 * No contraints.
	 */
	inline bool check_integrity(Dart) const
	{
		return true;
	}

	/**
	 * @brief Check the integrity of a boundary dart
	 * @param d the dart to check
	 * @return true if the bondary constraints are locally statisfied
	 * No boundary dart is accepted.
	 */
	inline bool check_boundary_integrity(Dart d) const
	{
		return !this->is_boundary(d);
	}

	/*******************************************************************************
	 * High-level embedded and topological operations
	 *******************************************************************************/

public:

	/*!
	 * \brief Add an embedded vertex (or dart) in the map.
	 * \return The added dart. If the map has DART attributes,
	 * the inserted darts are automatically embedded on new attribute elements.
	 */
	inline Vertex add_vertex()
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		const Vertex v(this->add_topology_element());

		if (this->template is_embedded<Vertex>())
			this->new_orbit_embedding(v);

		return v;
	}

	/*!
	 * \brief Remove a vertex (or dart) from the map.
	 */
	inline void remove_vertex(Vertex v)
	{
		CGOGN_CHECK_CONCRETE_TYPE;

		this->remove_topology_element(v.dart);
	}

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(ORBIT == Orbit::DART, "Orbit not supported in a CMap0");
		f(c.dart);
	}

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit_until(Cell<ORBIT> c, const FUNC& f) const
	{
		static_assert(ORBIT == Orbit::DART, "Orbit not supported in a CMap0");
		f(c.dart);
	}

protected:

	/**
	 * @brief check if embedding of map is also embedded in this (create if not). Used by merge method
	 * @param map
	 */
	void merge_check_embedding(const Self& map)
	{
		if (!this->template is_embedded<Orbit::DART>() && map.template is_embedded<Orbit::DART>())
			this->template create_embedding<Orbit::DART>();
	}

	/**
	 * @brief ensure all cells (introduced while merging) are embedded.
	 * @param first index of first dart to scan
	 */
	void merge_finish_embedding(uint32 first)
	{
		if (this->template is_embedded<Orbit::DART>())
		{
			for (uint32 j = first; j != this->topology_.end(); this->topology_.next(j))
			{
				if ((*this->embeddings_[Orbit::DART])[j] == INVALID_INDEX)
					this->new_orbit_embedding(Cell<Orbit::DART>(Dart(j)));
			}
		}
	}
};

struct CMap0Type
{
	using TYPE = CMap0_T<CMap0Type>;
};

using CMap0 = CMap0_T<CMap0Type>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP0_CPP_))
extern template class CGOGN_CORE_API DartMarker<CMap0>;
extern template class CGOGN_CORE_API DartMarkerStore<CMap0>;
extern template class CGOGN_CORE_API DartMarkerNoUnmark<CMap0>;
extern template class CGOGN_CORE_API CellMarker<CMap0, CMap0::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerNoUnmark<CMap0, CMap0::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap0, CMap0::Vertex::ORBIT>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP0_CPP_))

} // namespace cgogn

#endif // CGOGN_CORE_CMAP_CMAP0_H_

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

#ifndef CORE_CMAP_CMAP0_H_
#define CORE_CMAP_CMAP0_H_

#include <core/cmap/map_base.h>

namespace cgogn
{

template <typename MAP_TRAITS, typename MAP_TYPE>
class CMap0_T : public MapBase<MAP_TRAITS, MAP_TYPE>
{
public:

	static const int32 PRIM_SIZE = 1;

	using MapTraits = MAP_TRAITS;
	using MapType = MAP_TYPE;
	using Inherit = MapBase<MAP_TRAITS, MAP_TYPE>;
	using Self = CMap0_T<MAP_TRAITS, MAP_TYPE>;

	friend class MapBase<MAP_TRAITS, MAP_TYPE>;
	friend class DartMarker_T<Self>;
	friend class cgogn::DartMarkerStore<Self>;

	using Vertex = Cell<Orbit::DART>;

	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;
	template <typename T>
	using ChunkArrayContainer = typename Inherit::template ChunkArrayContainer<T>;

	template <typename T, Orbit ORBIT>
	using AttributeHandler = typename Inherit::template AttributeHandler<T, ORBIT>;
	template <typename T>
	using VertexAttributeHandler = AttributeHandler<T, Vertex::ORBIT>;

	using DartMarker = typename cgogn::DartMarker<Self>;
	using DartMarkerStore = typename cgogn::DartMarkerStore<Self>;

	template <Orbit ORBIT>
	using CellMarker = typename cgogn::CellMarker<Self, ORBIT>;

public:

	CMap0_T() : Inherit()
	{}

	~CMap0_T() override
	{}

	CMap0_T(Self const&) = delete;
	CMap0_T(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

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

	inline bool check_integrity(Dart) const
	{
		return true;
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

		const Vertex v(this->add_dart());

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

		this->remove_dart(v.dart);
	}

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

protected:

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
};

template <typename MAP_TRAITS>
struct CMap0Type
{
	using TYPE = CMap0_T<MAP_TRAITS, CMap0Type<MAP_TRAITS>>;
};

template <typename MAP_TRAITS>
using CMap0 = CMap0_T<MAP_TRAITS, CMap0Type<MAP_TRAITS>>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP0_CPP_))
extern template class CGOGN_CORE_API CMap0_T<DefaultMapTraits, CMap0Type<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarker<CMap0<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerStore<CMap0<DefaultMapTraits>>;
extern template class CGOGN_CORE_API DartMarkerNoUnmark<CMap0<DefaultMapTraits>>;
extern template class CGOGN_CORE_API CellMarker<CMap0<DefaultMapTraits>, CMap0<DefaultMapTraits>::Vertex::ORBIT>;
extern template class CGOGN_CORE_API CellMarkerStore<CMap0<DefaultMapTraits>, CMap0<DefaultMapTraits>::Vertex::ORBIT>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP0_CPP_))

} // namespace cgogn

#endif // CORE_CMAP_CMAP0_H_

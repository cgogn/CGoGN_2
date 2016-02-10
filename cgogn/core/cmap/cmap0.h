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
#include <core/basic/dart.h>

namespace cgogn
{

template <typename MAP_TRAITS, typename MAP_TYPE>
class CMap0_T : public MapBase<MAP_TRAITS, MAP_TYPE>
{
public:

	static const int PRIM_SIZE = 1;

	typedef MAP_TRAITS MapTraits;
	typedef MAP_TYPE MapType;
	typedef MapBase<MAP_TRAITS, MAP_TYPE> Inherit;
	typedef CMap0_T<MAP_TRAITS, MAP_TYPE> Self;

	friend typename Self::Inherit;
	friend class DartMarker_T<Self>;

	static const Orbit VERTEX = Orbit::DART;

	typedef Cell<Self::VERTEX> Vertex;

	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;
	template <typename T>
	using ChunkArrayContainer = typename Inherit::template ChunkArrayContainer<T>;

	template <typename T, Orbit ORBIT>
	using AttributeHandler = typename Inherit::template AttributeHandler<T, ORBIT>;
	template <typename T>
	using VertexAttributeHandler = AttributeHandler<T, Self::VERTEX>;

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

protected:
	inline Dart add_dart()
	{
		CGOGN_CHECK_CONCRETE_TYPE;
		return Dart(this->add_topology_element());
	}

public:
	Vertex add_vertex()
	{
		const Dart d = this->to_concrete()->add_dart();

		if (this->template is_orbit_embedded<DART>()) {
			unsigned int idx = this->template add_attribute_element<DART>();
			this->template set_embedding<DART>(d, idx);
		}
		return Vertex(d);
	}

protected:

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(Cell<ORBIT> c, const FUNC& f) const
	{
		switch(ORBIT)
		{
			case Orbit::DART: f(c.dart); break;
			case Orbit::PHI1:
			case Orbit::PHI2:
			case Orbit::PHI1_PHI2:
			case Orbit::PHI1_PHI3:
			case Orbit::PHI2_PHI3:
			case Orbit::PHI21:
			case Orbit::PHI21_PHI31:
			default: cgogn_assert_not_reached("Cells of this dimension are not handled"); break;
		}
	}
};

template <typename MAP_TRAITS>
struct CMap0Type
{
	typedef CMap0_T<MAP_TRAITS, CMap0Type<MAP_TRAITS>> TYPE;
};

template <typename MAP_TRAITS>
using CMap0 = CMap0_T<MAP_TRAITS, CMap0Type<MAP_TRAITS>>;

} // namespace cgogn

#endif // CORE_CMAP_CMAP0_H_

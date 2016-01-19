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

	using VertexHandle = CellHandle<Self::VERTEX>;

	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;
	template<typename T>
	using ChunkArrayContainer = typename Inherit::template ChunkArrayContainer<T>;

	template<typename T, Orbit ORBIT>
	using AttributeHandler = typename Inherit::template AttributeHandler<T, ORBIT>;
	template<typename T>
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
		unsigned int di = this->add_topology_element();
		Dart d(di);
		return d;
	}

public:
	VertexHandle add_vertex()
	{
		Dart d = this->to_concrete()->add_dart();
		VertexHandle v(d);

		if (this->template is_orbit_embedded<Orbit::DART>())
			init_orbit_embedding<Orbit::DART>(d, this->template add_attribute_element<Orbit::DART>());

		return v;
	}

protected:

	/*******************************************************************************
	 * Orbits traversal
	 *******************************************************************************/

	template <Orbit ORBIT, typename FUNC>
	inline void foreach_dart_of_orbit(CellHandle<ORBIT> c, const FUNC& f) const
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

protected:

	/*******************************************************************************
	 * Embedding management
	 *******************************************************************************/

	template <Orbit ORBIT>
	inline void init_orbit_embedding(CellHandle<ORBIT> c, unsigned int emb)
	{
		foreach_dart_of_orbit(c, [this, emb] (Dart d) { this->template init_embedding<ORBIT>(d, emb); });
	}

	template <Orbit ORBIT>
	inline void set_orbit_embedding(CellHandle<ORBIT> c, unsigned int emb)
	{
		foreach_dart_of_orbit(c, [this, emb] (Dart d) { this->template set_embedding<ORBIT>(d, emb); });
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

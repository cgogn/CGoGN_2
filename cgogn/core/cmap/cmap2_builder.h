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

#ifndef CORE_MAP_MAP2_BUILDER_H_
#define CORE_MAP_MAP2_BUILDER_H_

#include <core/cmap/cmap2.h>

namespace cgogn
{

template <typename MAP_TRAITS>
class CMap2Builder_T
{
public:

	using Self = CMap2Builder_T<MAP_TRAITS>;
	using CMap2 = cgogn::CMap2<MAP_TRAITS>;

	template <typename T>
	using ChunkArrayContainer = typename CMap2::template ChunkArrayContainer<T>;

	inline CMap2Builder_T(CMap2& map) : map_(map)
	{}
	CMap2Builder_T(const Self&) = delete;
	CMap2Builder_T(Self&&) = delete;
	Self& operator=(const Self&) = delete;
	Self& operator=(Self&&) = delete;
	inline ~CMap2Builder_T() = default;

public:

	template <Orbit ORBIT>
	inline void create_embedding()
	{
		map_.template create_embedding<ORBIT>();
	}

	template <Orbit ORBIT, typename T>
	inline void swap_chunk_array_container(ChunkArrayContainer<T> &cac)
	{
		map_.attributes_[ORBIT].swap(cac);
	}

	template <class CellType>
	inline void set_embedding(Dart d, unsigned int emb)
	{
		map_.template set_embedding<CellType>(d, emb);
	}

	template <class CellType>
	inline void new_orbit_embedding(CellType c)
	{
		map_.template new_orbit_embedding(c);
	}

	inline void phi2_sew(Dart d, Dart e)
	{
		return map_.phi2_sew(d,e);
	}

	inline void phi2_unsew(Dart d)
	{
		map_.phi2_unsew(d);
	}

	inline Dart add_face_topo_parent(unsigned int nb_edges)
	{
		return map_.CMap2::Inherit::add_face_topo(nb_edges);
	}

	inline void close_hole_topo(Dart d)
	{
		map_.close_hole_topo(d);
	}

	inline void close_map()
	{
		map_.close_map();
	}

private:

	CMap2& map_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP2_BUILDER_CPP_))
extern template class CGOGN_CORE_API cgogn::CMap2Builder_T<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_MAP_MAP2_BUILDER_CPP_))
using CMap2Builder = cgogn::CMap2Builder_T<DefaultMapTraits>;

} // namespace cgogn


#endif // CORE_MAP_MAP2_BUILDER_H_


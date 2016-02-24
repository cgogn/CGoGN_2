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

#ifndef CORE_CMAP_CMAP3_BUILDER_H_
#define CORE_CMAP_CMAP3_BUILDER_H_

#include <core/cmap/cmap3.h>

namespace cgogn
{

template <typename MAP_TRAITS>
class CMap3Builder_T
{
public:

	using Self = CMap3Builder_T<MAP_TRAITS>;
	using CMap3 = cgogn::CMap3<MAP_TRAITS>;
	using Vertex = typename CMap3::Vertex;

	template <typename T>
	using ChunkArrayContainer = typename CMap3::template ChunkArrayContainer<T>;

	inline CMap3Builder_T(CMap3& map) : map_(map)
	{}
	CMap3Builder_T(const Self&) = delete;
	CMap3Builder_T(Self&&) = delete;
	Self& operator=(const Self&) = delete;
	Self& operator=(Self&&) = delete;
	inline ~CMap3Builder_T() = default;

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

	inline void init_parent_vertex_embedding(Dart d, unsigned int emb)
	{
		map_.foreach_dart_of_PHI21(d, [&] (Dart dit)
		{
			map_.set_embedding<Orbit::PHI21>(dit, emb);
		});
	}

	inline void phi2_sew(Dart d, Dart e)
	{
		return map_.phi2_sew(d,e);
	}

	inline void phi2_unsew(Dart d)
	{
		map_.phi2_unsew(d);
	}

	inline void phi3_sew(Dart d, Dart e)
	{
		return map_.phi3_sew(d,e);
	}

	inline void phi3_unsew(Dart d)
	{
		return map_.phi3_unsew(d);
	}

	inline Dart add_face_topo(unsigned int nb_edges)
	{
		return map_.add_face_topo(nb_edges);
	}

	inline Dart add_prism_topo(unsigned int nb_edges)
	{
		return map_.add_prism_topo(nb_edges);
	}

	inline Dart add_pyramid_topo(unsigned int nb_edges)
	{
		return map_.add_pyramid_topo(nb_edges);
	}

	inline unsigned int close_map()
	{
		return map_.close_map();
	}

private:

	CMap3& map_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_CMAP_CMAP3_BUILDER_CPP_))
extern template class CGOGN_CORE_API cgogn::CMap3Builder_T<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CORE_CMAP_CMAP3_BUILDER_CPP_))
using CMap3Builder = cgogn::CMap3Builder_T<DefaultMapTraits>;

} // namespace cgogn


#endif // CORE_CMAP_CMAP3_BUILDER_H_


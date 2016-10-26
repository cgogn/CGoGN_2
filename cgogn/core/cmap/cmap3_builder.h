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

#ifndef CGOGN_CORE_CMAP_CMAP3_BUILDER_H_
#define CGOGN_CORE_CMAP_CMAP3_BUILDER_H_

#include <cgogn/core/cmap/map_base.h>

namespace cgogn
{

template <typename MAP3>
class CMap3Builder_T
{
	static_assert(MAP3::DIMENSION == 3, "CMap3Builder_T works only with 3D Maps.");

public:

	using Self = CMap3Builder_T<MAP3>;
	using Map3 = MAP3;
	using CDart = typename Map3::CDart;
	using Vertex = typename Map3::Vertex;
	using Vertex2 = typename Map3::Vertex2;
	using Edge = typename Map3::Edge;
	using Edge2 = typename Map3::Edge2;
	using Face = typename Map3::Face;
	using Face2 = typename Map3::Face2;
	using Volume = typename Map3::Volume;

	using DartMarkerStore = typename Map3::DartMarkerStore;
	template <typename T>
	using ChunkArrayContainer = typename Map3::template ChunkArrayContainer<T>;

	inline CMap3Builder_T(Map3& map) : map_(map)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap3Builder_T);

	inline ~CMap3Builder_T()
	{}

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

	inline void phi3_sew(Dart d, Dart e)
	{
		return map_.phi3_sew(d,e);
	}

	inline void phi3_unsew(Dart d)
	{
		return map_.phi3_unsew(d);
	}

	inline Dart add_prism_topo_fp(std::size_t size)
	{
		return map_.add_prism_topo_fp(size);
	}

	inline Dart add_pyramid_topo_fp(std::size_t size)
	{
		return map_.add_pyramid_topo_fp(size);
	}

	inline Dart add_stamp_volume_topo_fp()
	{
		return map_.add_stamp_volume_topo_fp();
	}

	template <class CellType>
	inline void set_embedding(Dart d, uint32 emb)
	{
		map_.template set_embedding<CellType>(d, emb);
	}

	template <class CellType, Orbit ORBIT>
	inline void set_orbit_embedding(Cell<ORBIT> c, uint32 emb)
	{
		map_.template set_orbit_embedding<CellType>(c, emb);
	}

	template <class CellType>
	inline void new_orbit_embedding(CellType c)
	{
		map_.new_orbit_embedding(c);
	}

	inline void sew_volumes_fp(Dart v1, Dart v2)
	{
		map_.sew_volumes_fp(v1, v2);
	}

	template <Orbit ORBIT>
	inline void boundary_mark(Cell<ORBIT> c)
	{
		map_.boundary_mark(c);
	}

	template <Orbit ORBIT>
	void boundary_unmark(Cell<ORBIT> c)
	{
		map_.boundary_unmark(c);
	}

	inline uint32 close_map()
	{
		return map_.close_map();
	}

private:

	Map3& map_;
};

} // namespace cgogn

#endif // CGOGN_CORE_CMAP_CMAP3_BUILDER_H_

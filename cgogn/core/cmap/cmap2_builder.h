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

#ifndef CGOGN_CORE_CMAP_CMAP2_BUILDER_H_
#define CGOGN_CORE_CMAP_CMAP2_BUILDER_H_

#include <cgogn/core/cmap/map_base.h>

namespace cgogn
{

template <typename MAP2>
class CMap2Builder_T
{
	static_assert(MAP2::DIMENSION == 2, "CMap2Builder_T works only with 2D Maps.");

public:

	using Self = CMap2Builder_T<MAP2>;
	using Map2 = MAP2;
	using CDart = typename Map2::CDart;
	using Vertex = typename Map2::Vertex;
	using Edge = typename Map2::Edge;
	using Face = typename Map2::Face;
	using Volume = typename Map2::Volume;

	template <typename T>
	using ChunkArrayContainer = typename Map2::template ChunkArrayContainer<T>;

	template <typename T>
	using ChunkArray = typename Map2::template ChunkArray<T>;

	inline CMap2Builder_T(Map2& map) : map_(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap2Builder_T);
	inline ~CMap2Builder_T() {}

public:

	template <Orbit ORBIT>
	inline void create_embedding()
	{
		map_.template create_embedding<ORBIT>();
	}

	template <Orbit ORBIT>
	inline ChunkArrayContainer<uint32>& attribute_container()
	{
		return map_.template non_const_attribute_container<ORBIT>();
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
	inline uint32 new_orbit_embedding(CellType c)
	{
		return map_.new_orbit_embedding(c);
	}

	inline void phi2_sew(Dart d, Dart e)
	{
		return map_.phi2_sew(d,e);
	}

	inline void phi2_unsew(Dart d)
	{
		map_.phi2_unsew(d);
	}

	inline void remove_face_topo_fp(Dart d)
	{
		map_.remove_face_topo_fp(d);
	}

	inline Dart add_face_topo_fp(uint32 nb_edges)
	{
		return map_.add_face_topo_fp(nb_edges);
	}

	inline Dart collapse_edge_topo(Dart d)
	{
		return map_.collapse_edge_topo(d);
	}

	inline Dart close_hole_topo(Dart d)
	{
		return map_.close_hole_topo(d);
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

	inline Face close_hole(Dart d)
	{
		return map_.close_hole(d);
	}

	inline uint32 close_map()
	{
		return map_.close_map();
	}

	inline Dart add_topology_element()
	{
		return map_.add_topology_element();
	}


	template <bool B=true>
	inline auto ca_phi1() -> typename std::enable_if<B && MAP2::PRIM_SIZE==1,ChunkArray<Dart>&>::type
	{
		return *(map_.Map2::Inherit::phi1_);
	}

	template <bool B=true>
	inline auto ca_phi_1() -> typename std::enable_if< B &&MAP2::PRIM_SIZE==1,ChunkArray<Dart>&>::type
	{
		return *(map_.Map2::Inherit::phi_1_);
	}

	inline ChunkArray<Dart>& ca_phi2()
	{
		return *(map_.phi2_);
	}

	inline ChunkArrayContainer<uint8>& cac_topology()
	{
		return map_.topology_;
	}

private:

	Map2& map_;
};

} // namespace cgogn


#endif // CGOGN_CORE_CMAP_CMAP2_BUILDER_H_


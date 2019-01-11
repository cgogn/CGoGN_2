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

#ifndef CGOGN_CORE_CMAP_CMAP1_BUILDER_H_
#define CGOGN_CORE_CMAP_CMAP1_BUILDER_H_

#include <cgogn/core/cmap/map_base.h>

namespace cgogn
{

template <typename MAP1>
class CMap1Builder_T
{
	static_assert(MAP1::DIMENSION == 1, "CMap1Builder_T works only with 1D Maps.");

public:

	using Self = CMap1Builder_T<MAP1>;
	using Map1 = MAP1;
	using CDart = typename Map1::CDart;
	using Vertex = typename Map1::Vertex;
	using Face = typename Map1::Face;

	template <typename T>
	using ChunkArrayContainer = typename Map1::template ChunkArrayContainer<T>;

	template <typename T>
	using ChunkArray = typename Map1::template ChunkArray<T>;

	inline CMap1Builder_T(Map1& map) : map_(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap1Builder_T);
	inline ~CMap1Builder_T() {}

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

	inline void phi1_sew(Dart d, Dart e)
	{
		return map_.phi1_sew(d,e);
	}

	inline void phi1_unsew(Dart d)
	{
		map_.phi1_unsew(d);
	}

	inline void remove_face_topo(Dart d)
	{
		map_.remove_face_topo(d);
	}

	inline Dart add_face_topo(uint32 nb_edges)
	{
		return map_.add_face_topo(nb_edges);
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

	inline Dart add_topology_element()
	{
		return map_.add_topology_element();
	}

	inline ChunkArray<Dart>& ca_phi1()
	{
		return *(map_.phi1_);
	}

	inline ChunkArrayContainer<uint8>& cac_topology()
	{
		return map_.topology_;
	}

private:

	Map1& map_;
};

} //end namespace cgogn

#endif // CGOGN_CORE_CMAP_CMAP1_BUILDER_H_

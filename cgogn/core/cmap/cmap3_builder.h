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
#include <cgogn/core/cmap/cmap3.h>
//#include <cgogn/core/cmap/cmap3_tetra.h>
//#include <cgogn/core/cmap/cmap3_hexa.h>

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

	inline CMap3Builder_T(Map3& map) : map_(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap3Builder_T);
	inline ~CMap3Builder_T() {}

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

	inline void phi3_sew(Dart d, Dart e)
	{
		return map_.phi3_sew(d,e);
	}

	inline void phi3_unsew(Dart d)
	{
		return map_.phi3_unsew(d);
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

	inline void sew_volumes_fp(Dart v1, Dart v2)
	{
		map_.sew_volumes_fp(v1, v2);
	}

	inline void sew_volumes_topo(Face f1, Face f2)
	{
		map_.sew_volumes_topo(f1.dart, f2.dart);
	}

	inline Dart cut_face_topo(Dart d, Dart e)
	{
		return map_.cut_face_topo(d,e);
	}

	inline Dart cut_edge_topo(Dart d)
	{
		return map_.cut_edge_topo(d);
	}

	inline void merge_volumes_topo_fp(Dart v1, Dart v2)
	{
		using Map2 = typename Map3::Inherit;
		map_.Map2::merge_volumes_topo(v1, v2);
	}

	inline void merge_incident_faces_fp(Dart d)
	{
		using Map2 = typename Map3::Inherit;
		map_.Map2::merge_incident_faces_of_edge_topo(d);
	}

	inline Dart cut_face_topo_fp(Dart d, Dart e)
	{
		using Map2 = typename Map3::Inherit;
		return map_.Map2::cut_face_topo(d,e);
	}

	inline Dart cut_edge_topo_fp(Dart d)
	{
		using Map2 = typename Map3::Inherit;
		return map_.Map2::cut_edge_topo(d);
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

	inline uint32 close_map()
	{
		return map_.close_map();
	}

private:

	Map3& map_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_EXTERNAL_TEMPLATES_CPP_))
extern template class CGOGN_CORE_EXPORT CMap3Builder_T<CMap3>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_EXTERNAL_TEMPLATES_CPP_))


} // namespace cgogn

#endif // CGOGN_CORE_CMAP_CMAP3_BUILDER_H_

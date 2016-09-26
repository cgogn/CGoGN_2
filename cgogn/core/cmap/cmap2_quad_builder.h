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

#ifndef CGOGN_CORE_CMAP_CMAP2_QUAD_BUILDER_H_
#define CGOGN_CORE_CMAP_CMAP2_QUAD_BUILDER_H_

#include <cgogn/core/cmap/cmap2_quad.h>

namespace cgogn
{

template <typename MAP_TRAITS>
class CMap2QuadBuilder_T
{
public:

	using Self = CMap2QuadBuilder_T<MAP_TRAITS>;
	using CMap2Quad = cgogn::CMap2Quad<MAP_TRAITS>;
	using CDart = typename CMap2Quad::CDart;
	using Vertex = typename CMap2Quad::Vertex;
	using Edge = typename CMap2Quad::Edge;
	using Face = typename CMap2Quad::Face;
	using Volume = typename CMap2Quad::Volume;

	template <typename T>
	using ChunkArrayContainer = typename CMap2Quad::template ChunkArrayContainer<T>;

	inline CMap2QuadBuilder_T(CMap2Quad& map) : map_(map)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap2QuadBuilder_T);

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
	inline void set_embedding(Dart d, uint32 emb)
	{
		map_.template set_embedding<CellType>(d, emb);
	}

	template <class CellType>
	inline void new_orbit_embedding(CellType c)
	{
		map_.new_orbit_embedding(c);
	}

	inline void phi2_sew(Dart d, Dart e)
	{
		return map_.phi2_sew(d,e);
	}

	inline void phi2_unsew(Dart d)
	{
		map_.phi2_unsew(d);
	}

	inline Dart add_face_topo_parent(uint32 nb_edges)
	{
		cgogn_message_assert(nb_edges == 4u, "Can create only quads");
		if (nb_edges != 4u)
			cgogn_log_warning("add_face") << "Attempt to create a face which is not a quad";

		return map_.add_quad_topo_fp();
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

	inline Dart close_hole_topo(Dart d)
	{
		return map_.close_hole_topo(d);
	}

	/**
	 * @brief Close (not really) a hole with a set of quad.
	 * @return a face of the fan
	 */
	inline Face close_hole(Dart d)
	{
		//	const Face f(map_.close_hole_topo(d));
		Dart dh = map_.close_hole_topo(d);

		Dart di = dh;

		do
		{
			Dart di0 = map_.phi2(di);
			Dart di1 = map_.phi1(di);

			if (map_.template is_embedded<Vertex>())
			{
				map_.template copy_embedding<Vertex>(di,map_.phi1(di0));
				map_.template copy_embedding<Vertex>(di1,di0);
			}

			if (map_.template is_embedded<Edge>())
			{
				map_.template copy_embedding<Edge>(di,di0);
			}

			if (map_.template is_embedded<Volume>())
			{
				const uint32 idx = map_.embedding(Volume(d));
				map_.foreach_dart_of_orbit(Face(di), [this, idx] (Dart it)
				{
					map_.template set_embedding<Volume>(it, idx);
				});
			}

			di = map_.template phi<21>(di1);
		} while (di!=dh);

		return Face(dh);
	}

	/**
	 * @brief close_map
	 * @return the number of holes (filled)
	 */
	inline int32 close_map()
	{
		uint32 nb_holes = 0;

		std::vector<Dart>* fix_point_darts = cgogn::dart_buffers()->buffer();

		map_.foreach_dart([&] (Dart d)
		{
			if (map_.phi2(d) == d)
				fix_point_darts->push_back(d);
		});

		for (Dart d : (*fix_point_darts))
		{
			if (map_.phi2(d) == d)
			{
				Face f = close_hole(d);
				Dart df = f.dart;
				do
				{
					map_.boundary_mark(Face(df));
					df = map_.template phi<121>(df);
				} while (df != f.dart);
				++nb_holes;
			}
		}

		cgogn::dart_buffers()->release_buffer(fix_point_darts);

		return nb_holes;
	}

private:

	CMap2Quad& map_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CMAP_CMAP2_QUAD_BUILDER_CPP_))
extern template class CGOGN_CORE_API cgogn::CMap2QuadBuilder_T<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP2_BUILDER_CPP_))
using CMap2QuadBuilder = cgogn::CMap2QuadBuilder_T<DefaultMapTraits>;

} // namespace cgogn


#endif // CGOGN_CORE_CMAP_CMAP2_QUAD_BUILDER_H_


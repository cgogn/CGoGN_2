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

#ifndef CGOGN_CORE_CMAP_CMAP2_TRI_BUILDER_H_
#define CGOGN_CORE_CMAP_CMAP2_TRI_BUILDER_H_

#include <cgogn/core/cmap/cmap2_tri.h>

namespace cgogn
{

template <typename MAP_TRAITS>
class CMap2TriBuilder_T
{
public:

	using Self = CMap2TriBuilder_T<MAP_TRAITS>;
	using CMap2Tri = cgogn::CMap2Tri<MAP_TRAITS>;
	using CDart = typename CMap2Tri::CDart;
	using Vertex = typename CMap2Tri::Vertex;
	using Edge = typename CMap2Tri::Edge;
	using Face = typename CMap2Tri::Face;
	using Volume = typename CMap2Tri::Volume;

	template <typename T>
	using ChunkArrayContainer = typename CMap2Tri::template ChunkArrayContainer<T>;

	inline CMap2TriBuilder_T(CMap2Tri& map) : map_(map)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap2TriBuilder_T);

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
		cgogn_message_assert(nb_edges == 3u, "Can create only triangles");
		if (nb_edges != 3)
			cgogn_log_warning("add_face") << "Attempt to create a face which is not a triangle";

		return map_.add_tri_topo_fp();
	}

	inline Dart close_hole_topo(Dart d)
	{
		return map_.close_hole_topo(d);
	}

	/**
	 * @brief Close a hole with a triangle fan
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
	uint32 close_map()
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
				Vertex fan_center(map_.phi_1(f.dart));
				map_.foreach_incident_face(fan_center, [&] (Face ff)
				{
					map_.foreach_dart_of_orbit(ff, [&] (Dart e)
					{
						map_.set_boundary(e, true);
					});
				});
				++nb_holes;
			}
		}
		cgogn::dart_buffers()->release_buffer(fix_point_darts);
		return nb_holes;
	}

private:

	CMap2Tri& map_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CMAP_CMAP2_TRI_BUILDER_CPP_))
extern template class CGOGN_CORE_API cgogn::CMap2TriBuilder_T<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP2_BUILDER_CPP_))
using CMap2TriBuilder = cgogn::CMap2TriBuilder_T<DefaultMapTraits>;

} // namespace cgogn


#endif // CGOGN_CORE_CMAP_CMAP2_TRI_BUILDER_H_


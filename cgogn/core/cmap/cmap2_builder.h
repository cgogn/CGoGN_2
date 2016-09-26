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

#ifndef CGOGN_CORE_MAP_MAP2_BUILDER_H_
#define CGOGN_CORE_MAP_MAP2_BUILDER_H_

#include <cgogn/core/cmap/cmap2.h>

namespace cgogn
{

template <typename MAP_TRAITS>
class CMap2Builder_T
{
public:

	using Self = CMap2Builder_T<MAP_TRAITS>;
	using CMap2 = cgogn::CMap2<MAP_TRAITS>;
	using CDart = typename CMap2::CDart;
	using Vertex = typename CMap2::Vertex;
	using Edge = typename CMap2::Edge;
	using Face = typename CMap2::Face;
	using Volume = typename CMap2::Volume;

	template <typename T>
	using ChunkArrayContainer = typename CMap2::template ChunkArrayContainer<T>;

	inline CMap2Builder_T(CMap2& map) : map_(map)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap2Builder_T);

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
		return map_.CMap2::Inherit::add_face_topo(nb_edges);
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

	/*!
	 * \brief Close a hole with a new face and update the embedding of incident cells.
	 * \param d : a vertex of the hole
	 * This method is used to close a CMap2 that has been build through the 2-sewing of 1-faces.
	 * A face is inserted on the boundary that begin at dart d.
	 * If the map has Dart, Vertex, Edge, Face or Volume attributes,
	 * the embedding of the inserted face and incident cells are automatically updated.
	 * More precisely :
	 *  - a Face attribute is created, if needed, for the face that fill the hole.
	 *  - the Vertex, Edge and Volume attributes are copied, if needed, from incident cells.
	 */
	inline Face close_hole(Dart d)
	{
		const Face f(map_.close_hole_topo(d));

		//		if (map_.template is_embedded<CDart>())
		//		{
		//			map_.foreach_dart_of_orbit(f, [this] (Dart it)
		//			{
		//				map_.new_orbit_embedding(CDart(it));
		//			});
		//		}

		if (map_.template is_embedded<Vertex>())
		{
			map_.foreach_dart_of_orbit(f, [this] (Dart it)
			{
				map_.template copy_embedding<Vertex>(it, map_.phi1(map_.phi2(it)));
			});
		}

		if (map_.template is_embedded<Edge>())
		{
			map_.foreach_dart_of_orbit(f, [this] (Dart it)
			{
				map_.template copy_embedding<Edge>(it, map_.phi2(it));
			});
		}

		//		if (map_.template is_embedded<Face>())
		//			map_.new_orbit_embedding(f);

		if (map_.template is_embedded<Volume>())
		{
			const uint32 idx = map_.embedding(Volume(d));
			map_.foreach_dart_of_orbit(f, [this, idx] (Dart it)
			{
				map_.template set_embedding<Volume>(it, idx);
			});
		}

		return f;
	}

	/*!
	 * \brief Close the map by inserting faces in its holes and update the embedding of incident cells.
	 * This method is used to close a CMap2 that has been build through the 2-sewing of 1-faces.
	 * If the map has Dart, Vertex, Edge, Face or Volume attributes,
	 * the embedding of the inserted faces and incident cells are automatically updated.
	 * More precisely :
	 *  - Face attributes are created, if needed, for the faces that fill the holes.
	 *  - Vertex, Edge and Volume attributes are copied, if needed, from incident cells.
	 * If the indexation of embedding was unique, the closed map is well embedded.
	 */
	inline uint32 close_map()
	{
		uint32 nb_holes=0;

		std::vector<Dart>* fix_point_darts = dart_buffers()->buffer();
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
				map_.boundary_mark(f);
				++nb_holes;
			}
		}
		dart_buffers()->release_buffer(fix_point_darts);
		return nb_holes;
	}

private:

	CMap2& map_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP2_BUILDER_CPP_))
extern template class CGOGN_CORE_API cgogn::CMap2Builder_T<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP2_BUILDER_CPP_))
using CMap2Builder = cgogn::CMap2Builder_T<DefaultMapTraits>;

} // namespace cgogn


#endif // CGOGN_CORE_MAP_MAP2_BUILDER_H_


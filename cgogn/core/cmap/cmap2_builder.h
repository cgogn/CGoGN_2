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
	using CDart = typename CMap2::CDart;
	using Vertex = typename CMap2::Vertex;
	using Edge = typename CMap2::Edge;
	using Face = typename CMap2::Face;
	using Volume = typename CMap2::Volume;

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

	inline Dart add_face_topo_parent(unsigned int nb_edges)
	{
		return map_.CMap2::Inherit::add_face_topo(nb_edges);
	}

	/*!
	 * \brief Close the topological hole that contains Dart d (a fixed point for PHI2).
	 * \param d : a vertex of the hole
	 * \return a vertex of the face that close the hole
	 * This method is used to close a CMap2 that has been build through the 2-sewing of 1-faces.
	 * A face is inserted on the boundary that begin at dart d.
	 */
	inline Dart close_hole_topo(Dart d)
	{
		cgogn_message_assert(map_.phi2(d) == d, "CMap2: close hole called on a dart that is not a phi2 fix point");

		Dart first = map_.add_dart();	// First edge of the face that will fill the hole
		map_.phi2_sew(d, first);		// 2-sew the new edge to the hole

		Dart d_next = d;				// Turn around the hole
		Dart d_phi1;					// to complete the face
		do
		{
			do
			{
				d_phi1 = map_.phi1(d_next); // Search and put in d_next
				d_next = map_.phi2(d_phi1); // the next dart of the hole
			} while (d_next != d_phi1 && d_phi1 != d);

			if (d_phi1 != d)
			{
				Dart next = map_.split_vertex_topo(first);	// Add a vertex into the built face
				phi2_sew(d_next, next);						// and 2-sew the face to the hole
			}
		} while (d_phi1 != d);

		return first;
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
		const Face f(close_hole_topo(d));

		if (map_.template is_embedded<CDart>())
		{
			map_.foreach_dart_of_orbit(f, [this] (Dart it)
			{
				map_.new_orbit_embedding(CDart(it));
			});
		}

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

		if (map_.template is_embedded<Face>())
			map_.new_orbit_embedding(f);

		if (map_.template is_embedded<Volume>())
		{
			const unsigned int idx = map_.get_embedding(Volume(d));
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
	inline void close_map()
	{
		std::vector<Dart>* fix_point_darts = get_dart_buffers()->get_buffer();
		map_.foreach_dart_nomask( [&] (Dart d)
			{
				if (map_.phi2(d) == d)
					fix_point_darts->push_back(d);
			});

		for (Dart d : (*fix_point_darts))
		{
			if (map_.phi2(d) == d)
			{
				Face f = close_hole(d);
				map_.foreach_dart_of_orbit(f, [&] (Dart e)
				{
					map_.set_boundary(e, true);
				});
			}
		}
		get_dart_buffers()->release_buffer(fix_point_darts);
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


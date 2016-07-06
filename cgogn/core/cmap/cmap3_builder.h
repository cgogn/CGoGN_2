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

#include <cgogn/core/cmap/cmap3.h>

namespace cgogn
{

template <typename MAP_TRAITS>
class CMap3Builder_T
{
public:

	using Self = CMap3Builder_T<MAP_TRAITS>;
	using CMap3 = cgogn::CMap3<MAP_TRAITS>;
	using CDart = typename CMap3::CDart;
	using Vertex = typename CMap3::Vertex;
	using Vertex2 = typename CMap3::Vertex2;
	using Edge = typename CMap3::Edge;
	using Edge2 = typename CMap3::Edge2;
	using Face = typename CMap3::Face;
	using Face2 = typename CMap3::Face2;
	using Volume = typename CMap3::Volume;

	using DartMarkerStore = typename CMap3::DartMarkerStore;
	template <typename T>
	using ChunkArrayContainer = typename CMap3::template ChunkArrayContainer<T>;

	inline CMap3Builder_T(CMap3& map) : map_(map)
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

	inline void init_parent_vertex_embedding(Dart d, uint32 emb)
	{
		map_.foreach_dart_of_orbit(Vertex2(d), [&] (Dart dit)
		{
			map_.template set_embedding<Vertex>(dit, emb);
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

	inline Dart add_face_topo(uint32 nb_edges)
	{
		return map_.add_face_topo(nb_edges);
	}

	inline Dart add_prism_topo(uint32 nb_edges)
	{
		return map_.add_prism_topo(nb_edges);
	}

	inline Dart add_pyramid_topo(uint32 nb_edges)
	{
		return map_.add_pyramid_topo(nb_edges);
	}

	inline Dart add_stamp_volume_topo()
	{
		return map_.add_stamp_volume_topo();
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

	/**
	 * @brief sew two volumes along a face
	 * The darts given in the Volume parameters must be part of Face2 that have
	 * a similar co-degree and whose darts are all phi3 fix points
	 * @param v1 first volume
	 * @param v2 second volume
	 */
	inline void sew_volumes(Dart v1, Dart v2)
	{
		Dart it1 = v1;
		Dart it2 = v2;
		const Dart begin = it1;
		do
		{
			phi3_sew(it1, it2);
			it1 = map_.phi1(it1);
			it2 = map_.phi_1(it2);
		} while (it1 != begin);
	}

	inline void close_hole_topo(Dart d)
	{
		cgogn_message_assert(map_.phi3(d) == d, "CMap3: close hole called on a dart that is not a phi3 fix point");

		DartMarkerStore dmarker(map_);
		DartMarkerStore boundary_marker(map_);

		std::vector<Dart> visitedFaces;	// Faces that are traversed
		visitedFaces.reserve(1024);

		visitedFaces.push_back(d);		// Start with the face of d
		dmarker.mark_orbit(Face2(d));

		uint32 count = 0u;

		// For every face added to the list
		for(uint32 i = 0u; i < visitedFaces.size(); ++i)
		{
			Dart it = visitedFaces[i];
			Dart f = it;

			const Dart b = map_.CMap3::Inherit::Inherit::add_face_topo(map_.codegree(Face(f)));
			boundary_marker.mark_orbit(Face2(b));
			++count;

			Dart bit = b;
			do
			{
				Dart e = map_.phi3(map_.phi2(f));;
				bool found = false;
				do
				{
					if (map_.phi3(e) == e)
					{
						found = true;
						if (!dmarker.is_marked(e))
						{
							visitedFaces.push_back(e);
							dmarker.mark_orbit(Face2(e));
						}
					}
					else
					{
						if (boundary_marker.is_marked(e))
						{
							found = true;
							this->phi2_sew(e, bit);
						}
						else
							e = map_.phi3(map_.phi2(e));
					}
				} while(!found);

				phi3_sew(f, bit);
				bit = map_.phi_1(bit);
				f = map_.phi1(f);
			} while(f != it);
		}
	}

	/**
	 * @brief close_map : /!\ DO NOT USE /!\ Close the map removing topological holes (only for import/creation)
	 * Add volumes to the map that close every existing hole.
	 * @return the number of closed holes
	 */
	inline void close_map()
	{
		// Search the map for topological holes (fix points of phi3)
		std::vector<Dart>* fix_point_darts = dart_buffers()->buffer();
		map_.foreach_dart([&] (Dart d)
		{
			if (map_.phi3(d) == d)
				fix_point_darts->push_back(d);
		});
		for (Dart d : (*fix_point_darts))
		{
			if (map_.phi3(d) == d)
			{
				close_hole_topo(d);
				map_.foreach_dart_of_orbit(Volume(map_.phi3(d)), [&] (Dart db)
				{
					map_.set_boundary(db, true);
				});

				const Volume new_volume(map_.phi3(d));

				if (map_.template is_embedded<Vertex>())
				{
					map_.foreach_dart_of_orbit(new_volume, [this] (Dart it)
					{
						map_.template copy_embedding<Vertex>(it, map_.phi1(map_.phi3(it)));
					});
				}

				if (map_.template is_embedded<Edge>())
				{
					map_.foreach_dart_of_orbit(new_volume, [this] (Dart wd)
					{
						map_.template copy_embedding<Edge>(wd, map_.phi3(wd));
					});
				}

				if (map_.template is_embedded<Face>())
				{
					map_.foreach_dart_of_orbit(new_volume, [this] (Dart wd)
					{
						map_.template copy_embedding<Face>(wd, map_.phi3(wd));
					});
				}
			}
		}
		dart_buffers()->release_buffer(fix_point_darts);
	}

private:

	CMap3& map_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CMAP_CMAP3_BUILDER_CPP_))
extern template class CGOGN_CORE_API cgogn::CMap3Builder_T<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CMAP_CMAP3_BUILDER_CPP_))
using CMap3Builder = cgogn::CMap3Builder_T<DefaultMapTraits>;

} // namespace cgogn


#endif // CGOGN_CORE_CMAP_CMAP3_BUILDER_H_


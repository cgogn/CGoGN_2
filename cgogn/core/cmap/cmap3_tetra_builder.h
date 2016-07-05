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

#ifndef CGOGN_CORE_CMAP_CMAP3_TETRA_BUILDER_H_
#define CGOGN_CORE_CMAP_CMAP3_TETRA_BUILDER_H_

#include <cgogn/core/cmap/cmap3_tetra.h>

namespace cgogn
{

template <typename MAP_TRAITS>
class CMap3TetraBuilder_T
{
public:

	using Self = CMap3TetraBuilder_T<MAP_TRAITS>;
	using CMap3Tetra = cgogn::CMap3Tetra<MAP_TRAITS>;
	using CDart = typename CMap3Tetra::CDart;
	using Vertex = typename CMap3Tetra::Vertex;
	using Vertex2 = typename CMap3Tetra::Vertex2;
	using Edge = typename CMap3Tetra::Edge;
	using Edge2 = typename CMap3Tetra::Edge2;
	using Face = typename CMap3Tetra::Face;
	using Face2 = typename CMap3Tetra::Face2;
	using Volume = typename CMap3Tetra::Volume;

	using DartMarkerStore = typename CMap3Tetra::DartMarkerStore;
	template <typename T>
	using ChunkArrayContainer = typename CMap3Tetra::template ChunkArrayContainer<T>;



	inline CMap3TetraBuilder_T(CMap3Tetra& map) : map_(map)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CMap3TetraBuilder_T);

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

	inline void phi3_sew(Dart d, Dart e)
	{
		return map_.phi3_sew(d,e);
	}

	inline void phi3_unsew(Dart d)
	{
		return map_.phi3_unsew(d);
	}

	inline Dart add_pyramid_topo(uint32 nb_edges)
	{
		cgogn_message_assert(nb_edges == 3u, "Can create only tetra");
		if (nb_edges != 3)
		{
			cgogn_log_warning("add_pyramid_topo") << "Attempt to create a volume which is not a tetrahedron in Map3Tetra";
			return Dart();
		}
		return map_.add_tetra_topo_fp();
	}

	inline Dart add_face_topo(uint32)
	{
		cgogn_message_assert(false, "Can create only tetra");
		cgogn_log_warning("add_face_topo") << "Attempt to create a volume which is not a tetrahedron in Map3Tetra";
		return Dart();
	}

	inline Dart add_prism_topo(uint32)
	{
		cgogn_message_assert(false, "Can create only tetra");
		cgogn_log_warning("add_prism_topo") << "Attempt to create a volume which is not a tetrahedron in Map3Tetra";
		return Dart();
	}

	inline Dart add_stamp_volume_topo()
	{
		cgogn_message_assert(false, "Can create only tetra");
		cgogn_log_warning("add_stamp_volume_topo") << "Attempt to create a volume which is not a tetrahedron in Map3Tetra";
		return Dart();
	}


	/**
	 * @brief sew two tetrahedra along a face
	 * The darts given in the parameters must be part of Face2 that have
	 * a similar co-degree and whose darts are all phi3 fix points
	 * @param dv1 dart of first tetrahedron
	 * @param dv2 dart of second tetrahedron
	 */
	inline void sew_volumes(Dart dv1, Dart dv2)
	{
		phi3_sew(dv1, dv2);

		dv1 = map_.phi1(dv1);
		dv2 = map_.phi_1(dv2);

		phi3_sew(dv1, dv2);

		dv1 = map_.phi1(dv1);
		dv2 = map_.phi_1(dv2);

		phi3_sew(dv1, dv2);
	}


	void close_hole_topo(Dart d, bool mark_boundary=false)
	{
		cgogn_message_assert(map_.phi3(d) == d, "CMap3Tetra: close hole called on a dart that is not a phi3 fix point");

		DartMarkerStore fmarker(map_);
		DartMarkerStore boundary_marker(map_);

		std::vector<Dart> visited_faces;	// Faces that are traversed
		visited_faces.reserve(1024);
		visited_faces.push_back(d);		// Start with the face of d
		fmarker.mark_orbit(Face2(d));


		auto local_func = [&] (Dart f)
		{
			Dart e = map_.phi3(map_.phi2(f));;
			bool found = false;
			do
			{
				if (map_.phi3(e) == e)
				{
					found = true;
					if (!fmarker.is_marked(e))
					{
						visited_faces.push_back(e);
						fmarker.mark_orbit(Face2(e));
					}
				}
				else
				{
					if (boundary_marker.is_marked(e))
					{
						found = true;
						sew_volumes(map_.template phi<32>(f),map_.phi2(e));
					}
					else
						e = map_.phi3(map_.phi2(e));
				}
			} while(!found);
		};



		// For every face added to the list
		for(uint32 i = 0u; i < visited_faces.size(); ++i)
		{
			Dart f = visited_faces[i];

			const Dart tb = map_.add_tetra_topo_fp();
			boundary_marker.mark_orbit(Volume(tb));
			sew_volumes(tb,f);

			local_func(f);
			f = map_.phi1(f);
			local_func(f);
			f = map_.phi1(f);
			local_func(f);
		}

		if (mark_boundary)
		{
			for (Dart d: *(boundary_marker.marked_darts()) )
			{
				map_.set_boundary(d,true);
			}
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
				close_hole_topo(d,true);
			}
		}

		if (map_.template is_embedded<Vertex>())
		{
			for (Dart d : (*fix_point_darts))
			{
				Dart e = map_.phi3(d);
				map_.template copy_embedding<Vertex>(map_.phi2(e),d);
				e = map_.phi1(e);
				map_.template copy_embedding<Vertex>(e,d);
				map_.template copy_embedding<Vertex>(map_.template phi<21>(e),d);
			}
		}

		if (map_.template is_embedded<Edge>())
		{
			for (Dart d : (*fix_point_darts))
			{
				Dart e = map_.phi3(d);
				map_.template copy_embedding<Edge>(e,d);
				map_.template copy_embedding<Edge>(map_.phi2(e),d);
			}
		}

		if (map_.template is_embedded<Face>())
		{
			for (Dart d : (*fix_point_darts))
				map_.template copy_embedding<Face>(map_.phi3(d),d);
		}

		dart_buffers()->release_buffer(fix_point_darts);
	}

private:

	CMap3Tetra& map_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_CMAP_CMAP3_TETRA_BUILDER_CPP_))
extern template class CGOGN_CORE_API cgogn::CMap3TetraBuilder_T<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_CORE_MAP_MAP3_TETRA_BUILDER_CPP_))
using CMap3TetraBuilder = cgogn::CMap3TetraBuilder_T<DefaultMapTraits>;

} // namespace cgogn


#endif // CGOGN_CORE_CMAP_CMAP3_TETRA_BUILDER_H_


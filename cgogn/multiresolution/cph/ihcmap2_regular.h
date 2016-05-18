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

#ifndef CGOGN_MULTIRESOLUTION_CPH_IHCMAP2_REGULAR_H_
#define CGOGN_MULTIRESOLUTION_CPH_IHCMAP2_REGULAR_H_

#include <cgogn/multiresolution/cph/ihcmap2.h>

namespace cgogn
{

template <typename MAP_TRAITS, typename MAP_TYPE>
class IHCMap2Regular_T : public IHCMap2_T<MAP_TRAITS, MAP_TYPE>
{
public:
	using MapType = MAP_TYPE;
	using Inherit = IHCMap2_T<MAP_TRAITS, MAP_TYPE>;
	using Self = IHCMap2Regular_T<MAP_TRAITS,MAP_TYPE>;

	friend class MapBase<MAP_TRAITS, MAP_TYPE>;

	using Vertex = typename Inherit::Vertex;
	using Edge = typename Inherit::Edge;
	using Face = typename Inherit::Face;
	using Volume = typename Inherit::Volume;

	inline IHCMap2Regular_T() : Inherit() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(IHCMap2Regular_T);
	inline ~IHCMap2Regular_T() {}

	/*******************************************************************************
	 * Low-level topological operations
	 *******************************************************************************/

protected:

	/**
	* \brief Init an newly added dart.
	* The dart is added to the current level of resolution
	*/
	inline void init_dart(Dart d)
	{
		Inherit::init_dart(d);
	}

public:

	inline void add_triangular_level()
	{
		uint32 cur = Inherit::get_current_level() ;

		Inherit::set_current_level(Inherit::get_maximum_level() + 1) ;

		//cut edges
		Inherit::template foreach_cell<TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Edge e)
		{
			Dart dd = Inherit::phi2(e.dart);
			//			Inherit::cut_edge(e);

			uint32 eid = Inherit::get_edge_id(e.dart);
			Inherit::set_edge_id(Inherit::phi1(e.dart), eid);
			Inherit::set_edge_id(Inherit::phi1(dd), eid);
		});

		//cut faces
		Inherit::template foreach_cell<TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Face d)
		{
			Dart old = d.dart ;

			if(Inherit::get_dart_level(old) == Inherit::get_maximum_level())
				old = Inherit::phi1(old) ;

			Dart dd = Inherit::phi1(old) ;
			Dart e = Inherit::phi1(Inherit::phi1(dd)) ;
			// insert a new edge
			//			Inherit::cut_face(dd, e) ;

			uint32 id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
			Inherit::set_edge_id(Inherit::phi_1(dd), id) ;		// set the edge id of the inserted
			Inherit::set_edge_id(Inherit::phi_1(e), id) ;		// edge to the next available id

			dd = e ;
			e = Inherit::phi1(Inherit::phi1(dd)) ;
			//			Inherit::cut_face(dd, e) ;
			id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
			Inherit::set_edge_id(Inherit::phi_1(dd), id) ;
			Inherit::set_edge_id(Inherit::phi_1(e), id) ;

			dd = e ;
			e = Inherit::phi1(Inherit::phi1(dd)) ;
			//			Inherit::cut_face(dd, e) ;
			id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
			Inherit::set_edge_id(Inherit::phi_1(dd), id) ;
			Inherit::set_edge_id(Inherit::phi_1(e), id) ;
		});

		Inherit::set_current_level(cur) ;
	}

	inline void add_quadrangular_level()
	{
		uint32 cur = Inherit::get_current_level() ;

		Inherit::set_current_level(Inherit::get_maximum_level() + 1) ;

		//cut edges
		Inherit::template foreach_cell<TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Edge e)
		{
			Dart dd = Inherit::phi2(e.dart);
			//			Inherit::cut_edge(e);

			uint32 eid = Inherit::get_edge_id(e.dart);
			Inherit::set_edge_id(Inherit::phi1(e.dart), eid);
			Inherit::set_edge_id(Inherit::phi1(dd), eid);
		});

		//cut faces
		Inherit::template foreach_cell<TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Face d)
		{
			Dart old = d.dart ;

			if(Inherit::get_dart_level(old) == Inherit::get_maximum_level())
				old = Inherit::phi1(old) ;

			Dart dd = Inherit::phi1(old) ;
			Dart next = Inherit::phi1(Inherit::phi1(dd)) ;
			//			Inherit::cut_face(dd, next) ;		// insert a first edge

			Dart ne = Inherit::phi2(Inherit::phi_1(dd)) ;
			Dart ne2 = Inherit::phi2(ne) ;
			//			Inherit::cut_edge(ne) ;				// cut the new edge to insert the central vertex

			uint32 id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(Inherit::phi2(ne)));
			Inherit::set_edge_id(ne, id) ;
			Inherit::set_edge_id(Inherit::phi2(ne), id) ;			// set the edge id of the inserted

			id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(ne2));
			Inherit::set_edge_id(ne2, id) ;					// edges to the next available ids
			Inherit::set_edge_id(Inherit::phi2(ne2), id) ;

			dd = Inherit::phi1(Inherit::phi1(next)) ;
			while(dd != ne)				// turn around the face and insert new edges
			{							// linked to the central vertex
				Dart tmp = Inherit::phi1(ne) ;
				//				Inherit::cut_face(tmp, dd) ;

				Dart nne = Inherit::phi2(Inherit::phi_1(dd)) ;

				id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(Inherit::phi2(nne)));
				Inherit::set_edge_id(nne, id) ;
				Inherit::set_edge_id(Inherit::phi2(nne), id) ;
				dd = Inherit::phi1(Inherit::phi1(dd)) ;
			}
		});

		Inherit::set_current_level(cur) ;
	}

	inline void add_mixed_level()
	{
		uint32 cur = Inherit::get_current_level() ;

		Inherit::set_current_level(Inherit::get_maximum_level() + 1) ;

		//cut edges
		Inherit::template foreach_cell<TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Edge e)
		{
			Dart dd = Inherit::phi2(e.dart);
			//			Inherit::cut_edge(e);

			uint32 eid = Inherit::get_edge_id(e.dart);
			Inherit::set_edge_id(Inherit::phi1(e.dart), eid);
			Inherit::set_edge_id(Inherit::phi1(dd), eid);
		});

		//cut faces
		Inherit::template foreach_cell<TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Face d)
		{
			Dart old = d.dart ;

			if(Inherit::get_dart_level(old) == Inherit::get_maximum_level())
				old = Inherit::phi1(old) ;

			uint32 cur = Inherit::get_current_level();
			Inherit::set_current_level(cur - 1);
			uint32 degree = Inherit::face_degree(old) ;
			Inherit::set_current_level(cur);

			if(degree == 3)
			{
				Dart dd = Inherit::phi1(old) ;
				Dart e = Inherit::phi1(Inherit::phi1(dd)) ;
				// insert a new edge
				//				Inherit::cut_face(dd, e) ;

				uint32 id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
				Inherit::set_edge_id(Inherit::phi_1(dd), id) ;		// set the edge id of the inserted
				Inherit::set_edge_id(Inherit::phi_1(e), id) ;		// edge to the next available id

				dd = e ;
				e = Inherit::phi1(Inherit::phi1(dd)) ;
				//				Inherit::cut_face(dd, e) ;
				id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
				Inherit::set_edge_id(Inherit::phi_1(dd), id) ;
				Inherit::set_edge_id(Inherit::phi_1(e), id) ;

				dd = e ;
				e = Inherit::phi1(Inherit::phi1(dd)) ;
				//				Inherit::cut_face(dd, e) ;
				id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
				Inherit::set_edge_id(Inherit::phi_1(dd), id) ;
				Inherit::set_edge_id(Inherit::phi_1(e), id) ;
			}
			else
			{
				Dart dd = Inherit::phi1(old) ;
				Dart next = Inherit::phi1(Inherit::phi1(dd)) ;
				//				Inherit::cut_face(dd, next) ;		// insert a first edge

				Dart ne = Inherit::phi2(Inherit::phi_1(dd)) ;
				Dart ne2 = Inherit::phi2(ne) ;
				//				Inherit::cut_edge(ne) ;				// cut the new edge to insert the central vertex

				uint32 id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(Inherit::phi2(ne)));
				Inherit::set_edge_id(ne, id) ;
				Inherit::set_edge_id(Inherit::phi2(ne), id) ;			// set the edge id of the inserted

				id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(ne2));
				Inherit::set_edge_id(ne2, id) ;					// edges to the next available ids
				Inherit::set_edge_id(Inherit::phi2(ne2), id) ;

				dd = Inherit::phi1(Inherit::phi1(next)) ;
				while(dd != ne)				// turn around the face and insert new edges
				{							// linked to the central vertex
					Dart tmp = Inherit::phi1(ne) ;
					//					Inherit::cut_face(tmp, dd) ;

					Dart nne = Inherit::phi2(Inherit::phi_1(dd)) ;

					id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(Inherit::phi2(nne)));
					Inherit::set_edge_id(nne, id) ;
					Inherit::set_edge_id(Inherit::phi2(nne), id) ;
					dd = Inherit::phi1(Inherit::phi1(dd)) ;
				}
			}
		});

		Inherit::set_current_level(cur) ;
	}
protected:
	inline Face add_face_update_emb(Face f)
	{
		CGOGN_CHECK_CONCRETE_TYPE;
		cgogn_log_error("IHCMap2Regular_T::add_face_update_emb") << "Method is not implemented yet.";
		return f;
	}
};

template <typename MAP_TRAITS>
struct IHCMap2RegularType
{
	using TYPE = IHCMap2Regular_T<MAP_TRAITS, IHCMap2RegularType<MAP_TRAITS>>;
};

template <typename MAP_TRAITS>
using IHCMap2Regular = IHCMap2Regular_T<MAP_TRAITS, IHCMap2RegularType<MAP_TRAITS>>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MULTIRESOLUTION_CPH_IHCMAP2_REGULAR_CPP_))
extern template class CGOGN_MULTIRESOLUTION_API IHCMap2Regular_T<DefaultMapTraits, IHCMap2RegularType<DefaultMapTraits>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MULTIRESOLUTION_CPH_IHCMAP2_REGULAR_CPP_))

} // namespace cgogn

#endif // CGOGN_MULTIRESOLUTION_CPH_IHCMAP2_REGULAR_H_

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

#ifndef MULTIRESOLUTION_CPH_IHCMAP2_REGULAR_H_
#define MULTIRESOLUTION_CPH_IHCMAP2_REGULAR_H_

#include <multiresolution/cph/ihcmap2.h>

namespace cgogn
{

template <typename MAP_TRAITS, typename MAP_TYPE>
class IHCMap2Regular_T : public IHCMap2_T<MAP_TRAITS, MAP_TYPE>
{
public:
	using MapType = MAP_TYPE;
	using Inherit = IHCMap2_T<MAP_TRAITS, MAP_TYPE>;
	using Self = IHCMap2Regular_T<MAP_TRAITS,MAP_TYPE>;
	friend typename Inherit::Inherit_CMAP;

	using Vertex = typename Inherit::Vertex;
	using Edge = typename Inherit::Edge;
	using Face = typename Inherit::Face;
	using Volume = typename Inherit::Volume;

	IHCMap2Regular_T() : Inherit()
	{}

	IHCMap2Regular_T(const Self&) = delete;
	IHCMap2Regular_T(Self&&) = delete;
	Self& operator=(const Self&) = delete;
	Self& operator=(Self&&) = delete;
	inline ~IHCMap2Regular_T() = default;

public:

	inline void add_triangular_level()
	{
		unsigned int cur = Inherit::get_current_level() ;

		Inherit::set_current_level(Inherit::get_maximum_level() + 1) ;

		//cut edges
		Inherit::template foreach_cell<TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Edge e)
		{
			Dart dd = Inherit::phi2(e);
			//			Inherit::cut_edge(e);

			unsigned int eid = Inherit::get_edge_id(e);
			Inherit::set_edge_id(Inherit::phi1(e), eid);
			Inherit::set_edge_id(Inherit::phi1(dd), eid);
		});

		//cut faces
		Inherit::template foreach_cell<TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Face d)
		{
			Dart old = d ;

			if(Inherit::get_dart_level(old) == Inherit::get_maximum_level())
				old = Inherit::phi1(old) ;

			Dart dd = Inherit::phi1(old) ;
			Dart e = Inherit::phi1(Inherit::phi1(dd)) ;
			// insert a new edge
			//			Inherit::split_face(dd, e) ;

			unsigned int id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
			Inherit::set_edge_id(Inherit::phi_1(dd), id) ;		// set the edge id of the inserted
			Inherit::set_edge_id(Inherit::phi_1(e), id) ;		// edge to the next available id

			dd = e ;
			e = Inherit::phi1(Inherit::phi1(dd)) ;
			//			Inherit::split_face(dd, e) ;
			id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
			Inherit::set_edge_id(Inherit::phi_1(dd), id) ;
			Inherit::set_edge_id(Inherit::phi_1(e), id) ;

			dd = e ;
			e = Inherit::phi1(Inherit::phi1(dd)) ;
			//			Inherit::split_face(dd, e) ;
			id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
			Inherit::set_edge_id(Inherit::phi_1(dd), id) ;
			Inherit::set_edge_id(Inherit::phi_1(e), id) ;
		});

		Inherit::set_current_level(cur) ;
	}

	inline void add_quadrangular_level()
	{
		unsigned int cur = Inherit::get_current_level() ;

		Inherit::set_current_level(Inherit::get_maximum_level() + 1) ;

		//cut edges
		Inherit::template foreach_cell<TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Edge e)
		{
			Dart dd = Inherit::phi2(e);
			//			Inherit::cut_edge(e);

			unsigned int eid = Inherit::get_edge_id(e);
			Inherit::set_edge_id(Inherit::phi1(e), eid);
			Inherit::set_edge_id(Inherit::phi1(dd), eid);
		});

		//cut faces
		Inherit::template foreach_cell<TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Face d)
		{
			Dart old = d ;

			if(Inherit::get_dart_level(old) == Inherit::get_maximum_level())
				old = Inherit::phi1(old) ;

			Dart dd = Inherit::phi1(old) ;
			Dart next = Inherit::phi1(Inherit::phi1(dd)) ;
			//			Inherit::split_face(dd, next) ;		// insert a first edge

			Dart ne = Inherit::phi2(Inherit::phi_1(dd)) ;
			Dart ne2 = Inherit::phi2(ne) ;
			//			Inherit::cut_edge(ne) ;				// cut the new edge to insert the central vertex

			unsigned int id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(Inherit::phi2(ne)));
			Inherit::set_edge_id(ne, id) ;
			Inherit::set_edge_id(Inherit::phi2(ne), id) ;			// set the edge id of the inserted

			id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(ne2));
			Inherit::set_edge_id(ne2, id) ;					// edges to the next available ids
			Inherit::set_edge_id(Inherit::phi2(ne2), id) ;

			dd = Inherit::phi1(Inherit::phi1(next)) ;
			while(dd != ne)				// turn around the face and insert new edges
			{							// linked to the central vertex
				Dart tmp = Inherit::phi1(ne) ;
				//				Inherit::split_face(tmp, dd) ;

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
		unsigned int cur = Inherit::get_current_level() ;

		Inherit::set_current_level(Inherit::get_maximum_level() + 1) ;

		//cut edges
		Inherit::template foreach_cell<TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Edge e)
		{
			Dart dd = Inherit::phi2(e);
			//			Inherit::cut_edge(e);

			unsigned int eid = Inherit::get_edge_id(e);
			Inherit::set_edge_id(Inherit::phi1(e), eid);
			Inherit::set_edge_id(Inherit::phi1(dd), eid);
		});

		//cut faces
		Inherit::template foreach_cell<TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Face d)
		{
			Dart old = d ;

			if(Inherit::get_dart_level(old) == Inherit::get_maximum_level())
				old = Inherit::phi1(old) ;

			unsigned int cur = Inherit::get_current_level();
			Inherit::set_current_level(cur - 1);
			unsigned int degree = Inherit::face_degree(old) ;
			Inherit::set_current_level(cur);

			if(degree == 3)
			{
				Dart dd = Inherit::phi1(old) ;
				Dart e = Inherit::phi1(Inherit::phi1(dd)) ;
				// insert a new edge
				//				Inherit::split_face(dd, e) ;

				unsigned int id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
				Inherit::set_edge_id(Inherit::phi_1(dd), id) ;		// set the edge id of the inserted
				Inherit::set_edge_id(Inherit::phi_1(e), id) ;		// edge to the next available id

				dd = e ;
				e = Inherit::phi1(Inherit::phi1(dd)) ;
				//				Inherit::split_face(dd, e) ;
				id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
				Inherit::set_edge_id(Inherit::phi_1(dd), id) ;
				Inherit::set_edge_id(Inherit::phi_1(e), id) ;

				dd = e ;
				e = Inherit::phi1(Inherit::phi1(dd)) ;
				//				Inherit::split_face(dd, e) ;
				id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
				Inherit::set_edge_id(Inherit::phi_1(dd), id) ;
				Inherit::set_edge_id(Inherit::phi_1(e), id) ;
			}
			else
			{
				Dart dd = Inherit::phi1(old) ;
				Dart next = Inherit::phi1(Inherit::phi1(dd)) ;
				//				Inherit::split_face(dd, next) ;		// insert a first edge

				Dart ne = Inherit::phi2(Inherit::phi_1(dd)) ;
				Dart ne2 = Inherit::phi2(ne) ;
				//				Inherit::cut_edge(ne) ;				// cut the new edge to insert the central vertex

				unsigned int id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(Inherit::phi2(ne)));
				Inherit::set_edge_id(ne, id) ;
				Inherit::set_edge_id(Inherit::phi2(ne), id) ;			// set the edge id of the inserted

				id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(ne2));
				Inherit::set_edge_id(ne2, id) ;					// edges to the next available ids
				Inherit::set_edge_id(Inherit::phi2(ne2), id) ;

				dd = Inherit::phi1(Inherit::phi1(next)) ;
				while(dd != ne)				// turn around the face and insert new edges
				{							// linked to the central vertex
					Dart tmp = Inherit::phi1(ne) ;
					//					Inherit::split_face(tmp, dd) ;

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
		std::cerr << "IHCMap2Regular_T::add_face_update_emb method is not implemented yet." << std::endl;
		return f;
	}
};

template <typename MAP_TRAITS>
struct IHCMap2RegularType
{
	typedef IHCMap2Regular_T<MAP_TRAITS, IHCMap2RegularType<MAP_TRAITS>> TYPE;
};

template <typename MAP_TRAITS>
using IHCMap2Regular = IHCMap2Regular_T<MAP_TRAITS, IHCMap2RegularType<MAP_TRAITS>>;

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(MULTIRESOLUTION_CPH_IHCMAP2_REGULAR_CPP_))
extern template class CGOGN_MULTIRESOLUTION_API IHCMap2Regular_T<DefaultMapTraits, IHCMap2RegularType<DefaultMapTraits>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(MULTIRESOLUTION_CPH_IHCMAP2_REGULAR_CPP_))

} // namespace cgogn

#endif // MULTIRESOLUTION_CPH_IHCMAP2_REGULAR_H_

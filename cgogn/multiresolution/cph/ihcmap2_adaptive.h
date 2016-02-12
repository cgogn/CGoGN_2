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

#ifndef MULTIRESOLUTION_CPH_IHCMAP2_ADAPTIVE_H_
#define MULTIRESOLUTION_CPH_IHCMAP2_ADAPTIVE_H_

#include <multiresolution/cph/ihcmap2.h>

namespace cgogn
{

template <typename MAP_TRAITS>
class IHCMap2Adaptive : IHCMap2<MAP_TRAITS>
{
public:
	typedef IHCMap2<MAP_TRAITS> Inherit;
	typedef IHCMap2Adaptive<MAP_TRAITS> Self;


	IHCMap2Adaptive() : Inherit()
	{}

	~IHCMap2Adaptive() override
	{}

	IHCMap2Adaptive(const Self&) = delete;
	IHCMap2Adaptive(Self&&) = delete;
	Self& operator=(const Self&) = delete;
	Self& operator=(Self&&) = delete;

public:
	/***************************************************
	 *               CELLS INFORMATION                 *
	 ***************************************************/

	/**
	 * Return the level of the edge of d in the current level map
	 * \details The level of an edge is the maximum of the levels of
	 * its darts. As phi1(d) and phi2(d) are from the same level we can
	 * optimize by checking phi1(d) instead of phi2(d)
	 */
	unsigned int edge_level(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");
		return std::max(Inherit::get_dart_level(d),Inherit::get_dart_level(Inherit::phi1(d)));
	}

	/**
	 * Return the level of the face of d in the current level map
	 * \details The level of a face is the minimum of the levels of its edges
	 * but a specific treatment has to be done in the particular case of a
	 * face with all neighboring faces are regularly subdivided
	 * but not the face itself
	 */
	unsigned int face_level(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		if(Inherit::get_current_level() == 0)
			return 0;

		Dart it = d;
		Dart old = it;
		unsigned int l_old = Inherit::get_dart_level(old);
		unsigned int fLevel = edge_level(it);
		do
		{
			it = Inherit::phi1(it);
			unsigned int dl = Inherit::get_dart_level(it);

			// compute the oldest dart of the face in the same time
			if(dl < l_old)
			{
				old = it;
				l_old = dl;
			}
			unsigned int l = edge_level(it);
			fLevel = l < fLevel ? l : fLevel;
		} while(it != d);

		unsigned int cur = Inherit::get_current_level();
		Inherit::set_current_level(fLevel);

		unsigned int nbSubd = 0;
		it = old;
		unsigned int eId = Inherit::get_edge_id(old);
		do
		{
			++nbSubd;
			it = Inherit::phi1(it);
		} while(Inherit::get_edge_id(it) == eId);

		while(nbSubd > 1)
		{
			nbSubd /= 2;
			--fLevel;
		}

		Inherit::set_current_level(cur) ;
		return fLevel ;
	}

	/**
	 * Given the face of d in the current level map,
	 * return a level 0 dart of its origin face
	 */
	Dart face_origin(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		unsigned int cur = Inherit::get_current_level();
		Dart p = d;
		unsigned int pLevel = Inherit::get_dart_level(p);
		while(pLevel > 0)
		{
			p = face_oldest_dart(p);
			pLevel = Inherit::get_dart_level(p);
			Inherit::set_current_level(pLevel);
		}
		Inherit::set_current_level(cur);
		return p;
	}

	/**
	 * Return the oldest dart of the face of d in the current level map
	 */
	Dart face_oldest_dart(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		Dart it = d ;
		Dart oldest = it ;
		unsigned int l_old = Inherit::get_dart_level(oldest);
		do
		{
			unsigned int l = Inherit::get_dart_level(it);
			if(l == 0)
				return it;

			if(l < l_old)
				//		if(l < l_old || (l == l_old && it < oldest))
			{
				oldest = it;
				l_old = l;
			}
			it = Inherit::phi1(it);
		} while(it != d);

		return oldest;
	}

	/**
	 * Return true if the edge of d in the current level map
	 * has already been subdivided to the next level
	 * As before testing phi2(d) or phi1(d) is the same
	 */
	bool edge_is_subdivided(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		if(Inherit::get_current_level() == Inherit::get_maximum_level())
			return false ;

		Dart d1 = Inherit::phi1(d);
		unsigned int cur = Inherit::get_current_level();
		Inherit::set_current_level(cur + 1);
		Dart d1_l = Inherit::phi1(d);
		Inherit::set_current_level(cur);
		if(d1 != d1_l)
			return true;
		else
			return false;
	}

	/**
	 * Return true if the edge of d in the current level map
	 * is subdivided to the next level,
	 * none of its resulting edges is in turn subdivided to the next level
	 * and the middle vertex is of degree 2
	 */
	bool edge_can_be_coarsened(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		bool subd = false ;
		bool subdOnce = true ;
		bool degree2 = false ;
		if(edge_is_subdivided(d))
		{
			subd = true ;
			Dart d2 = Inherit::phi2(d) ;
			unsigned int cur = Inherit::get_current_level();
			Inherit::set_current_level(cur + 1);
//			if(Inherit::vertexDegree(Inherit::phi1(d)) == 2)
//			{
//				degree2 = true ;
//				if(edge_is_subdivided(d) || edge_is_subdivided(d2))
//					subdOnce = false ;
//			}
			Inherit::set_current_level(cur);

		}
		return subd && degree2 && subdOnce ;
	}

	/**
	 * Return true if the face of d in the current level map
	 * has already been subdivided to the next level
	 */
	bool face_is_subdivided(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		unsigned int fLevel = face_level(d) ;
		if(fLevel <= Inherit::get_current_level())
			return false ;

		bool subd = false ;

		unsigned int cur = Inherit::get_current_level();
		Inherit::set_current_level(cur + 1);
		if(Inherit::get_dart_level(Inherit::phi1(d)) == Inherit::get_current_level()
				&& Inherit::get_edge_id(Inherit::phi1(d)) != Inherit::get_edge_id(d))
			subd = true;
		Inherit::set_current_level(cur);

		return subd;
	}

	/**
	 * Return true if the face of d in the current level map
	 * is subdivided to the next level
	 * and none of its resulting faces is in turn subdivided to the next level
	 * \details
	 * A face whose level in the current level map is lower than the current
	 * level can not be subdivided to higher levels
	 */
	bool face_is_subdivided_once(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "Access to a dart introduced after current level");

		unsigned int fLevel = face_level(d);
		if(fLevel < Inherit::get_current_level())
			return false;

		unsigned int degree = 0 ;
		bool subd = false ;
		bool subdOnce = true ;
		Dart fit = d ;
		do
		{
			unsigned int cur = Inherit::get_current_level();
			Inherit::set_current_level(cur + 1);
			if(Inherit::get_dart_level(Inherit::phi1(fit)) == Inherit::get_current_level()
					&& Inherit::get_edge_id(Inherit::phi1(fit)) != Inherit::get_edge_id(fit))
			{
				subd = true ;
				unsigned int cur2 = Inherit::get_current_level();
				Inherit::set_current_level(cur2 + 1);
				if(Inherit::get_dart_level(Inherit::phi1(fit)) == Inherit::get_current_level()
						&& Inherit::get_edge_id(this->phi1(fit)) != Inherit::get_edge_id(fit))
					subdOnce = false ;
				Inherit::set_current_level(cur2);
			}
			Inherit::set_current_level(cur);
			++degree ;
			fit = Inherit::phi1(fit) ;
		} while(subd && subdOnce && fit != d) ;

		if(degree == 3 && subd)
		{
			unsigned int cur = Inherit::get_current_level();
			Inherit::set_current_level(cur + 1);
			Dart cf = Inherit::phi2(Inherit::phi1(d)) ;
			unsigned int cur2 = Inherit::get_current_level();
			Inherit::set_current_level(cur2 + 1);
			if(Inherit::get_dart_level(Inherit::phi1(cf)) == Inherit::get_current_level()
					&& Inherit::get_edge_id(Inherit::phi1(cf)) != Inherit::get_edge_id(cf))
				subdOnce = false ;
			Inherit::set_current_level(cur2);
			Inherit::set_current_level(cur);
		}

		return subd && subdOnce ;
	}

protected:
	/***************************************************
	 *               SUBDIVISION                       *
	 ***************************************************/

	/**
	 * subdivide the edge of d to the next level
	 */
	void subdivide_edge(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "subdivideEdge : called with a dart inserted after current level") ;
		cgogn_message_assert(!edge_is_subdivided(d), "Trying to subdivide an already subdivided edge");

		unsigned int eLevel = edge_level(d);

		unsigned int cur = Inherit::get_current_level();
		Inherit::set_current_level(eLevel);

		Dart dd = Inherit::phi2(d);

		Inherit::set_current_level(eLevel + 1);

		this->cut_edge_topo(d);// previously : Inherit::cut_edge(d); TODO : write cut_edge for ihcmap2
		unsigned int eId = Inherit::get_edge_id(d);
		Inherit::set_edge_id(Inherit::phi1(d), eId);
		Inherit::set_edge_id(Inherit::phi1(dd), eId);

		//		if(Inherit::template is_orbit_embedded<Orbit::PHI21>())
		//		{
		//            (*edgeVertexFunctor)(Inherit::phi1(d));
		//		}

		//	    //quid des autres cellules ?
		//	    (*edgeEdgeFunctor)(Inherit::phi1(d));
		//		(*edgeEdgeFunctor)(d);


		Inherit::set_current_level(cur);
	}

	/**
	 * coarsen the edge of d from the next level
	 */
	void coarsen_edge(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "coarsenEdge : called with a dart inserted after current level");
		cgogn_message_assert(edge_can_be_coarsened(d), "Trying to coarsen an edge that can not be coarsened");


		unsigned int cur = Inherit::get_current_level();
		//	Dart e = Inherit::phi2(d);
		Inherit::set_current_level(cur + 1);
		//	unsigned int dl = Inherit::get_dart_level(e);
		//	Inherit::set_dart_level(Inherit::phi1(e), dl);
		//	Inherit::collapseEdge(e);
//		Inherit::uncut_edge(d);
		Inherit::set_current_level(cur);
	}

public:
	/**
	 * subdivide the face of d to the next level
	 */
	unsigned int subdivide_face(Dart d, bool triQuad = true, bool OneLevelDifference = true)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "coarsenEdge : called with a dart inserted after current level");
		cgogn_message_assert(!face_is_subdivided(d), "Trying to coarsen an edge that can not be coarsened");

		unsigned int fLevel = face_level(d);
		Dart old = face_oldest_dart(d);

		unsigned int cur = Inherit::get_current_level();
		Inherit::set_current_level(fLevel);		// go to the level of the face to subdivide its edges

		unsigned int degree = 0;
		Dart it = old;
		do
		{
			++degree;						// compute the degree of the face

			if(OneLevelDifference)
			{
				Dart nf = Inherit::phi2(it);
				if(face_level(nf) == fLevel - 1)	// check if neighboring faces have to be subdivided first
					subdivide_face(nf,triQuad);
			}

			if(!edge_is_subdivided(it))
				subdivide_edge(it);			// and cut the edges (if they are not already)
			it = Inherit::phi1(it);
		} while(it != old);

		Inherit::set_current_level(fLevel + 1);	// go to the next level to perform face subdivision

		if((degree == 3) && triQuad)					// if subdividing a triangle
		{
			Dart dd = Inherit::phi1(old);
			Dart e = Inherit::phi1(dd);
			//            (*vertexVertexFunctor)(e) ;

			e = Inherit::phi1(e);
			this->split_face_topo(dd,e); // previously Inherit::split_face(dd, e); TODO : write split_face for ihcmap2

			unsigned int id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
			Inherit::set_edge_id(Inherit::phi_1(dd), id);		// set the edge id of the inserted
			Inherit::set_edge_id(Inherit::phi_1(e), id);		// edge to the next available id

			dd = e ;
			e = Inherit::phi1(dd);
			//            (*vertexVertexFunctor)(e);
			e = Inherit::phi1(e);
			this->split_face_topo(dd,e); // previously : Inherit::split_face(dd, e); TODO : write split_face for ihcmap2
			id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
			Inherit::set_edge_id(Inherit::phi_1(dd), id);
			Inherit::set_edge_id(Inherit::phi_1(e), id);

			dd = e ;
			e = Inherit::phi1(dd);
			//            (*vertexVertexFunctor)(e);
			e = Inherit::phi1(e);
			this->split_face_topo(dd,e);// previously : Inherit::split_face(dd, e); TODO : write split_face for ihcmap2
			id = Inherit::get_tri_refinement_edge_id(Inherit::phi_1(Inherit::phi_1(dd)), Inherit::phi1(Inherit::phi_1(dd)));
			Inherit::set_edge_id(Inherit::phi_1(dd), id);
			Inherit::set_edge_id(Inherit::phi_1(e), id);
		}
		else											// if subdividing a polygonal face
		{
			Dart dd = Inherit::phi1(old);
			Dart next = this->phi1(dd);
			//            (*vertexVertexFunctor)(next);
			next = Inherit::phi1(next);
			this->split_face_topo(dd,next);// previously : Inherit::split_face(dd, next); TODO : write split_face for ihcmap2 // insert a first edge
			Dart ne = Inherit::phi2(Inherit::phi_1(dd));
			Dart ne2 = Inherit::phi2(ne);
			this->cut_edge_topo(ne);// previously : Inherit::cut_edge(ne); TODO : write cut_edge for ihcmap2// cut the new edge to insert the central vertex

			unsigned int id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(Inherit::phi2(ne)));
			Inherit::set_edge_id(ne, id);
			Inherit::set_edge_id(Inherit::phi2(ne), id);			// set the edge id of the inserted

			id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(ne2));
			Inherit::set_edge_id(ne2, id);					// edges to the next available ids
			Inherit::set_edge_id(Inherit::phi2(ne2), id);


			dd = Inherit::phi1(next);
			//            (*vertexVertexFunctor)(dd);
			dd = Inherit::phi1(dd);
			while(dd != ne)								// turn around the face and insert new edges
			{											// linked to the central vertex
				this->split_face_topo(Inherit::phi1(ne), dd);// previously : Inherit::split_face(Inherit::phi1(ne), dd); TODO : write split_face for ihcmap2
				Dart nne = Inherit::phi2(Inherit::phi_1(dd));

				id = Inherit::get_quad_refinement_edge_id(Inherit::phi1(Inherit::phi2(nne)));
				Inherit::set_edge_id(nne, id) ;
				Inherit::set_edge_id(Inherit::phi2(nne), id);

				dd = Inherit::phi1(dd);
				//                (*vertexVertexFunctor)(dd);
				dd = Inherit::phi1(dd);
			}

			//            (*faceVertexFunctor)(Inherit::phi1(ne));
		}

		Inherit::set_current_level(cur);

		return fLevel ;
	}

	/**
	 * coarsen the face of d from the next level
	 */
	void coarsen_face(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(),
							 "coarsenEdge : called with a dart inserted after current level");
		cgogn_message_assert(face_is_subdivided_once(d), "Trying to coarsen an edge that can not be coarsened");

		unsigned int cur = Inherit::get_current_level();

		unsigned int degree = 0;
		Dart fit = d;
		do
		{
			++degree;
			fit = Inherit::phi1(fit);
		} while(fit != d);

		if(degree == 3)
		{
			fit = d ;
			do
			{
				Inherit::set_current_level(cur + 1);
				Dart innerEdge = Inherit::phi1(fit);
				Inherit::set_current_level(Inherit::get_maximum_level());
//				Inherit::merge_faces(innerEdge);
				Inherit::set_current_level(cur);
				fit = this->phi1(fit);
			} while(fit != d);
		}
		else
		{
			Inherit::set_current_level(cur + 1);
			Dart centralV = Inherit::phi1(Inherit::phi1(d));
			Inherit::set_current_level(Inherit::get_maximum_level());
//			Inherit::delete_vertex(centralV);
			Inherit::set_current_level(cur);
		}

		fit = d ;
		do
		{
			if(edge_can_be_coarsened(fit))
				coarsen_edge(fit);
			fit = Inherit::phi1(fit);
		} while(fit != d);
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(MULTIRESOLUTION_CPH_IHCMAP2_ADAPTIVE_CPP_))
extern template class CGOGN_MULTIRESOLUTION_API IHCMap2Adaptive<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(MULTIRESOLUTION_CPH_IHCMAP2_ADAPTIVE_CPP_))


} // namespace cgogn

#endif // MULTIRESOLUTION_CPH_IHCMAP2_ADAPTIVE_H_

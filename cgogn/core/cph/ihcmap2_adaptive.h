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

#ifndef CORE_CPH_IHCMAP2_ADAPTIVE_H_
#define CORE_CPH_IHCMAP2_ADAPTIVE_H_

#include <core/cph/ihcmap2.h>

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
	inline ~IHCMap2Adaptive() = default;

public:
   /***************************************************
     *               CELLS INFORMATION                 *
     ***************************************************/

    /**
     * Return the level of the edge of d in the current level map
     */
    unsigned int edge_level(Dart d)
    {
	    cgogn_message_assert(Inherit::getDartLevel(d) <= Inherit::getCurrentLevel(), "Access to a dart introduced after current level");
	    unsigned int ld = Inherit::getDartLevel(d);
	//	unsigned int ldd = m_dartLevel[phi2(d)] ;	// the level of an edge is the maximum of the
	    unsigned int ldd = Inherit::getDartLevel(Inherit::phi1(d));
	    return ld < ldd ? ldd : ld ;
    }

    /**
     * Return the level of the face of d in the current level map
     */
    unsigned int face_level(Dart d)
    {
	    cgogn_message_assert(Inherit::getDartLevel(d) <= Inherit::getCurrentLevel(), "Access to a dart introduced after current level") ;

	    if(Inherit::getCurrentLevel() == 0)
	        return 0 ;
    }

    /**
     * Given the face of d in the current level map,
     * return a level 0 dart of its origin face
     */
    Dart face_origin(Dart d)
    {
	    cgogn_message_assert(Inherit::getDartLevel(d) <= Inherit::getCurrentLevel(), "Access to a dart introduced after current level") ;
	    unsigned int cur = Inherit::getCurrentLevel() ;
	    Dart p = d ;
	    unsigned int pLevel = Inherit::getDartLevel(p) ;
	    while(pLevel > 0)
	    {
	        p = faceOldestDart(p) ;
	        pLevel = Inherit::getDartLevel(p) ;
	        Inherit::setCurrentLevel(pLevel) ;
	    }
	    Inherit::setCurrentLevel(cur) ;
	    return p ;
    }

    /**
     * Return the oldest dart of the face of d in the current level map
     */
    Dart face_oldest_dart(Dart d)
    {

    }

    /**
     * Return true if the edge of d in the current level map
     * has already been subdivided to the next level
     */
    bool edge_is_subdivided(Dart d)
    {
	    cgogn_message_assert(Inherit::getDartLevel(d) <= Inherit::getCurrentLevel(), "Access to a dart introduced after current level") ;

	    if(Inherit::getCurrentLevel() == Inherit::getMaxLevel())
	        return false ;

	//	Dart d2 = Inherit::phi2(d) ;
	    Dart d1 = Inherit::phi1(d) ;
	    Inherit::incCurrentLevel() ;
	//	Dart d2_l = phi2(d) ;
	    Dart d1_l = Inherit::phi1(d) ;
	    Inherit::decCurrentLevel();
	    if(d1 != d1_l)
	        return true ;
	    else
	        return false ;
    }

    /**
     * Return true if the edge of d in the current level map
     * is subdivided to the next level,
     * none of its resulting edges is in turn subdivided to the next level
     * and the middle vertex is of degree 2
     */
    bool edge_can_be_coarsened(Dart d) ;

    /**
     * Return true if the face of d in the current level map
     * has already been subdivided to the next level
     */
    bool face_is_subdivided(Dart d) ;

    /**
     * Return true if the face of d in the current level map
     * is subdivided to the next level
     * and none of its resulting faces is in turn subdivided to the next level
     */
    bool face_is_subdivided_once(Dart d) ;

protected:
	/***************************************************
	 *               SUBDIVISION                       *
	 ***************************************************/

	/**
	 * subdivide the edge of d to the next level
	 */
	void subdivideEdge(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(), "subdivideEdge : called with a dart inserted after current level") ;
	    cgogn_message_assert(!edge_is_subdivided(d), "Trying to subdivide an already subdivided edge") ;

	    unsigned int eLevel = edge_level(d) ;

		unsigned int cur = Inherit::get_current_level() ;
		Inherit::set_current_level(eLevel) ;

		Dart dd = Inherit::phi2(d) ;

		Inherit::set_current_level(eLevel + 1) ;

	    Inherit::cut_edge(d) ;
		unsigned int eId = Inherit::get_edge_id(d) ;
		Inherit::set_edge_id(Inherit::phi1(d), eId) ;
		Inherit::set_edge_id(Inherit::phi1(dd), eId) ;
	    
		if(Inherit::template is_orbit_embedded<Orbit::PHI21>())
		{
		    (*edgeVertexFunctor)(Inherit::phi1(d)) ;
		}

	    //quid des autres cellules ?
	    (*edgeEdgeFunctor)(Inherit::phi1(d));
		(*edgeEdgeFunctor)(d);


		Inherit::set_current_level(cur) ;
	}

	/**
	 * coarsen the edge of d from the next level
	 */
	void coarsenEdge(Dart d)
	{
		cgogn_message_assert(Inherit::get_dart_level(d) <= Inherit::get_current_level(), "coarsenEdge : called with a dart inserted after current level") ;
	    cgogn_message_assert(edge_can_be_coarsened(d), "Trying to coarsen an edge that can not be coarsened") ;


		unsigned int cur = Inherit::get_current_level() ;
	//	Dart e = Inherit::phi2(d) ;
		Inherit::set_current_level(cur + 1) ;
	//	unsigned int dl = Inherit::get_dart_level(e) ;
	//	Inherit::set_dart_level(Inherit::phi1(e), dl) ;
	//	Inherit::collapseEdge(e) ;
		Inherit::uncut_edge(d) ;
		Inherit::set_current_level(cur) ;
	}

public:
	/**
	 * subdivide the face of d to the next level
	 */
	unsigned int subdivideFace(Dart d, bool triQuad = true, bool OneLevelDifference = true);

	/**
	 * coarsen the face of d from the next level
	 */
	void coarsenFace(Dart d) ;


};

} // namespace cgogn

#endif // CORE_CPH_IHCMAP2_ADAPTIVE_H_

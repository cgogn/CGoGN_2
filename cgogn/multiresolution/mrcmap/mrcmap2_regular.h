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

#ifndef MULTIRESOLUTION_MRCMAP_MRCMAP2_REGULAR_H_
#define MULTIRESOLUTION_MRCMAP_MRCMAP2_REGULAR_H_


#include <multiresolution/mrcmap/mrcmap2.h>

namespace cgogn
{

template <typename MAP_TRAITS>
class MRCMap2Regular : public MRCMap2_T<MAP_TRAITS>
{
public:

    typedef MRCMap2<MAP_TRAITS> Inherit;
    typedef MRCMap2Regular<MAP_TRAITS> Self;


    MRCMap2Regular() : Inherit()
    {}

    MRCMap2Regular(const Self&) = delete;
    MRCMap2Regular(Self&&) = delete;
    Self& operator=(const Self&) = delete;
    Self& operator=(Self&&) = delete;
    inline ~MRCMap2Regular() = default;

protected:
    inline Vertex cut_edge(Edge e)
    {
        Inherit::current()->cut_edge(e);
    }

    inline void split_face(Vertex d, Vertex e)
    {
        Inherit::current()->split_face(d, e);
    }

public:

    inline void add_triangular_level()
    {
        Inherit::push_level();

        Inherit::add_level_back();

        Inherit::set_current_level(Inherit::get_maximum_level());

        Inherit::template foreach_cell<Inherit::EDGE, TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Edge e)
        {
            cut_edge(e);

            //skip the new edges
        });

        //cut faces
        Inherit::template foreach_cell<Inherit::FACE, TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Face d)
        {
            Dart old = d ;

            if(Inherit::get_dart_level(old) == Inherit::get_maximum_level())
                old = Inherit::phi1(old) ;

            Dart dd = Inherit::phi1(old) ;
            Dart e = Inherit::phi1(Inherit::phi1(dd)) ;
            // insert a new edge
            split_face(dd, e) ;

            dd = e ;
            e = Inherit::phi1(Inherit::phi1(dd)) ;
            split_face(dd, e) ;

            dd = e ;
            e = Inherit::phi1(Inherit::phi1(dd)) ;
            split_face(dd, e) ;

            //skip the new faces
        });

        Inherit::pop_level();
    }

    inline void add_quadrangular_level()
    {
        Inherit::push_level();

        Inherit::add_level_back();

        Inherit::set_current_level(Inherit::get_maximum_level());

        Inherit::template foreach_cell<Inherit::EDGE, TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Edge e)
        {
            cut_edge(e);

            //skip the new edges
        });

        //cut faces
        Inherit::template foreach_cell<Inherit::FACE, TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Face d)
        {
            Dart old = d ;

            if(Inherit::get_dart_level(old) == Inherit::get_maximum_level())
                old = Inherit::phi1(old) ;

            Dart dd = Inherit::phi1(old) ;
            Dart next = Inherit::phi1(Inherit::phi1(dd)) ;
            split_face(dd, next) ;		// insert a first edge

            Dart ne = Inherit::phi2(Inherit::phi_1(dd)) ;
            cut_edge(ne) ;				// cut the new edge to insert the central vertex

            dd = Inherit::phi1(Inherit::phi1(next)) ;
            while(dd != ne)				// turn around the face and insert new edges
            {							// linked to the central vertex
                Dart tmp = Inherit::phi1(ne) ;
                split_face(tmp, dd) ;
                dd = Inherit::phi1(Inherit::phi1(dd)) ;
            }
        });

        Inherit::pop_level();
    }

    inline void add_mixed_level()
    {

    }
};

}

#endif // MULTIRESOLUTION_MRCMAP_MRCMAP2_REGULAR_H_

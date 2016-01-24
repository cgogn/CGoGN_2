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

#ifndef MR_MRCMAP2_PRIMAL_ADAPTIVE_H_
#define MR_MRCMAP2_PRIMAL_ADAPTIVE_H_

#include <core/mrcmap/mrcmap2.h>

namespace cgogn
{

template <typename MAP_TRAITS>
class MRCMap2PrimalAdaptive
{
public:
	typedef MRCMap2<MAP_TRAITS> MRCMap;
	typedef MRCMap2PrimalAdaptive<MAP_TRAITS> Self;

protected:
	MRCMap& mrcmap_;

	FunctorType* vertex_vertex_;
	FunctorType* edge_vertex_;
	FunctorType* face_vertex_;

public:
	Map2MRAdapt(MRCMap& mrcmap) : 
		mrcmap_(mrcmap)
	{}

	~Map2MRAdapt()
	{}

	MRCMap2PrimalAdaptive(Self const&) = delete;
	MRCMap2PrimalAdaptive(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

	unsigned int edge_level(Edge d)
	{
		cgogn_message_assert(mrcmap_.getDartLevel(d) <= mrcmap_.getCurrentLevel(), "edgeLevel : called with a dart inserted after current level") ;

		// the level of an edge is the maximum of the
		// level of its darts
		unsigned int ld = mrcmap_.getDartLevel(d) ;
		unsigned int ldd = mrcmap_.getDartLevel(mrcmap_.phi2(d)) ;
		return ld > ldd ? ld : ldd ;
	}

	void set_vertex_vertex_functor(FunctorType* f) 
	{ 
		vertex_vertex_ = f ; 
	}

	void set_edge_vertex_functor(FunctorType* f) 
	{ 
		edge_vertex_ = f ; 
	}
	
	void set_face_vertex_functor(FunctorType* f) 
	{ 
		face_vertex_ = f ; 
	}

protected:

	void subdivide_edge(Edge e)
	{

	}

	Vertex cut_edge(Edge e)
	{
		mrcmap_.cut_edge(e);
	}

public:

	void subdivide_face(Face f)
	{

	}

	void coarsen_face(Face f)
	{

	}
};

} // namespace cgogn

#endif // MR_MRCMAP2_PRIMAL_ADAPTIVE_H_

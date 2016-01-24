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

#ifndef MR_MRCMAP2_PRIMAL_REGULAR_H_
#define MR_MRCMAP2_PRIMAL_REGULAR_H_

#include <core/mrcmap/mrcmap2.h>

namespace cgogn
{

template <typename MAP_TRAITS>
class MRCMap2PrimalRegular
{
public:
	typedef MRCMap2<MAP_TRAITS> MRCMap;
	typedef MRCMap2PrimalRegular<MAP_TRAITS> Self;


protected:
	MRCMap& mrcmap_;

	std::vector<MRFilter*> synthesis_filters_;
	std::vector<MRFilter*> analysis_filters_;

public:
	MRCMap2PrimalRegular(MRCMap& mrcmap) : 
		mrcmap_(mrcmap)
	{}

	~MRCMap2PrimalRegular()
	{
		unsigned int level = mrcmap_.get_current_level();
		unsigned int max_level = mrcmap_.get_maximum_level();

		for(unsigned int i = max_level ; i > level ; --i)
			mrcmap_.remove_level_back();

		for(unsigned int i = 0 ; i < level ; ++i)
			mrcmap_.remove_level_front();
	}

	MRCMap2PrimalRegular(Self const&) = delete;
	MRCMap2PrimalRegular(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

	void add_level(bool do_tri_quad)
	{
		mrcmap_.push_level();

		//1. add a nex level back as a copy of previous level;
		mrcmap_.add_level_back();

		//cut edges
		mrcmap_.foreach_cell<Inherit::Edge, TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Edge e)
		{
			mrcmap_.cut_edge(e);
			//skip the two resulting edges d and phi1(d) ?
			//the new vertex should automatically be embedded
		});

		// split faces
		mrcmap_.foreach_cell<Inherit::Face, TraversalStrategy::FORCE_DART_MARKING>([&] (typename Inherit::Face f)
		{
			//TODO search the oldest dart of the face
			typename Inherit::xFace old = f;
			if(mrcmap_.get_dart_level(old) == mrcmap_.get_maximum_level())
				old = mrcmap_.phi1(old);

			//compute the degree of the face
			mrcmap_.dec_current_level();
			unsigned int degree = mrcmap_.degree(f);
			mrcmap_.inc_current_level();

			if(do_tri_quad && (degree == 3))
			{
				Dart dd = mrcmap_.phi1(old);
				Dart e = mrcmap_.phi1(mrcmap_.phi1(dd));
				mrcmap_.split_face(dd, e);
				//skip dd ?

				dd = e ;
				e = mrcmap_.phi1(mrcmap_.phi1(dd)) ;
				mrcmap_.split_face(dd, e) ;
				//skip dd ?

				dd = e ;
				e = mrcmap_.phi1(mrcmap_.phi1(dd)) ;
				mrcmap_.split_face(dd, e) ;
				//skip dd ?
				//and skip e ?
			}
			else
			{

			}
		});

		mrcmap_.pop_level();
	}

	void add_synthesis_filter(Algo::MR::Filter* f) 
	{ 
		synthesis_filters_.push_back(f); 
	}

	void add_analysis_filter(Algo::MR::Filter* f) 
	{ 
		analysis_filters_.push_back(f); 
	}

	void clear_synthesis_filters() 
	{ 
		synthesis_filters_.clear(); 
	}
	
	void clear_analysis_filters() 
	{ 
		analysis_filters_.clear(); 
	}

	void analysis()
	{
		cgogn_message_assert(mrcmap_.getCurrentLevel() > 0, "analysis : called on level 0") ;

		mrcmap_.decCurrentLevel() ;

		for(unsigned int i = 0; i < analysis_filters_.size(); ++i)
			(*analysis_filters_[i])() ;
	}

	void synthesis()
	{
		cgogn_message_assert(mrcmap_.getCurrentLevel() < mrcmap_.getMaxLevel(), "synthesis : called on max level") ;

		for(unsigned int i = 0; i < synthesis_filters_.size(); ++i)
			(*synthesis_filters_[i])() ;

		mrcmap_.incCurrentLevel() ;
	}

};

} // namespace cgogn

#endif // MR_MRCMAP2_REGULAR_H_

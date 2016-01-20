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

class Map2MR
{

protected:
	MRCMap2& mrmap_;

	std::vector<MRFilter*> synthesis_filters_;
	std::vector<MRFilter*> analysis_filters_;

public:
	Map2MR(MRCMap2& mrmap) : 
		mrmap_(mrmap)
	{}

	~Map2MR()
	{
		unsigned int level = mrmap_.get_current_level();
		unsigned int max_level = mrmap_.get_maximum_level();

		for(unsigned int i = max_level ; i > level ; --i)
			mrmap_.remove_level_back();

		for(unsigned int i = 0 ; i < level ; ++i)
			mrmap_.remove_level_front();
	}

	// level management
	void add_new_level(bool do_tri_quad)
	{
		mrmap_.push_level();

		//1. add a nex level back as a copy of previous level;
		mrmap_.add_level_back();

		//cut edges
		mrmap_.foreach_cell<MRCMap2::Edge>([&] (MRCMap2::Edge e)
		{
			mrmap_.cut_edge(e);
			//skip the two resulting edges
			//the new vertex should automatically be embedded
		});

		// split faces
		mrmap_.foreach_cell<MRCMap2::Face>([&] (MRCMap2::Face f)
		{
			//TODO search the oldest dart of the face
			Face old = f;
			if(mrmap_.get_dart_level(old) == mrmap_.get_maximum_level())
				old = mrmap_.phi1(old);

			//compute the degree of the face
			mrmap_.dec_current_level();
			unsigned int degree = mrmap_.degree(f);
			mrmap_.inc_current_level();

			if(do_tri_quad && (degree == 3))
			{
				Dart dd = mrmap_.phi1(old);
				Dart e = mrmap_.phi1(mrmap_.phi1(dd));
				mrmap_.split_face(dd, e);
			}
			else
			{

			}
		});

		mrmap_.pop_level();
	}


	//
	void add_synthesis_filter(MRFilter* f) 
	{ 
		synthesis_filters_.push_back(f); 
	}
	
	void add_analysis_filter(MRFilter* f) 
	{
		analysis_filters_.push_back(f); 
	}

	void clear_synthesis_filter() 
	{ 
		synthesis_filters_.clear(); 
	}
	
	void clear_analysis_filter() 
	{
		analysis_filters_.clear(); 
	}

	void synthesis()
	{
		cgogn_message_assert(mrmap_.get_current_level() < mrmap_.get_maximum_level(), "synthesis : called on max level") ;

		for(unsigned int i = 0; i < synthesis_filters_.size(); ++i)
			(*synthesis_filters_[i])() ;

		mrmap_.inc_current_level() ;
	}

	void analysis()
	{
		cgogn_message_assert(mrmap_.get_current_level() > 0, "analysis : called on level 0") ;

		mrmap_.dec_current_level() ;

		for(unsigned int i = 0; i < analysis_filters_.size(); ++i)
			(*analysis_filters_[i])() ;
	}

};

}

#endif // MR_MRCMAP2_REGULAR_H_
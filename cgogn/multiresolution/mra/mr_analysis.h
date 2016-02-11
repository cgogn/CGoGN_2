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

#ifndef MULTIRESOLUTION_MRA_MR_ANALYSIS_H_
#define MULTIRESOLUTION_MRA_MR_ANALYSIS_H_

namespace cgogn {

template <typename MRMAP>
class MRAnalysis
{
public:
	typedef MRAnalysis<MRMAP> Self;


protected:
	MRMAP& map_;

	std::vector<std::function<void()>> synthesis_filters_;
	std::vector<std::function<void()>> analysis_filters_;

public:
	MRAnalysis(MRMAP& map):
		map_(map)
	{}

	MRAnalysis(Self const& ) = delete;
	MRAnalysis(Self&& ) = delete;
	MRAnalysis& operator=(Self const& ) = delete;
	MRAnalysis& operator=(Self&& ) = delete;

	virtual ~MRAnalysis()
	{}

	void analysis()
	{
		cgogn_message_assert(map_.get_current_level() > 0, "analysis : called on level 0") ;

		map_.dec_current_level() ;

		for(unsigned int i = 0; i < analysis_filters_.size(); ++i)
			analysis_filters_[i]();
	}

	void synthesis()
	{
		cgogn_message_assert(map_.get_current_level() < map_.get_maximum_level(), "synthesis : called on max level") ;

		for(unsigned int i = 0; i < synthesis_filters_.size(); ++i)
			synthesis_filters_[i]();

		map_.inc_current_level();
	}

	virtual void add_level() = 0;
};

} //namespace cgogn

#endif // MULTIRESOLUTION_MRA_MR_ANALYSIS_H_

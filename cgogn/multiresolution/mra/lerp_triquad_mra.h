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

#ifndef MULTIRESOLUTION_MRA_LERP_TRI_QUAD_MRANALYSIS_H_
#define MULTIRESOLUTION_MRA_LERP_TRI_QUAD_MRANALYSIS_H_

#include <core/basic/dart.h>
#include <multiresolution/mra/mr_analysis.h>

namespace cgogn {

template <typename MRMAP, typename VEC3>
class LerpTriQuadMRAnalysis : public MRAnalysis<MRMAP>
{

public:
	typedef LerpTriQuadMRAnalysis<MRMAP, VEC3> Self;
	typedef MRAnalysis<MRMAP> Inherit;

	using VertexAttributeHandler = typename MRMAP::template VertexAttributeHandler<VEC3>;

protected:
	VertexAttributeHandler& va_;

public:
	LerpTriQuadMRAnalysis(MRMAP& map, VertexAttributeHandler& v):
		Inherit(map),
		va_(v)
	{
		this->synthesis_filters_.push_back(lerp_tri_quad_odd_synthesis_);
	}

	LerpTriQuadMRAnalysis(Self const& ) = delete;
	LerpTriQuadMRAnalysis(Self&& ) = delete;
	LerpTriQuadMRAnalysis& operator=(Self const& ) = delete;
	LerpTriQuadMRAnalysis& operator=(Self&& ) = delete;

	~LerpTriQuadMRAnalysis() override
	{}

protected:

	std::function<void()> lerp_tri_quad_odd_synthesis_ = [this] ()
	{
		this->map_.foreach_cell([&] (typename MRMAP::Face f)
		{
			if(this->map_.degree(f) != 3)
			{
				VEC3 vf(0.0);
				VEC3 ef(0.0);

				unsigned int count = 0;

				this->map_.foreach_incident_edge(f, [&] (typename MRMAP::Edge e)
				{
					vf += va_[e.dart];
					this->map_.inc_current_level();
					ef += va_[this->map_.phi1(e.dart)];
					this->map_.dec_current_level();
					++count;
				});

				ef /= count;
				ef *= 2.0;

				vf /= count;

				this->map_.inc_current_level() ;
				Dart midF = this->map_.phi1(this->map_.phi1(f.dart));
				va_[midF] += vf + ef ;
				this->map_.dec_current_level() ;
			}
		});

		this->map_.foreach_cell([&] (typename MRMAP::Edge e)
		{
			VEC3 ve = (va_[e.dart] + va_[this->map_.phi1(e)]) * 0.5;

			this->map_.inc_current_level() ;
			Dart midV = this->map_.phi1(e) ;
			va_[midV] += ve ;
			this->map_.dec_current_level() ;
		});
	};

public:

	void add_level() override
	{
		this->map_.add_mixed_level();
	}

};

} //namespace cgogn

#endif // MULTIRESOLUTION_MRA_LERP_TRI_QUAD_MRANALYSIS_H_


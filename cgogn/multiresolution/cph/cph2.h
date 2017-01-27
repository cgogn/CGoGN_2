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

#ifndef CGOGN_MULTIRESOLUTION_CPH_CPH2_BASE_H_
#define CGOGN_MULTIRESOLUTION_CPH_CPH2_BASE_H_

#include <cgogn/multiresolution/cph/cph_base.h>

namespace cgogn
{

class CPH2 : public CPHBase
{
public:

	using Self = CPH2;
	using Inherit = CPHBase;

	template <typename T>
	using ChunkArrayContainer =  typename Inherit::template ChunkArrayContainer<T>;
	template <typename T>
	using ChunkArray =  typename Inherit::template ChunkArray<T>;

protected:

	ChunkArray<uint32>* edge_id_;

public:

	CPH2(ChunkArrayContainer<unsigned char>& topology) : Inherit(topology)
	{
		edge_id_ = topology.template add_chunk_array<uint32>("edgeId");
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CPH2);

	~CPH2() override
	{}

	/***************************************************
	 *             EDGE ID MANAGEMENT                  *
	 ***************************************************/

	inline uint32 get_edge_id(Dart d) const
	{
		return (*edge_id_)[d.index] ;
	}

	inline void set_edge_id(Dart d, uint32 i)
	{
		(*edge_id_)[d.index] = i ;
	}

	uint32 get_tri_refinement_edge_id(Dart d, Dart e) const;

	uint32 get_quad_refinement_edge_id(Dart d) const;
};

} // namespace cgogn

#endif // CGOGN_MULTIRESOLUTION_CPH_CPH2_BASE_H_

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

#ifndef MULTIRESOLUTION_CPH_CPH3_BASE_H_
#define MULTIRESOLUTION_CPH_CPH3_BASE_H_

#include <multiresolution/cph/cph2.h>

namespace cgogn
{

template <typename DATA_TRAITS>
class CPH3 : public CPH2<DATA_TRAITS>
{

public:
	using Self =  CPH3<DATA_TRAITS>;
	using Inherit = CPH2<DATA_TRAITS>;
	template <typename T>
	using ChunkArray =  typename Inherit::template ChunkArray<T>;
	template <typename T>
	using ChunkArrayContainer =  typename Inherit::template ChunkArrayContainer<T>;

protected:
	ChunkArray<unsigned int>* face_id_;

public:
	CPH3(ChunkArrayContainer<unsigned char>& topology): Inherit(topology)
	{
		face_id_ = topology.template add_attribute<unsigned int>("faceId");
	}

	~CPH3() override
	{
		// TODO : check the way the destructors free memory in the class hierarchy
		// this->topo_.remove_attribute(face_id_);
	}

	CPH3(Self const&) = delete;
	CPH3(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

	/***************************************************
	 *             FACE ID MANAGEMENT                  *
	 ***************************************************/

	inline unsigned int get_face_id(Dart d) const
	{
		return (*face_id_)[d.index] ;
	}

	inline void set_face_id(Dart d, unsigned int i)
	{
		(*face_id_)[d.index] = i ;
	}

	inline unsigned int get_tri_refinement_face_id(Dart d, Dart e) const
	{

	}

	inline unsigned int get_quad_refinement_face_id(Dart d) const
	{

	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(MULTIRESOLUTION_CPH_CPH3_CPP_))
extern template class CGOGN_MULTIRESOLUTION_API CPH3<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(MULTIRESOLUTION_CPH_CPH3_CPP_))

} // namespace cgogn

#endif // MULTIRESOLUTION_CPH_CPH3_BASE_H_

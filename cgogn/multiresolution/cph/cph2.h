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

#ifndef MULTIRESOLUTION_CPH_CPH2_BASE_H_
#define MULTIRESOLUTION_CPH_CPH2_BASE_H_

#include <multiresolution/cph/cph_base.h>

namespace cgogn
{

template <typename DATA_TRAITS>
class CPH2 : public CPHBase<DATA_TRAITS>
{

public:

	typedef CPH2<DATA_TRAITS> Self;
	typedef CPHBase<DATA_TRAITS> Inherit;

	template <typename T>
	using ChunkArray =  typename Inherit::template ChunkArray<T>;
	template <typename T>
	using ChunkArrayContainer =  typename Inherit::template ChunkArrayContainer<T>;

protected:

	ChunkArray<unsigned int>* edge_id_;

public:

	CPH2(ChunkArrayContainer<unsigned char>& topology): Inherit(topology)
	{
		edge_id_ = topology.template add_attribute<unsigned int>("edgeId");
	}

	~CPH2() override
	{
		// TODO : check the way the destructors free memory in the class hierarchy
		// this->topo_->remove_attribute(edge_id_);
	}

	CPH2(Self const&) = delete;
	CPH2(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

	/***************************************************
	 *             EDGE ID MANAGEMENT                  *
	 ***************************************************/

	inline unsigned int get_edge_id(Dart d) const
	{
		return (*edge_id_)[d.index] ;
	}

	inline void set_edge_id(Dart d, unsigned int i)
	{
		(*edge_id_)[d.index] = i ;
	}

	inline unsigned int get_tri_refinement_edge_id(Dart d, Dart e) const
	{
		unsigned int d_id = get_edge_id(d);
		unsigned int e_id = get_edge_id(e);

		unsigned int id = d_id + e_id;

		if(id == 0u)
			return 1u;
		else if(id == 1u)
			return 2u;
		else if(id == 2u)
		{
			if(d_id == e_id)
				return 0u;
			else
				return 1u;
		}
		// else if(id == 3)
		return 0u;
	}

	inline unsigned int get_quad_refinement_edge_id(Dart d) const
	{
		unsigned int e_id = get_edge_id(d);

		if(e_id == 0u)
			return 1u;
		// else if(e_id == 1)
		return 0u;
	}

};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(MULTIRESOLUTION_CPH_CPH2_CPP_))
extern template class CGOGN_MULTIRESOLUTION_API CPH2<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(MULTIRESOLUTION_CPH_CPH2_CPP_))

} // namespace cgogn

#endif // MULTIRESOLUTION_CPH_CPH2_BASE_H_

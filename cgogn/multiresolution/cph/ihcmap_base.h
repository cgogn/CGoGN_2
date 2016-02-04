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

#ifndef CORE_CPH_CPH_H_
#define CORE_CPH_CPH_H_

#include <core/container/chunk_array_container.h>
#include <core/utils/assert.h>

namespace cgogn
{

template <typename DATA_TRAITS>
class IHCMapBase
{
	typedef IHCMapBase<DATA_TRAITS> Self;

	template <typename T>
	using ChunkArray =  cgogn::ChunkArray<DATA_TRAITS::CHUNK_SIZE, T>;
    template <typename T>
    using ChunkArrayContainer = cgogn::ChunkArrayContainer<DATA_TRAITS::CHUNK_SIZE, T>;

protected:
	unsigned int current_level_;
	unsigned int maximum_level_;

	// DartAttributeHandler<unsigned int> dart_level_ ;
	// DartAttributeHandler<unsigned int> edge_id_ ;
    ChunkArray<unsigned int>* dart_level_;
    ChunkArray<unsigned int>* edge_id_;

	std::vector<unsigned int> nb_darts_per_level;

    ChunkArrayContainer<unsigned char>& topo_;

public:
    IHCMapBase(ChunkArrayContainer<unsigned char>& topology):
        topo_(topology),
	    current_level_(0),
        maximum_level_(0)
     {
     	init();
     } 

	~IHCMapBase()
	{
        topo_.remove_attribute(dart_level_);
        topo_.remove_attribute(edge_id_);
	}

	IHCMapBase(Self const&) = delete;
	IHCMapBase(Self &&) = delete;
	Self& operator=(Self const&) = delete;
	Self& operator=(Self &&) = delete;

public:

	void init()
	{
        dart_level_ = topo_.template add_attribute<unsigned int>("dartLevel") ;
        edge_id_ = topo_.template add_attribute<unsigned int>("edgeId");
	}

	/***************************************************
	 *              LEVELS MANAGEMENT                  *
	 ***************************************************/

	inline unsigned int get_current_level() const
	{
		return current_level_ ;
	}

	inline void set_current_level(unsigned int l)
	{
		current_level_ = l ;
	}

	inline unsigned int get_maximum_level() const
	{
		return maximum_level_ ;
	}

	inline void set_maximum_level(unsigned int l)
	{
		maximum_level_ = l;
	}

	inline unsigned int get_dart_level(Dart d) const
	{
        return (*dart_level_)[d.index] ;
	}

	inline void set_dart_level(Dart d, unsigned int l)
	{
        (*dart_level_)[d.index] = l ;
	}

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

		//else if(id == 3)
		return 0u;
	}

	inline unsigned int get_quad_refinement_edge_id(Dart d) const
    {
		unsigned int e_id = get_edge_id(d);

		if(e_id == 0u)
			return 1u;

		//else if(e_id == 1)
		return 0u;
	}

	inline void inc_nb_darts(unsigned int level)
	{
		cgogn_message_assert(level < get_maximum_level(), "inc_nb_darts : already at maximum resolution level");
		nb_darts_per_level[level]++;
	}

	inline void new_level_darts()
	{
		nb_darts_per_level.push_back(0);
	}

};

} // namespace cgogn


#endif // CORE_CPH_CPH_H_

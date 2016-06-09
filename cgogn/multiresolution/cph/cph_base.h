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

#ifndef CGOGN_MULTIRESOLUTION_CPH_CPH_BASE_H_
#define CGOGN_MULTIRESOLUTION_CPH_CPH_BASE_H_

#include <cgogn/core/utils/assert.h>
#include <cgogn/core/basic/dart.h>
#include <cgogn/core/container/chunk_array_container.h>
#include <cgogn/core/cmap/map_traits.h>

#include <cgogn/multiresolution/dll.h>

namespace cgogn
{

template <typename DATA_TRAITS>
class CPHBase
{
public:

	using Self = CPHBase<DATA_TRAITS>;

	template <typename T>
	using ChunkArray = cgogn::ChunkArray<DATA_TRAITS::CHUNK_SIZE, T>;
	template <typename T>
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<DATA_TRAITS::CHUNK_SIZE, T>;

protected:

	uint32 current_level_;
	uint32 maximum_level_;

	/*!
	 * \brief Store the current number of darts per resolution level.
	 * This information is used to detect that a level of resolution
	 * becomes empty (contains no more dart) and that the maximum_level
	 * of the hierarchy should be decremented.
	 */
	std::vector<uint32> nb_darts_per_level_;

	ChunkArray<uint32>* dart_level_;

public:

	inline CPHBase(ChunkArrayContainer<unsigned char>& topology) :
		current_level_(0u),
		maximum_level_(0u)
	{
		nb_darts_per_level_.reserve(32u);
		nb_darts_per_level_.push_back(0);
		dart_level_ = topology.template add_chunk_array<uint32>("dartLevel") ;
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(CPHBase);

	virtual ~CPHBase()
	{
		// TODO : check the way the destructors free memory in the class hierarchy
		// topo_->remove_attribute(dart_level_);
	}

	/***************************************************
	 *              LEVELS MANAGEMENT                  *
	 ***************************************************/

	inline uint32 get_current_level() const
	{
		return current_level_ ;
	}

	inline void set_current_level(uint32 l)
	{
		current_level_ = l ;
	}

	inline uint32 get_maximum_level() const
	{
		return maximum_level_ ;
	}

	inline void set_maximum_level(uint32 l)
	{
		maximum_level_ = l;
	}

	inline uint32 get_dart_level(Dart d) const
	{
		return (*dart_level_)[d.index] ;
	}

	inline void set_dart_level(Dart d, uint32 l)
	{
		(*dart_level_)[d.index] = l ;
	}

	inline void inc_current_level()
	{
		current_level_++;

		while (nb_darts_per_level_.size() < current_level_+1u)
			nb_darts_per_level_.push_back(0);

		if (current_level_ > maximum_level_)
			maximum_level_ = current_level_;
	}

	inline void dec_current_level()
	{
		cgogn_message_assert(current_level_ > 0u, "dec_current_level : already at minimal resolution level");

		if (current_level_ == maximum_level_)
		{
			if (nb_darts_per_level_[current_level_] == 0u)
			{
				maximum_level_--;
				nb_darts_per_level_.pop_back();
			}
		}
		current_level_--;
	}

	inline void inc_nb_darts()
	{
		nb_darts_per_level_[current_level_]++;
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MULTIRESOLUTION_CPH_CPH_BASE_CPP_))
extern template class CGOGN_MULTIRESOLUTION_API CPHBase<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MULTIRESOLUTION_CPH_CPH_BASE_CPP_))

} // namespace cgogn

#endif // CGOGN_MULTIRESOLUTION_CPH_CPH_BASE_H_

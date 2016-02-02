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

#ifndef CORE_CPH_IHCMAP2_REGULAR_H_
#define CORE_CPH_IHCMAP2_REGULAR_H_

#include <core/cph/ihcmap2.h>

namespace cgogn
{

template <typename MAP_TRAITS>
class IHCMap2Regular : IHCMap2<MAP_TRAITS>
{
public:

	typedef IHCMap2<MAP_TRAITS> Inherit;
	typedef IHCMap2Regular<MAP_TRAITS> Self;


	IHCMap2Regular() : Inherit()
	{}

	~IHCMap2Regular() override
	{}

	IHCMap2Regular(const Self&) = delete;
	IHCMap2Regular(Self&&) = delete;
	Self& operator=(const Self&) = delete;
	Self& operator=(Self&&) = delete;
	inline ~IHCMap2Regular() = default;

public:

	inline void add_triangular_level()
	{

	}

	inline void add_quadrangular_level()
	{

	}

	inline void add_mixed_level()
	{

	}
};

} // namespace cgogn

#endif // CORE_CPH_IHCMAP2_REGULAR_H_

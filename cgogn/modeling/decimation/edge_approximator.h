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

#ifndef CGOGN_MODELING_DECIMATION_EDGE_APPROXIMATOR_H_
#define CGOGN_MODELING_DECIMATION_EDGE_APPROXIMATOR_H_

#include <cgogn/geometry/types/geometry_traits.h>

namespace cgogn
{

namespace modeling
{

template <typename MAP, typename VEC3>
class EdgeApproximator
{
public:

	using Edge = typename MAP::Edge;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(EdgeApproximator);

	inline EdgeApproximator(const MAP& m) : map_(m)
	{}

	virtual ~EdgeApproximator()
	{}

	virtual void init() = 0;

	virtual VEC3 operator()(Edge e) const = 0;

protected:

	const MAP& map_;
};

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_DECIMATION_EDGE_APPROXIMATOR_H_

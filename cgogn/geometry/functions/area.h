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

#ifndef CGOGN_GEOMETRY_FUNCTIONS_AREA_H_
#define CGOGN_GEOMETRY_FUNCTIONS_AREA_H_

#include <cgogn/geometry/types/geometry_traits.h>

namespace cgogn
{

namespace geometry
{

/**
 * area of the triangle formed by 3 points in 3D
 */
template <typename VEC3, typename VEC3b, typename VEC3c>
inline auto area(const VEC3& p1, const VEC3b& p2, const VEC3c& p3)
-> typename std::enable_if <(vector_traits<VEC3>::SIZE == 3) && is_same3vector<VEC3, VEC3b, VEC3c>::value, typename vector_traits<VEC3>::Scalar>::type
{
	using Scalar = typename vector_traits<VEC>::Scalar;
	return (Scalar(0.5) * ((p2 - p1).cross(p3 - p1)).norm());
}

/**
 * area of the triangle formed by 3 points in 2D
 */
template <typename VEC2, typename VEC2b, typename VEC2c>
inline auto area(const VEC2& p1, const VEC2b& p2, const VEC2c& p3)
-> typename std::enable_if <(vector_traits<VEC2>::SIZE == 2) && is_same3vector<VEC2, VEC2b, VEC2c>::value, typename vector_traits<VEC3>::Scalar>::type
{
	using Scalar = typename vector_traits<VEC>::Scalar;
	VEC v1 = p2 - p1;
	VEC v2 = p3 - p1;
	return (Scalar(0.5) * (v1[0] * v2[1] - v1[1] * v2[0]));
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_FUNCTIONS_AREA_H_

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
template <typename VEC3a, typename VEC3b, typename VEC3c, bool B=true>
inline auto area(const Eigen::MatrixBase<VEC3a>& p1, const Eigen::MatrixBase<VEC3b>& p2, const Eigen::MatrixBase<VEC3c>& p3)
-> typename std::enable_if <is_dim_of<VEC3a, 3>::value, ScalarOf<VEC3a>>::type
{
	static_assert(is_same_vector<VEC3a,VEC3b,VEC3c>::value, "parameters must have same type");
	using Scalar = ScalarOf<VEC3a>;
	return (Scalar(0.5) * ((p2 - p1).cross(p3 - p1)).norm());
}


/**
 * area of the triangle formed by 3 points in 2D
 */
template <typename VEC2a, typename VEC2b, typename VEC2c, bool B=true>
inline auto area(const Eigen::MatrixBase<VEC2a>& p1, const Eigen::MatrixBase<VEC2b>& p2, const Eigen::MatrixBase<VEC2c>& p3)
-> typename std::enable_if <is_dim_of<VEC2a, 2>::value, ScalarOf<VEC2a>>::type
{
	static_assert(is_same_vector<VEC2a,VEC2b,VEC2c>::value, "parameters must have same type");
	using Scalar = ScalarOf<VEC2a>;
	auto v1 = p2 - p1;
	auto v2 = p3 - p1;
	return (Scalar(0.5) * (v1[0] * v2[1] - v1[1] * v2[0]));
}



/**
 * area of the triangle formed by 3 points (for not-Eigen parameters)
 */
template <typename VEC>
inline auto area(const VEC& p1, const VEC& p2, const VEC& p3)
-> typename std::enable_if <is_vec_non_eigen<VEC>::value, ScalarOf<VEC> >::type
{
	return area(eigenize(p1),eigenize(p2),eigenize(p3));
}



} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_FUNCTIONS_AREA_H_

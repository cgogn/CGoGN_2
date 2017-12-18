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

#ifndef CGOGN_GEOMETRY_FUNCTIONS_BASICS_H_
#define CGOGN_GEOMETRY_FUNCTIONS_BASICS_H_

#include <cmath>
#include <algorithm>

#include <cgogn/geometry/types/geometry_traits.h>

namespace cgogn
{

namespace geometry
{

/**
 * @brief normalize_safe, normalize a non-zero vector
 * @param v
 */
template <typename VEC>
inline void normalize_safe(VEC& v)
{
	using Scalar = typename vector_traits<VEC>::Scalar;

	const Scalar norm2 = v.squaredNorm();
	if (norm2 > Scalar(0))
		v /= std::sqrt(norm2);
}

/**
 * @brief cosinus of the angle formed by 2 vectors
 */
template <typename VEC>
typename VEC::Scalar cos_angle(const VEC& a, const VEC& b)
{
	using Scalar = typename vector_traits<VEC>::Scalar;

	Scalar na2 = a.squaredNorm();
	Scalar nb2 = b.squaredNorm();

	Scalar res = a.dot(b) / std::sqrt(na2 * nb2);
	return std::max(Scalar(-1), std::min(res, Scalar(1)));
}

/**
 * @brief angle formed by 2 vectors
 */
template <typename VEC>
typename VEC::Scalar angle(const VEC& a, const VEC& b)
{
	return std::acos(cos_angle(a, b));
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_FUNCTIONS_BASICS_H_

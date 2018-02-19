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
 * @brief normalize a non-zero vector
 */
template <typename VEC>
inline void normalize_safe(Eigen::MatrixBase<VEC>& v)
{
	using Scalar = ScalarOf<VEC>;
	const Scalar norm2 = v.squaredNorm();
	if (norm2 > Scalar(0))
		v /= std::sqrt(norm2);
}

/**
 * @brief cosinus of the angle formed by 2 vectors (Eigen type)
 */
template <typename VECa, typename VECb>
inline ScalarOf<VECa> cos_angle(const Eigen::MatrixBase<VECa>& a, const Eigen::MatrixBase<VECb>& b)
{
	static_assert(is_same_vector<VECa,VECb>::value, "parameters must have same type");

	using Scalar = typename vector_traits<VECa>::Scalar;

	Scalar na2 = a.squaredNorm();
	Scalar nb2 = b.squaredNorm();

	Scalar res = a.dot(b) / std::sqrt(na2 * nb2);
	return std::max(Scalar(-1), std::min(res, Scalar(1)));
}

/**
 * @brief angle formed by 2 vectors
 */
template <typename VECa, typename VECb>
inline ScalarOf<VECa> angle(const Eigen::MatrixBase<VECa>& a, const Eigen::MatrixBase<VECb>& b)
{
	static_assert(is_same_vector<VECa,VECb>::value, "parameters must have same type");
	return std::acos(cos_angle(a,b));
}


/// non-eigen versions

template <typename VEC>
inline auto normalize_safe(VEC& v) -> typename std::enable_if <is_vec_non_eigen<VEC>::value,void>::type
{
	auto w = eigenize(v);
	normalize_safe(w);
}

template <typename VEC>
inline auto cos_angle(const VEC& a, const VEC& b)
-> typename std::enable_if<is_vec_non_eigen<VEC>::value, ScalarOf<VEC>>::type
{
	return cos_angle(eigenize(a),eigenize(b));
}

template <typename VEC>
inline auto angle(const VEC& a, const VEC& b)
-> typename std::enable_if<is_vec_non_eigen<VEC>::value, ScalarOf<VEC> >::type
{
	return angle(eigenize(a),eigenize(b));
}


} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_FUNCTIONS_BASICS_H_

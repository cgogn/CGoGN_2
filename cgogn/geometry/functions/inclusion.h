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

#ifndef CGOGN_GEOMETRY_INCLUSION_H_
#define CGOGN_GEOMETRY_INCLUSION_H_

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/functions/normal.h>

namespace cgogn
{

namespace geometry
{

/**
 * \todo geometric predicate : move it to a specific location with other geometric predicates
 */
template <typename VEC3, typename VEC3b>
auto in_sphere(const VEC3& point, const VEC3b& center, const typename vector_traits<VEC3>::Scalar radius)
-> typename std::enable_if <is_same2vector<VEC3, VEC3b>::value, bool>::type
{
	return (point - center).norm() < radius;
}

template <typename VEC3, typename VEC3b, typename VEC3c, typename VEC3d, typename VEC3e>
auto in_triangle(const VEC3& P, const VEC3b& normal, const VEC3c& Ta,  const VEC3d& Tb, const VEC3e& Tc)
-> typename std::enable_if <is_same5vector<VEC3, VEC3b, VEC3c, VEC3d, VEC3e>::value, bool>::type
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	static const auto triple_positive = [] (const VEC3& U, const VEC3& V, const VEC3& W) -> bool
	{
		return U.dot(V.cross(W)) >= Scalar(0);
	};

	if (triple_positive(P-Ta, Tb-Ta, normal) ||
		triple_positive(P-Tb, Tc-Tb, normal) ||
		triple_positive(P-Tc, Ta-Tc, normal) )
		return false;

	return true;
}

template <typename VEC3, typename VEC3b, typename VEC3c, typename VEC3d>
auto in_triangle(const VEC3& P, const VEC3b& Ta,  const VEC3c& Tb, const VEC3d& Tc)
-> typename std::enable_if <is_same4vector<VEC3, VEC3b, VEC3c, VEC3d>::value, bool>::type
{
	return in_triangle(P, normal(Ta, Tb, Tc), Ta, Tb,Tc );
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_INCLUSION_H_

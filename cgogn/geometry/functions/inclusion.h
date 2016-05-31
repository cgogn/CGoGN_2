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
template <typename VEC3>
bool in_sphere(const VEC3& point, const VEC3& center, const typename vector_traits<VEC3>::Scalar radius)
{
	return (point - center).norm() < radius;
}

template <typename VEC3>
inline bool in_triangle(const VEC3& P, const VEC3& normal, const VEC3& Ta,  const VEC3& Tb, const VEC3& Tc)
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

template <typename VEC3>
inline bool in_triangle(const VEC3& P, const VEC3& Ta,  const VEC3& Tb, const VEC3& Tc)
{
	return in_triangle(P, normal(Ta, Tb, Tc), Ta, Tb,Tc );
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_INCLUSION_H_

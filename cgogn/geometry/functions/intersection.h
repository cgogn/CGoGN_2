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

#ifndef CGOGN_GEOMETRY_FUNCTIONS_INTERSECTION_H_
#define CGOGN_GEOMETRY_FUNCTIONS_INTERSECTION_H_

#include <cmath>
#include <cgogn/core/utils/numerics.h>
#include <cgogn/geometry/types/geometry_traits.h>

namespace cgogn
{

namespace geometry
{

/**
 * \todo geometric predicate : move it to a specific location with other geometric predicates
 */
template <typename VEC3>
bool in_sphere(const VEC3& point, const VEC3& center, const typename VEC3::Scalar& radius)
{
	return (point - center).norm() < radius;
}

template <typename VEC3>
bool intersection_ray_triangle(const VEC3& P, const VEC3& Dir, const VEC3& Ta, const VEC3& Tb, const VEC3& Tc, VEC3* inter = nullptr)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;

	VEC3 u = Ta - P ;
	VEC3 v = Tb - P ;
	VEC3 w = Tc - P ;

	Scalar x = Dir.dot(u.cross(v));//tripleProduct(Dir, u, v) ;
	Scalar y = Dir.dot(v.cross(w));//tripleProduct(Dir, v, w) ;
	Scalar z = Dir.dot(w.cross(u));//tripleProduct(Dir, w, u) ;

	uint32 np = 0 ;
	uint32 nn = 0 ;
	uint32 nz = 0 ;

	if (x > Scalar(0))
		++np ;
	else if (x < Scalar(0))
		++nn ;
	else
		++nz;

	if (y > Scalar(0))
		++np ;
	else if (y < Scalar(0))
		++nn ;
	else
		++nz;

	if (z > Scalar(0))
		++np ;
	else if (z < Scalar(0))
		++nn ;
	else
		++nz;

	// line intersect the triangle
	if (((np != 0) && (nn != 0)) || (nz == 3))
		return false ;

	Scalar sum = x + y + z ;
	Scalar alpha = y / sum ;
	Scalar beta = z / sum ;
	Scalar gamma =Scalar(1) - alpha - beta ;
	VEC3 I = Ta * alpha + Tb * beta + Tc * gamma ;

	//  it's a ray not a line !
	if (Dir.dot(I-P) < 0.0)
		return false;

	if (inter)
		*inter = I;

	return true ;
}

/**
 * \param[in] center the position of the center of the sphere.
 * \param[in] radius the radius of the sphere
 * \param[in] p1 first point of the edge
 * \param[in] p2 second point of the edge
 * \param[out] alpha size of the edge inside the sphere
 */
template <typename VEC3>
bool intersection_sphere_edge(
	const VEC3& center,
	const typename vector_traits<VEC3>::Scalar& radius,
	const VEC3& p1,
	const VEC3& p2,
	typename vector_traits<VEC3>::Scalar& alpha
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;

	if(in_sphere(p1, center, radius) && !in_sphere(p2, center, radius))
	{
		VEC3 p = p1 - center;
		VEC3 qminusp = p2 - center - p;
		Scalar s = p.dot(qminusp);
		Scalar n2 = qminusp.squaredNorm();
		alpha = (- s + std::sqrt(s*s + n2 * (radius*radius - p.squaredNorm()))) / n2;
		return true ;
	}
	return false ;
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_FUNCTIONS_INTERSECTION_H_

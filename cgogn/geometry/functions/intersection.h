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
#include <cgogn/geometry/functions/inclusion.h>

namespace cgogn
{

namespace geometry
{

enum Intersection
{
	NO_INTERSECTION = 0,
	VERTEX_INTERSECTION = 1,
	EDGE_INTERSECTION = 2,
	FACE_INTERSECTION = 3
} ;


template <typename VEC3, typename VEC3b, typename VEC3c, typename VEC3d, typename VEC3e>
auto intersection_ray_triangle(const VEC3& P, const VEC3b& Dir, const VEC3c& Ta, const VEC3d& Tb, const VEC3e& Tc, typename vector_traits<VEC3>::Type* inter = nullptr)
-> typename std::enable_if <is_same5vector<VEC3, VEC3b, VEC3c, VEC3d, VEC3e>::value, bool>::type
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using NVEC3 = typename vector_traits<VEC3>::Type;

	NVEC3 u = Ta - P;
	NVEC3 v = Tb - P;
	NVEC3 w = Tc - P;

	Scalar x = Dir.dot(u.cross(v));
	Scalar y = Dir.dot(v.cross(w));
	Scalar z = Dir.dot(w.cross(u));

	uint32 np = 0;
	uint32 nn = 0;
	uint32 nz = 0;

	if (x > Scalar(0)) ++np;
	else if (x < Scalar(0)) ++nn;
	else ++nz;

	if (y > Scalar(0)) ++np;
	else if (y < Scalar(0)) ++nn;
	else ++nz;

	if (z > Scalar(0)) ++np;
	else if (z < Scalar(0)) ++nn;
	else ++nz;

	// line intersect the triangle
	if (((np != 0) && (nn != 0)) || (nz == 3))
		return false;

	Scalar sum = x + y + z;
	Scalar alpha = y / sum;
	Scalar beta = z / sum;
	Scalar gamma =Scalar(1) - alpha - beta;
	NVEC3 I = Ta * alpha + Tb * beta + Tc * gamma;

	//  it's a ray not a line !
	if (Dir.dot(I-P) < 0.0)
		return false;

	if (inter)
		*inter = I;

	return true;
}

/**
 * \param[in] center the position of the center of the sphere.
 * \param[in] radius the radius of the sphere
 * \param[in] p1 first point of the segment
 * \param[in] p2 second point of the segment
 * \param[out] alpha ratio of the segment inside the sphere
 */
template <typename VEC3, typename VEC3b, typename VEC3c>
auto intersection_sphere_segment(
	const VEC3& center,
	const typename vector_traits<VEC3>::Scalar radius,
	const VEC3b& p1,
	const VEC3c& p2,
	typename vector_traits<VEC3>::Scalar& alpha)
	-> typename std::enable_if <is_same3vector <VEC3, VEC3b, VEC3c>::value, bool>::type

{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using NVEC3 = typename vector_traits<VEC3>::Type;

	if (in_sphere(p1, center, radius) && !in_sphere(p2, center, radius))
	{
		NVEC3 p = p1 - center;
		NVEC3 qminusp = p2 - center - p;
		Scalar s = p.dot(qminusp);
		Scalar n2 = qminusp.squaredNorm();
		alpha = (- s + std::sqrt(s*s + n2 * (radius*radius - p.squaredNorm()))) / n2;
		return true;
	}

	return false;
}

template <typename VEC3, typename VEC3b, typename VEC3c, typename VEC3d, typename VEC3e>
auto intersection_segment_segment(
		const VEC3& PA,
		const VEC3b& PB,
		const VEC3c& QA,
		const VEC3d& QB,
		VEC3e& Inter)
		-> typename std::enable_if <is_same5vector<VEC3, VEC3b, VEC3c, VEC3d, VEC3e>::value, Intersection>::type
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	const Scalar PRECISION = std::numeric_limits<Scalar>::epsilon();
	using NVEC3 = typename vector_traits<VEC3>::Type;


	NVEC3 vp1p2 = PB - PA;
	NVEC3 vq1q2 = QB - QA;
	NVEC3 vp1q1 = QA - PA;
	Scalar delta = vp1p2[0] * vq1q2[1] - vp1p2[1] * vq1q2[0] ;
	Scalar coeff = vp1q1[0] * vq1q2[1] - vp1q1[1] * vq1q2[0] ;

	if (delta == 0) //parallel
	{
		//test if collinear
		if (coeff == 0)
		{
			//collinear
			//TODO : check if there is a common point between the two edges
			Inter = QA;
			return EDGE_INTERSECTION;
		}
		else
			return NO_INTERSECTION;
	}
	else
		Inter = NVEC3((PA[0] * delta + vp1p2[0] * coeff) / delta, (PA[1] * delta + vp1p2[1] * coeff) / delta, (PA[2] * delta + vp1p2[2] * coeff) / delta) ;

	//test if inter point is outside the edges
	if(
		(Inter[0] < PA[0] && Inter[0] < PB[0]) || (Inter[0] > PA[0] && Inter[0] > PB[0]) ||
		(Inter[0] < QA[0] && Inter[0] < QB[0]) || (Inter[0] > QA[0] && Inter[0] > QB[0]) ||
		(Inter[1] < PA[1] && Inter[1] < PB[1]) || (Inter[1] > PA[1] && Inter[1] > PB[1]) ||
		(Inter[1] < QA[1] && Inter[1] < QB[1]) || (Inter[1] > QA[1] && Inter[1] > QB[1])
	)
		return NO_INTERSECTION;

	if(numerics::almost_equal_absolute(PA, Inter) || numerics::almost_equal_absolute(PB, Inter) || numerics::almost_equal_absolute(QA, Inter) || numerics::almost_equal_absolute(QB, Inter))
		return VERTEX_INTERSECTION;

	return EDGE_INTERSECTION;
}

template <typename VEC3, typename VEC3b, typename VEC3c, typename VEC3d>
auto intersection_line_plane(const VEC3& point_line, const VEC3b& dir_line, const VEC3c& point_plane, const VEC3d& normal_plane, typename vector_traits<VEC3>::Type* inter = nullptr)
-> typename std::enable_if <is_same4vector<VEC3, VEC3b, VEC3c, VEC3d>::value, bool>::type
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	const Scalar PRECISION = std::numeric_limits<Scalar>::epsilon();

	Scalar b = normal_plane.dot(dir_line);

	if (std::abs(b) < PRECISION)
		return false;

	Scalar a = normal_plane.dot(point_plane - point_line);
	if (inter)
		*inter = point_line + (a / b) * dir_line;

	return true;
}



} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_FUNCTIONS_INTERSECTION_H_

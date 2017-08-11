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


template <typename VEC3>
bool intersection_ray_triangle(const VEC3& P, const VEC3& Dir, const VEC3& Ta, const VEC3& Tb, const VEC3& Tc, VEC3* inter = nullptr)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;

	VEC3 u = Ta - P;
	VEC3 v = Tb - P;
	VEC3 w = Tc - P;

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
	VEC3 I = Ta * alpha + Tb * beta + Tc * gamma;

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
template <typename VEC3>
bool intersection_sphere_segment(
	const VEC3& center,
	const typename vector_traits<VEC3>::Scalar radius,
	const VEC3& p1,
	const VEC3& p2,
	typename vector_traits<VEC3>::Scalar& alpha
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;

	if (in_sphere(p1, center, radius) && !in_sphere(p2, center, radius))
	{
		VEC3 p = p1 - center;
		VEC3 qminusp = p2 - center - p;
		Scalar s = p.dot(qminusp);
		Scalar n2 = qminusp.squaredNorm();
		alpha = (- s + std::sqrt(s*s + n2 * (radius*radius - p.squaredNorm()))) / n2;
		return true;
	}

	return false;
}

template <typename VEC3>
Intersection intersection_segment_segment(
		const VEC3& PA,
		const VEC3& PB,
		const VEC3& QA,
		const VEC3& QB,
		VEC3& Inter)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	const Scalar PRECISION = std::numeric_limits<Scalar>::epsilon();

	VEC3 vp1p2 = PB - PA;
	VEC3 vq1q2 = QB - QA;
	VEC3 vp1q1 = QA - PA;
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
		Inter = VEC3((PA[0] * delta + vp1p2[0] * coeff) / delta, (PA[1] * delta + vp1p2[1] * coeff) / delta, (PA[2] * delta + vp1p2[2] * coeff) / delta) ;

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

template <typename VEC3>
bool intersection_line_plane(const VEC3& point_line, const VEC3& dir_line, const VEC3& point_plane, const VEC3& normal_plane, VEC3* inter = nullptr)
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

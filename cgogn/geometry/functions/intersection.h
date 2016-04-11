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

namespace cgogn
{

namespace geometry
{


template <typename VEC3_T>
bool intersection_ray_triangle(const VEC3_T& P, const VEC3_T& Dir, const VEC3_T& Ta, const VEC3_T& Tb, const VEC3_T& Tc, VEC3_T* inter=nullptr)
{
	using Scalar = typename VEC3_T::Scalar;

	VEC3_T u = Ta - P ;
	VEC3_T v = Tb - P ;
	VEC3_T w = Tc - P ;

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
	VEC3_T I = Ta * alpha + Tb * beta + Tc * gamma ;

	//  it's a ray not a line !
	if (Dir.dot(I-P)<0.0)
		return false;

	if (inter)
		*inter = I;


	return true ;

}



} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_FUNCTIONS_INTERSECTION_H_

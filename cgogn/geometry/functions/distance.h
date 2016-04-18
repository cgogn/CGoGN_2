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

#ifndef CGOGN_GEOMETRY_FUNCTIONS_DISTANCE_H_
#define CGOGN_GEOMETRY_FUNCTIONS_DISTANCE_H_

#include <cgogn/core/utils/assert.h>

namespace cgogn
{

namespace geometry
{


/**
 * @brief squared distance line point (optimized version for testing many points with the same line
 * @param A one point of line
 * @param AB normalized vector or line
 * @param P point o compute distance to line
 * @return distance
 */
template <typename VEC3_T>
inline typename VEC3_T::Scalar squared_distance_normalized_line_point(const VEC3_T& A, const VEC3_T& AB_norm, const VEC3_T& P)
{
	// here use const & ? Strange Schmitt optimization proposition ;)
	const VEC3_T& V = A - P ;
	const VEC3_T& W = V.cross(AB_norm) ;
	return W.squaredNorm() ;
}

/**
 * @brief squared distance line point
 * @param A one point of line
 * @param B second point of line
 * @param P point o compute distance to line
 * @return distance
 */
template <typename VEC3_T>
inline typename VEC3_T::Scalar squared_distance_line_point(const VEC3_T& A, const VEC3_T& B, const VEC3_T& P)
{
	VEC3_T AB = B - A ;
	cgogn_message_assert(AB.squaredNorm()>0.0,"line must be defined by 2 different points");
	AB.normalize();
	return squared_distance_normalized_line_point(A, AB, P) ;
}






} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_FUNCTIONS_DISTANCE_H_

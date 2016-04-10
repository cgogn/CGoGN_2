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

#ifndef CGOGN_GEOMETRY_ALGOS_ANGLE_H_
#define CGOGN_GEOMETRY_ALGOS_ANGLE_H_

#include <cmath>

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/algos/normal.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3_T, typename MAP>
inline typename VEC3_T::Scalar angle_between_face_normals(
		const MAP& map,
		const typename MAP::Edge e,
		const typename MAP::template VertexAttribute<VEC3_T>& position)
{
	using Scalar = typename VEC3_T::Scalar;
    using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;

	if(map.is_boundary(e.dart))
        return Scalar(0) ;

    Vertex v1(e.dart);
	Vertex v2(map.phi2(e.dart));
	const VEC3_T n1 = face_normal<VEC3_T, MAP>(map, Face(v1.dart), position);
	const VEC3_T n2 = face_normal<VEC3_T, MAP>(map, Face(v2.dart), position);

	VEC3_T edge = position[v2] - position[v1] ;
    edge.normalize() ;
	Scalar s = edge.dot(n1.cross(n2)) ;
	Scalar c = n1.dot(n2);
    Scalar a(0) ;

    // the following trick is useful for avoiding NaNs (due to floating point errors)
    if (c > 0.5) a = std::asin(s) ;
    else
    {
        if(c < -1) c = -1 ;
        if (s >= 0) a = std::acos(c) ;
        else a = -std::acos(c) ;
    }
    //	if (isnan(a))
    if(a != a)
        std::cerr<< "Warning : computeAngleBetweenNormalsOnEdge returns NaN on edge " << v1 << "-" << v2 << std::endl ;

    return a ;
}

template <typename VEC3_T, typename MAP>
inline void angle_between_face_normals(
		const MAP& map,
		const typename MAP::template VertexAttribute<VEC3_T>& position,
		typename MAP::template Attribute<typename VEC3_T::Scalar, Orbit::PHI2>& angles)
{
	using Edge = typename MAP::Edge;

	map.foreach_cell([&] (Edge e)
	{
		angles[e] = angle_between_face_normals<VEC3_T, MAP>(map, e, position);
	});
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_ANGLE_H_

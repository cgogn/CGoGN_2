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

#ifndef CGOGN_GEOMETRY_ALGOS_CONVEXITY_H
#define CGOGN_GEOMETRY_ALGOS_CONVEXITY_H

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/algos/length.h>

#include <cgogn/core/cmap/attribute.h>

namespace cgogn
{

namespace geometry
{

/**
 * @brief is_planar verifies if a face is planar: defined by the fact that all coordinates other than the first two (x and y) are constant (up to epsilon)
 * @param map the map
 * @param f the face
 * @param position container of vertex positions
 * @param epsilon maximum deviation of each component
 * @return true iff planar
 */
template <typename VEC, typename MAP>
inline bool is_planar(
	const MAP& map,
	const typename MAP::Face f,
	const typename MAP::template VertexAttribute<VEC>& position,
	typename vector_traits<VEC>::Scalar epsilon = Eigen::NumTraits<typename vector_traits<VEC>::Scalar>::dummy_precision()
)
{
	using Scalar = typename vector_traits<VEC>::Scalar;
	using Vertex = typename MAP::Vertex;

	if (vector_traits<VEC>::SIZE < 3)
		return true ;

	using VecN_2 = Eigen::Matrix<Scalar,vector_traits<VEC>::SIZE - 2,1> ;
	// Init with first vertex
	VecN_2 depth0 ;
	for (uint i = 0 ; i < vector_traits<VEC>::SIZE - 2 ; ++i)
		depth0[i] = position[Vertex(f.dart)][i+2] ;

	// verify that other vertices do not deviate in coordinates 3 (z) and higher.
	bool planar = true ;
	map.foreach_incident_vertex(f, [&] (Vertex v) -> bool
	{
		VecN_2 depth ;
		for (uint i = 0 ; i < vector_traits<VEC>::SIZE - 2 ; ++i)
		{
			depth[i] = position[v][i+2] ;
			if (!depth.isApprox(depth0,epsilon))
			{
				planar = false ;
			}
		}
		return planar ;
	});

	return planar ;
}

/**
 * @brief is_convex verifies if a given face is convex (in the plane, not in 3D).
 * @param map the map
 * @param f the face
 * @param position the geometric embedding of the vertices
 * @note if the face is not planar (see #is_planar), false is returned.
 * @return true iff the face is convex.
 */
template <typename VEC, typename MAP>
inline bool is_convex(
	const MAP& map,
	const typename MAP::Face f,
	const typename MAP::template VertexAttribute<VEC>& position
)
{
	if (!is_planar<VEC,MAP>(map,f,position))
	{
		cgogn_log_error("Trying to assess convexity of a face while it's not planar. This is ill-defined") ;
		return false ;
	}

	using Scalar = typename vector_traits<VEC>::Scalar;
	using Vertex = typename MAP::Vertex;
	//using Dart = typename MAP::Dart;

	bool convex = true ;
	if (map.codegree(f) >= 3) // true otherwise
	{
		// Take all two consecutive darts and compute cross products.
		// The face is convex iff the z components of all products have the same sign.
		bool sign ;
		bool first = true ;
		map.foreach_dart_of_orbit(f, [&] (Dart d)
		{
			const VEC d1 = vector_from<VEC>(map,d,position) ;
			const VEC d2 = vector_from<VEC>(map,map.phi1(d),position) ;
			const Scalar zcross = d1[0]*d2[1] - d1[1]*d2[0] ;
			if (first)
			{
				sign = (zcross > Scalar(0)) ;
				first = false ;
			}
			else
			{
				if (sign != (zcross > Scalar(0)))
					convex = false ;
			}
		});
	}
	return convex ;
}

} // Geometry

} // CGoGN

#endif // CGOGN_GEOMETRY_ALGOS_CONVEXITY_H

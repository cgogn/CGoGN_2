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

#ifndef CGOGN_GEOMETRY_ALGOS_CENTROID_H_
#define CGOGN_GEOMETRY_ALGOS_CENTROID_H_

#include <cgogn/core/basic/cell.h>
#include <cgogn/geometry/types/geometry_traits.h>

namespace cgogn
{

namespace geometry
{

template <typename T, Orbit ORBIT, typename MAP>
inline T centroid(const MAP& map, Cell<ORBIT> c, const typename MAP::template VertexAttribute<T>& attribute)
{
	T result;
	set_zero(result);
	uint32 count = 0;
	map.foreach_incident_vertex(c, [&] (typename MAP::Vertex v)
	{
		result += attribute[v];
		++count;
	});
	result /= typename T::Scalar(count);
	return result;
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_CENTROID_H_

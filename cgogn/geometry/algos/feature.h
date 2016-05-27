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

#ifndef CGOGN_GEOMETRY_ALGOS_FEATURE_H_
#define CGOGN_GEOMETRY_ALGOS_FEATURE_H_

#include <cgogn/core/utils/numerics.h>
#include <cgogn/geometry/functions/basics.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3, typename MAP>
void mark_feature_edges(
	const MAP& map,
	const typename MAP::template FaceAttribute<VEC3>& normal,
	typename MAP::template CellMarker<MAP::Edge::ORBIT>& feature_edge
)
{
	feature_edge.unmark_all();

	map.parallel_foreach_cell([&] (typename MAP::Edge e, uint32)
	{
		if (angle(normal[e.dart], normal[map.phi2(e.dart)] > M_PI / 6.))
			feature_edge.mark(e);
	});
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_FEATURE_H_

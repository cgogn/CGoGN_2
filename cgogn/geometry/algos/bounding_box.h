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

#ifndef CGOGN_GEOMETRY_ALGO_BOUNDING_BOX_H_
#define CGOGN_GEOMETRY_ALGO_BOUNDING_BOX_H_

#include <cgogn/geometry/types/aabb.h>
#include <cgogn/core/basic/cell.h>
#include <cgogn/core/utils/masks.h>

namespace cgogn
{

namespace geometry
{

template <typename ATTR>
void compute_AABB(const ATTR& attr, AABB<array_data_type<ATTR>>& bb)
{
	bb.reset();
	for(const auto& p : attr)
		bb.add_point(p);
}

template <typename ATTR, typename MAP, typename MASK>
void compute_AABB(const ATTR& attr, const MAP& map, const MASK& mask, AABB<array_data_type<ATTR>>& bb)
{
	bb.reset();
	map.foreach_cell([&] (Cell<ATTR::orb_> c)
	{
		bb.add_point(attr[c]);
	},
	mask);
}

template <typename ATTR, typename MAP>
void compute_AABB(const ATTR& attr, const MAP& map, AABB<array_data_type<ATTR>>& bb)
{
	compute_AABB(attr, map, AllCellsFilter(), bb);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGO_BOUNDING_BOX_H_

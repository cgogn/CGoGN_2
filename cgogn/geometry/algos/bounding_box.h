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

#include <cgogn/geometry/types/bounding_box.h>

namespace cgogn
{

namespace geometry
{

template<typename ATTR>
void compute_bounding_box(const ATTR& attr, BoundingBox<inside_type(ATTR)>& bb)
{
	bb.reset();
	for(const auto& p : attr)
		bb.add_point(p);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGO_BOUNDING_BOX_H_

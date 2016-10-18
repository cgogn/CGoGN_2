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

#include <cgogn/multiresolution/cph/cph2.h>

namespace cgogn
{

uint32 CPH2::get_tri_refinement_edge_id(Dart d, Dart e) const
{
	uint32 d_id = get_edge_id(d);
	uint32 e_id = get_edge_id(e);

	uint32 id = d_id + e_id;

	if (id == 0u)
		return 1u;
	else if (id == 1u)
		return 2u;
	else if (id == 2u)
	{
		if (d_id == e_id)
			return 0u;
		else
			return 1u;
	}
	// else if (id == 3)
	return 0u;
}

uint32 CPH2::get_quad_refinement_edge_id(Dart d) const
{
	uint32 e_id = get_edge_id(d);

	if (e_id == 0u)
		return 1u;
	// else if (e_id == 1)
	return 0u;
}

} // namespace cgogn

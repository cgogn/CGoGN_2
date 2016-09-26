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

#ifndef CGOGN_MODELING_TILING_TILING_H_
#define CGOGN_MODELING_TILING_TILING_H_

#include <vector>
#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/basic/dart.h>

namespace cgogn
{

namespace modeling
{

template <typename MAP>
class Tiling
{

protected:
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;

public:
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Tiling);
	/**
	 * @brief Map in which we are working
	 */
	MAP& map_;

	/**
	 * @brief Dimensions of the tiling
	 */
	uint32 nx_, ny_, nz_;

	/**
	 * @brief Reference dart;
	 */
	Dart dart_;

	/**
	 * @brief Table of vertices
	 * Order depends on the tiling kind
	 */
	std::vector<Vertex> vertex_table_;

	/**
	 * @brief Table of faces
	 */
	std::vector<Face> face_table_;

	Tiling(MAP& map, uint32 x, uint32 y, uint32 z):
		map_(map),
		nx_(x),
		ny_(y),
		nz_(z)
	{}

	Tiling(MAP& map):
		Tiling(map, UINT32_MAX, UINT32_MAX, 1u)
	{}

	std::vector<Vertex>& vertices()
	{
		return vertex_table_;
	}

	std::vector<Face>& faces()
	{
		return face_table_;
	}
};

} //namespace modeling

} //namespace cgogn

#endif // CGOGN_MODELING_TILING_TILING_H_

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

#define CGOGN_MODELING_ALGOS_TETRAHEDRALIZATION_CPP_

#include <cgogn/modeling/algos/tetrahedralization.h>

namespace cgogn
{

namespace modeling
{

template CGOGN_MODELING_API void flip_edge(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Edge2);
template CGOGN_MODELING_API void flip_back_edge(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Edge2);
template CGOGN_MODELING_API Dart split_vertex<DefaultMapTraits>(CMap3<DefaultMapTraits>& , std::vector<Dart>&);
template CGOGN_MODELING_API Dart swap_22<DefaultMapTraits>(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Volume);
template CGOGN_MODELING_API Dart swap_32<DefaultMapTraits>(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Edge);
template CGOGN_MODELING_API Dart swap_23<DefaultMapTraits>(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Face);
template CGOGN_MODELING_API Dart swap_44<DefaultMapTraits>(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Volume);
template CGOGN_MODELING_API CMap3<DefaultMapTraits>::Vertex flip_14<DefaultMapTraits>(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Volume);
template CGOGN_MODELING_API CMap3<DefaultMapTraits>::Vertex flip_13<DefaultMapTraits>(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Face);
template CGOGN_MODELING_API Dart edge_bisection(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Edge);
template CGOGN_MODELING_API Dart swap_gen_32(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Edge);

} // namespace modeling
} // namespace cgogn

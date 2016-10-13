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

#define CGOGN_MODELING_ALGOS_REFINEMENTS_CPP_

#include <cgogn/modeling/algos/refinements.h>

namespace cgogn
{

namespace modeling
{

template CGOGN_MODELING_API CMap2<DefaultMapTraits>::Vertex triangule<CMap2<DefaultMapTraits>>(CMap2<DefaultMapTraits>&, CMap2<DefaultMapTraits>::Face);
template CGOGN_MODELING_API CMap3<DefaultMapTraits>::Vertex triangule<CMap3<DefaultMapTraits>>(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Face);

} // namespace modeling
} // namespace cgogn

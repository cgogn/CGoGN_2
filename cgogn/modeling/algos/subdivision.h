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

#ifndef CGOGN_MODELING_ALGOS_SUBDIVISION_H_
#define CGOGN_MODELING_ALGOS_SUBDIVISION_H_

#include <cgogn/modeling/dll.h>
#include <cgogn/core/cmap/cmap3.h>

namespace cgogn
{

namespace modeling
{

/**
* Triangule a face with central vertex
* @param d dart of face
* @return a dart of the central vertex
*/
template <typename MAP>
typename MAP::Vertex triangule_face(MAP& map, Dart f)
{
	Dart d1 = map.phi1(f);
	if (d1 == f)
		cgogn_log_warning("modeling::trianguleFace") << "Triangulation of a face with only one edge.";
	if (map.phi1(d1) == f)
		cgogn_log_warning("modeling::trianguleFace") << "Triangulation of a face with only two edges.";

	map.cut_face(f, d1);
	map.cut_edge(typename MAP::Edge(map.phi_1(f)));
	const Dart x = map.phi2(map.phi_1(f));
	Dart dd = map.template phi<111>(x);
	while(dd != x)
	{
		const Dart next = map.phi1(dd);
		map.cut_face(dd, map.phi1(x));
		dd = next ;
	}

	return typename MAP::Vertex(map.phi2(x));
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_SUBDIVISION_CPP_))
extern template CGOGN_MODELING_API CMap2<DefaultMapTraits>::Vertex triangule_face<CMap2<DefaultMapTraits>>(CMap2<DefaultMapTraits>&, Dart);
extern template CGOGN_MODELING_API CMap3<DefaultMapTraits>::Vertex triangule_face<CMap3<DefaultMapTraits>>(CMap3<DefaultMapTraits>&, Dart);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_SUBDIVISION_CPP_))

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_SUBDIVISION_H_

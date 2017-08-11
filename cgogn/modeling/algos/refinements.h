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

#ifndef CGOGN_MODELING_ALGOS_REFINEMENTS_H_
#define CGOGN_MODELING_ALGOS_REFINEMENTS_H_

#include <cgogn/modeling/dll.h>
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/utils/masks.h>
#include <cgogn/geometry/algos/centroid.h>

namespace cgogn
{

namespace modeling
{

template <typename MAP>
typename MAP::Vertex triangule(MAP& map, typename MAP::Face f)
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;

	const Dart d = f.dart;
	const Dart d1 = map.phi1(d);
	map.cut_face(d, d1);
	map.cut_edge(Edge(map.phi_1(d)));

	const Dart x = map.phi2(map.phi_1(d));
	Dart dd = map.template phi<111>(x);
	while(dd != x)
	{
		Dart next = map.phi1(dd) ;
		map.cut_face(dd, map.phi1(x)) ;
		dd = next ;
	}

	return Vertex(map.phi2(x));
}

template<typename MAP, typename VEC3>
void triangule(MAP& map, typename MAP::template VertexAttribute<VEC3>& position)
{
	using Face = typename MAP::Face;
	typename MAP::CellCache cache(map);
	cache.template build<Face>();
	map.parallel_foreach_cell([&map, &position](Face f)
	{
		const VEC3& center = geometry::centroid<VEC3>(map, f, position);
		const auto central_vertex = triangule(map, f);
		position[central_vertex] = center;
	}, cache);
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_REFINEMENTS_CPP_))
extern template CGOGN_MODELING_API CMap2::Vertex triangule<CMap2>(CMap2&, CMap2::Face);
extern template CGOGN_MODELING_API CMap3::Vertex triangule<CMap3>(CMap3&, CMap3::Face);
extern template CGOGN_MODELING_API void triangule<CMap2, Eigen::Vector3f>(CMap2&, CMap2::VertexAttribute<Eigen::Vector3f>&);
extern template CGOGN_MODELING_API void triangule<CMap2, Eigen::Vector3d>(CMap2&, CMap2::VertexAttribute<Eigen::Vector3d>&);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_REFINEMENTS_CPP_))

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_REFINEMENTS_H_

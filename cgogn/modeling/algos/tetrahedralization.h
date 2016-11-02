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

#ifndef CGOGN_MODELING_ALGOS_TETRAHEDRALIZATION_H_
#define CGOGN_MODELING_ALGOS_TETRAHEDRALIZATION_H_

#include <cgogn/modeling/algos/refinements.h>
#include <cgogn/geometry/algos/ear_triangulation.h>

namespace cgogn
{

namespace modeling
{

inline Dart split_vertex(CMap3& map, std::vector<Dart>& vd);

bool is_tetrahedron(CMap3& map, CMap3::Volume w);

//bool is_tetrahedralization(CMap3& map);

Dart swap_22(CMap3& map, CMap3::Volume w);

Dart swap_44(CMap3& map, CMap3::Volume w);

Dart swap_32(CMap3& map, CMap3::Edge e);

Dart swap_23(CMap3& map, CMap3::Face f);

CMap3::Vertex flip_14(CMap3& map, CMap3::Volume w);

CMap3::Vertex flip_13(CMap3& map, CMap3::Face f);

Dart edge_bisection(CMap3& map, CMap3::Edge e);

Dart swap_gen_32(CMap3& map, CMap3::Edge e);

//template <typename VEC3>
//std::vector<Dart> swap_gen_32_optimized(CMap3& map, CMap3::Edge e)
//{
//	using Edge = typename CMap3::Edge;
//	using Face = typename CMap3::Face;
//	using Volume = typename CMap3::Volume;

//	std::vector<Dart>& edges = *cgogn::dart_buffers()->buffer();
//	const Dart stop = map.phi1(map.phi2(map.phi_1(e.dart)));
//	if (map.merge_incident_volumes(e).is_nil())
//	{
//		const Dart d_begin = map.boundary_face_of_edge(e).dart;
//		map.foreach_incident_volume(e, [&](Volume w) { edges.push_back(w.dart); });

//		for (Dart it : edges)
//			map.merge_incident_volumes(Face(it));

//		map.flip_back_edge(Edge(d_begin));
//	}

//	Dart dit = stop;
//	do
//	{
//		edges.push_back(dit);
//		dit = map.template phi<121>(dit);
//	} while(dit != stop);

//	map.cut_volume(edges);

//	geometry::EarTriangulation<VEC3, CMap3> triangulation(map);
////	triangulation.trianguleFace(map.phi1(map.phi2(stop)));

//	cgogn::dart_buffers()->release_buffer(&edges);
//	return triangulation.getResultingTets();
//}

//void swapGen2To3(CMap3& map, typename CMap3::Volume d);

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_TETRAHEDRALIZATION_CPP_))
//extern template CGOGN_MODELING_API std::vector<Dart> swap_gen_32_optimized<Eigen::Vector3f>(CMap3&, CMap3::Edge);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_TETRAHEDRALIZATION_CPP_))

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_TETRAHEDRALIZATION_H_

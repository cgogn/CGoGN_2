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

template <typename MAP_TRAITS>
inline Dart split_vertex(CMap3<MAP_TRAITS>& map, std::vector<Dart>& vd)
{
	//split the vertex
	const Dart dres = map.split_vertex(vd);

	//split the faces incident to the new vertex
	const Dart dbegin = map.phi1(map.phi2(vd.front()));
	Dart dit = dbegin;
	do {
		map.cut_face(map.phi1(dit),map.phi_1(dit));
		dit = map.phi3(map.phi2(dit));
	} while(dbegin != dit);

	//split the volumes incident to the new vertex
	for(unsigned int i = 0; i < vd.size(); ++i)
	{
		Dart dit = vd[i];

		std::vector<Dart> v;
		v.push_back(map.phi1(map.phi1(map.phi2(dit))));
		v.push_back(map.phi1(dit));
		v.push_back(map.phi1(map.phi2(map.phi_1(dit))));
		map.cut_volume(v);
	}

	return dres;
}

template <typename MAP_TRAITS>
bool is_tetrahedron(CMap3<MAP_TRAITS>& map, typename CMap3<MAP_TRAITS>::Volume w)
{
	return true;
}

//template <typename MAP_TRAITS>
//bool is_tetrahedralization(CMap3<MAP_TRAITS>& map)
//{

//}

template <typename MAP_TRAITS>
Dart swap_22(CMap3<MAP_TRAITS>& map, typename CMap3<MAP_TRAITS>::Volume w)
{
	using Face = typename CMap3<MAP_TRAITS>::Face;

	std::vector<Dart> edges;
	const Dart d2_1 = map.phi_1(map.phi2(w.dart));

	map.merge_incident_volumes(Face(w.dart));
	map.merge_incident_faces(map.phi1(d2_1));
	map.cut_face(d2_1, map.phi1(map.phi1(d2_1)));

	const Dart stop = map.phi_1(d2_1);
	Dart dit = stop;
	do {
		edges.push_back(dit);
		dit = map.phi1(map.phi2(map.phi1(dit)));
	} while(dit != stop);

	map.cut_volume(edges);

	return map.phi2(stop);
}

template <typename MAP_TRAITS>
Dart swap_44(CMap3<MAP_TRAITS>& map, typename CMap3<MAP_TRAITS>::Volume w)
{
	using Volume = typename CMap3<MAP_TRAITS>::Volume;

	const Dart e = map.phi2(map.phi3(w.dart));
	const Dart dd = map.phi2(w.dart);

	//unsew middle crossing darts
	map.unsew_volumes(w);
	map.unsew_volumes(map.phi2(map.phi3(dd)));

	const Dart d1 = swap_22(map, Volume(dd));
	const Dart d2 = swap_22(map, Volume(e));

	//sew middle darts so that they do not cross
	map.sew_volumes(map.phi2(d1),map.phi2(map.phi3(d2)));
	map.sew_volumes(map.phi2(map.phi3(d1)),map.phi2(d2));
	return d1;
}

template <typename MAP_TRAITS>
Dart swap_32(CMap3<MAP_TRAITS>& map, typename CMap3<MAP_TRAITS>::Volume w)
{
	using Edge = typename CMap3<MAP_TRAITS>::Edge;
	using Face = typename CMap3<MAP_TRAITS>::Face;
	std::vector<Dart> edges;

	Dart stop = map.phi_1(map.phi2(map.phi1(w.dart)));
	Dart d2 = map.phi2(w.dart);
	Dart d21 = map.phi1(d2);
	map.merge_incident_volumes(Face(w.dart));
	map.merge_incident_faces(d2);
	map.merge_incident_volumes(Face(d21));

	Dart dit = stop;
	do
	{
		edges.push_back(dit);
		dit = map.phi1(map.phi2(map.phi1(dit)));
	}
	while(dit != stop);
	map.cut_volume(edges);

	return map.phi2(edges[0]);
}

template <typename MAP_TRAITS>
Dart swap_23(CMap3<MAP_TRAITS>& map, Dart d)
{
	using Face = typename CMap3<MAP_TRAITS>::Face;

	std::vector<Dart> edges;

	Dart d2_1 = map.phi_1(map.phi2(d));
	map.merge_incident_volumes(Face(d));

	// Cut the 1st tetrahedron
	Dart stop = d2_1;
	Dart dit = stop;
	do {
		edges.push_back(dit);
		dit = map.phi1(map.phi2(map.phi1(dit)));
	} while(dit != stop);

	map.cut_volume(edges);
	map.cut_face(map.phi3(map.phi2(edges[0])), map.phi3(map.phi2(edges[2])));

	// Cut the 2nd tetrahedron
	edges.clear();
	stop = map.phi1(map.phi2(d2_1));
	dit = stop;
	do {
		edges.push_back(dit);
		dit = map.phi1(map.phi2(map.phi1(dit)));
	} while(dit != stop);

	map.cut_volume(edges);

	return stop;
}

template <typename MAP_TRAITS>
typename CMap3<MAP_TRAITS>::Vertex flip_14(CMap3<MAP_TRAITS>& map, typename CMap3<MAP_TRAITS>::Volume w)
{
	using Vertex = typename CMap3<MAP_TRAITS>::Vertex;
	using Face = typename CMap3<MAP_TRAITS>::Face;

	std::vector<Dart> edges;

	// Cut the 1st tetrahedron
	edges.push_back(map.phi2(w.dart));
	edges.push_back(map.phi2(map.phi1(w.dart)));
	edges.push_back(map.phi2(map.phi_1(w.dart)));
	map.cut_volume(edges);

	const Vertex x = triangule(map, Face(map.phi2(w.dart)));

	// Cut the 2nd tetrahedron
	Dart dit = map.phi2(map.phi3(x.dart));
	edges.clear();
	edges.push_back(dit);
	dit = map.phi1(map.phi2(map.phi1(dit)));
	edges.push_back(dit);
	dit = map.phi1(dit);
	edges.push_back(dit);
	dit = map.phi1(map.phi2(map.phi1(dit)));
	edges.push_back(dit);

	map.cut_volume(edges);
	map.cut_face(map.phi1(map.phi2(edges[0])),map.phi1(map.phi2(edges[2])));

	// Cut the 3rd tetrahedron
	dit = map.phi3(map.phi1(map.phi2(edges[0])));
	edges.clear();
	edges.push_back(dit);
	dit = map.phi1(map.phi2(map.phi1(dit)));
	edges.push_back(dit);
	dit = map.phi1(map.phi2(map.phi1(dit)));
	edges.push_back(dit);

	map.cut_volume(edges);

	return x;
}

template <typename MAP_TRAITS>
typename CMap3<MAP_TRAITS>::Vertex flip_13(CMap3<MAP_TRAITS>& map, Dart d)
{
	using Vertex = typename CMap3<MAP_TRAITS>::Vertex;
	using Face = typename CMap3<MAP_TRAITS>::Face;

	std::vector<Dart> edges;

	// Triangule one face
	const Vertex x = triangule(map,Face(d));

	// Cut the 1st Tetrahedron
	Dart dit = x.dart;
	edges.push_back(dit);
	dit = map.phi1(map.phi2(map.phi1(dit)));
	edges.push_back(dit);
	dit = map.phi1(map.phi2(map.phi1(dit)));
	edges.push_back(dit);
	dit = map.phi1(map.phi2(map.phi1(dit)));
	edges.push_back(dit);

	map.cut_volume(edges);

	// Cut the 2nd Tetrahedron
	map.cut_face(map.phi1(map.phi2(edges[0])),map.phi1(map.phi2(edges[2])));

	// Cut the 3rd Tetrahedron
	dit = map.phi1(map.phi2(edges[0]));
	edges.clear();
	edges.push_back(dit);
	dit = map.phi1(map.phi2(map.phi1(dit)));
	edges.push_back(dit);
	dit = map.phi1(map.phi2(map.phi1(dit)));
	edges.push_back(dit);

	map.cut_volume(edges);

	return x;
}

template <typename MAP_TRAITS>
Dart edge_bisection(CMap3<MAP_TRAITS>& map, typename CMap3<MAP_TRAITS>::Edge d)
{
	map.cut_edge(d);
	const Dart e = map.phi1(d.dart);

	Dart dit = e;
	do {
		map.cut_face(dit, map.phi1(map.phi1(dit)));
		dit = map.phi3(map.phi2(dit));
	} while(dit != e);

	dit = e;
	std::vector<Dart> edges;
	do{
		if(!map.is_boundary(dit))
		{
			edges.push_back(map.phi_1(dit));
			edges.push_back(map.phi_1(map.phi2(map.phi_1(edges[0]))));
			edges.push_back(map.phi1(map.phi2(dit)));
			map.cut_volume(edges);
			edges.clear();
		}
		dit = map.phi3(map.phi2(dit));
	} while (dit != e);

	return e;
}

template <typename MAP_TRAITS>
Dart swap_gen_32(CMap3<MAP_TRAITS>& map, typename CMap3<MAP_TRAITS>::Volume d)
{
	using Edge = typename CMap3<MAP_TRAITS>::Edge;
	using Edge2 = typename CMap3<MAP_TRAITS>::Edge2;
	using Face = typename CMap3<MAP_TRAITS>::Face;

	std::vector<Dart>* edges = cgogn::dart_buffers()->buffer();
	const Dart stop = map.phi1(map.phi2(map.phi_1(d.dart)));

	if (map.delete_edge(Edge(d.dart)).is_nil())
	{
		Dart d_begin;
		{
			//		Dart d_begin = map.findBoundaryFaceOfEdge(d);
			Dart it = d.dart;
			do {
				if (map.is_boundary(it))
				{
					d_begin = it;
					break;
				}
				it = map.phi3(map.phi2(it));
			} while(it != d.dart);
		}

		map.foreach_incident_edge(d, [edges](Edge e) { edges->push_back(e.dart);});
		for (Dart it : *edges)
			map.merge_incident_volumes(Face(it));
		map.flip_back_edge(Edge2(d_begin));
		map.flip_edge(Edge2(map.phi3(d_begin)));
	}

	Dart dit = stop;
	edges->clear();
	do {
		edges->push_back(dit);
		dit = map.template phi<121>(dit);
	} while(dit != stop);

	map.cut_volume(*edges);

	const Dart v = map.phi1(map.phi2(stop));
	dit = map.phi_1(map.phi_1(v));
	do
	{
		Dart save = map.phi_1(dit);
		map.cut_face(v,dit);

		//decoupe des tetraedres d'un cote du plan
		Dart d_1 = map.phi_1(v);
		edges->clear();
		edges->push_back(d_1);
		edges->push_back(map.template phi<121>(d_1));
		edges->push_back(map.phi_1(map.phi2(map.phi_1(d_1))));
		map.cut_volume(*edges);

		//decoupe des tetraedres d'un cote du plan
		d_1 = map.phi3(map.phi_1(v));
		edges->clear();
		edges->push_back(d_1);
		edges->push_back(map.template phi<121>(d_1));
		edges->push_back(map.phi_1(map.phi2(map.phi_1(d_1))));
		map.cut_volume(*edges);

		dit = save;
	} while(map.phi_1(dit) != v);

	cgogn::dart_buffers()->release_buffer(edges);
	return stop;
}

//template <typename MAP_TRAITS>
//std::vector<Dart> swapGen3To2Optimized(CMap3<MAP_TRAITS>& map, typename CMap3<MAP_TRAITS>::Volume d);

//template <typename MAP_TRAITS>
//void swapGen2To3(CMap3<MAP_TRAITS>& map, typename CMap3<MAP_TRAITS>::Volume d);

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_TETRAHEDRALIZATION_CPP_))
extern template CGOGN_MODELING_API Dart split_vertex<DefaultMapTraits>(CMap3<DefaultMapTraits>& , std::vector<Dart>&);
extern template CGOGN_MODELING_API Dart swap_22<DefaultMapTraits>(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Volume);
extern template CGOGN_MODELING_API Dart swap_32<DefaultMapTraits>(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Volume);
extern template CGOGN_MODELING_API Dart swap_23<DefaultMapTraits>(CMap3<DefaultMapTraits>&,Dart);
//extern template CGOGN_MODELING_API Dart swap_44<DefaultMapTraits>(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Volume);
extern template CGOGN_MODELING_API CMap3<DefaultMapTraits>::Vertex flip_14<DefaultMapTraits>(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Volume);
extern template CGOGN_MODELING_API CMap3<DefaultMapTraits>::Vertex flip_13<DefaultMapTraits>(CMap3<DefaultMapTraits>&, Dart);
extern template CGOGN_MODELING_API Dart edge_bisection(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Edge);
extern template CGOGN_MODELING_API Dart swap_gen_32(CMap3<DefaultMapTraits>&, CMap3<DefaultMapTraits>::Volume);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_TETRAHEDRALIZATION_CPP_))

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_TETRAHEDRALIZATION_H_

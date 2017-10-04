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

#ifndef CGOGN_TOPOLOGY_ADAPTIVE_TRI_QUAD_CMAP2_H_
#define CGOGN_TOPOLOGY_ADAPTIVE_TRI_QUAD_CMAP2_H_

#include <cgogn/core/cmap/cmap2.h>

namespace cgogn
{

class AdaptiveTriQuadCMap2
{
public:

	using CDart = typename CMap2::CDart;
	using Vertex = typename CMap2::Vertex;
	using Edge = typename CMap2::Edge;
	using Face = typename CMap2::Face;

	enum FaceType: uint8
	{
		TRI_CORNER = 0,
		TRI_CENTRAL,
		QUAD
	};

	AdaptiveTriQuadCMap2(CMap2& map);
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(AdaptiveTriQuadCMap2);
	~AdaptiveTriQuadCMap2();

	void init();

	template <typename CUT_EDGE_FUNC, typename PRE_SPLIT_FACE_FUNC, typename POST_SPLIT_FACE_FUNC>
	void subdivide_face(
		Face f,
		const CUT_EDGE_FUNC& cut_edge_func,
		const PRE_SPLIT_FACE_FUNC& pre_split_face_func,
		const POST_SPLIT_FACE_FUNC& post_split_face_func
	)
	{
		static_assert(is_func_parameter_same<CUT_EDGE_FUNC, Vertex>::value, "Wrong cut_edge function parameter type");
		static_assert(is_func_parameter_same<PRE_SPLIT_FACE_FUNC, Face>::value, "Wrong pre_split_face function parameter type");
		static_assert(is_func_parameter_same<POST_SPLIT_FACE_FUNC, Face>::value, "Wrong post_split_face function parameter type");

		f.dart = oldest_dart(f);
		uint8 fl = face_level(f);
		uint8 fid = face_subd_id_[f];

		// enforce neighbours level difference is not greater than 1
		map_.foreach_adjacent_face_through_edge(f, [&] (Face af)
		{
			if (face_level(af) < fl)
				subdivide_face(af, cut_edge_func, pre_split_face_func, post_split_face_func);
		});

		// cut edges (if not already done)
		Dart it = f.dart;
		do
		{
			Dart next = map_.phi1(it);
			if (dart_level_[next] > fl)
				next = map_.phi1(next);
			else
			{
				Vertex v = map_.cut_edge(Edge(it));
				if (map_.is_incident_to_boundary(Edge(it)))
					dart_level_[map_.phi1(it)] = fl+1;
				else
				{
					dart_level_[map_.phi1(it)] = fl+1;
					dart_level_[map_.phi2(it)] = fl+1;
				}

				cut_edge_func(v);
			}
			it = next;
		} while (it != f.dart);

		pre_split_face_func(f);

		if (tri_face_[f])
		{
			// cut face into 4 triangles
			it = map_.phi1(it);
			Dart it2 = map_.phi<11>(it);
			Edge e = map_.cut_face(it, it2);
			dart_level_[e.dart] = fl + 1;
			dart_level_[map_.phi2(e.dart)] = fl + 1;
			face_subd_id_[Face(it)] = 4*fid + 1;
			tri_face_[Face(it)] = true;

			it = map_.phi<11>(it2);
			e = map_.cut_face(it, it2);
			dart_level_[e.dart] = fl + 1;
			dart_level_[map_.phi2(e.dart)] = fl + 1;
			face_subd_id_[Face(it2)] = 4*fid + 2;
			tri_face_[Face(it2)] = true;

			it2 = map_.phi<11>(it);
			e = map_.cut_face(it, it2);
			dart_level_[e.dart] = fl + 1;
			dart_level_[map_.phi2(e.dart)] = fl + 1;
			face_subd_id_[Face(it)] = 4*fid + 3;
			tri_face_[Face(it)] = true;

			face_subd_id_[Face(it2)] = 4*fid + 4;
			tri_face_[Face(it2)] = true;
		}
		else
		{
			// cut face into quads
			it = map_.phi1(it);
			Dart it2 = map_.phi<11>(it);
			Edge e = map_.cut_face(it, it2);
			dart_level_[e.dart] = fl + 1;
			dart_level_[map_.phi2(e.dart)] = fl + 1;
			tri_face_[Face(it2)] = false;

			Vertex v = map_.cut_edge(e);
			dart_level_[map_.phi1(e.dart)] = fl + 1;
			dart_level_[map_.phi2(e.dart)] = fl + 1;

			it = map_.phi2(e.dart);
			it2 = map_.phi<11>(it2);
			do
			{
				Edge ee = map_.cut_face(it, it2);
				dart_level_[ee.dart] = fl + 1;
				dart_level_[map_.phi2(ee.dart)] = fl + 1;
				tri_face_[Face(it2)] = false;

				it = map_.phi2(map_.phi_1(it));
				it2 = map_.phi<11>(it2);
			} while (map_.phi1(it2) != it);

			uint32 cmpt = 0;
			map_.foreach_incident_face(v, [&] (Face af)
			{
				cmpt++;
				face_subd_id_[af] = 4*fid + cmpt;
			});
		}

		post_split_face_func(f);
	}

	bool is_simplifiable(Face f);

	template <typename PRE_MERGE_FACES_FUNC, typename POST_MERGE_FACES_FUNC, typename MERGE_EDGES_FUNC>
	Face simplify_face(
		Face f,
		const PRE_MERGE_FACES_FUNC& pre_merge_faces_func,
		const POST_MERGE_FACES_FUNC& post_merge_faces_func,
		const MERGE_EDGES_FUNC& merge_edges_func
	)
	{
		static_assert(is_func_parameter_same<PRE_MERGE_FACES_FUNC, Face>::value, "Wrong pre_merge_faces function parameter type");
		static_assert(is_func_parameter_same<POST_MERGE_FACES_FUNC, Face>::value, "Wrong post_merge_faces function parameter type");
		static_assert(is_func_parameter_same<MERGE_EDGES_FUNC, Edge>::value, "Wrong merge_edges function parameter type");

		// if we are here, f is simplifiable (part of a group of 4 triangle or quad faces)

		uint32 fid = face_subd_id_[f];
		uint8 fl = face_level(f);

		Face resF;

		pre_merge_faces_func(f);

		if (tri_face_[f])
		{
			Dart it = f.dart;
			switch (face_type(f))
			{
				case TRI_CORNER: {
					Dart od = oldest_dart(f);
					it = map_.phi<12>(od); // put 'it' in the central face
					resF.dart = od;
					break;
				}
				case TRI_CENTRAL: {
					resF.dart = map_.phi_1(map_.phi2(f.dart));
					break;
				}
			}

			Dart next = map_.phi1(it);
			map_.merge_incident_faces(Edge(map_.phi2(it)));
			it = next;
			next = map_.phi1(it);
			map_.merge_incident_faces(Edge(it));
			it = next;
			map_.merge_incident_faces(Edge(it));

			tri_face_[resF] = true;
		}
		else
		{
			Dart od = oldest_dart(f);
			resF.dart = od;
			Dart it = map_.phi<11>(od); // central vertex

			map_.merge_incident_faces(Vertex(it));

			tri_face_[resF] = false;
		}

		post_merge_faces_func(resF);

		// simplify edges (if possible)
		Dart it = resF.dart;
		do
		{
			Dart next = map_.phi<11>(it);
			if (map_.is_incident_to_boundary(Edge(map_.phi2(it))) || face_level(Face(map_.phi2(it))) == fl-1)
			{
				map_.merge_incident_edges(Vertex(map_.phi1(it)));

				merge_edges_func(Edge(it));
			}
			it = next;
		} while (it != resF.dart);

		face_subd_id_[resF] = uint32((fid-1) / 4);

		return resF;
	}

	inline uint8 dart_level(Dart d) { return dart_level_[d]; }
	uint8 face_level(Face f);
	FaceType face_type(Face f);
	inline bool is_triangle_face(Face f) { return tri_face_[f]; }
	Dart oldest_dart(Face f);

protected:

	CMap2& map_;

	CMap2::CDartAttribute<uint8> dart_level_; // dart insertion level
	CMap2::FaceAttribute<uint32> face_subd_id_; // face subdivision id
	CMap2::FaceAttribute<bool> tri_face_; // face is triangle or not
};

} // namespace cgogn

#endif // CGOGN_TOPOLOGY_ADAPTIVE_TRI_QUAD_CMAP2_H_

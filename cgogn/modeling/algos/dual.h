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

#ifndef CGOGN_MODELING_ALGOS_DUAL_H_
#define CGOGN_MODELING_ALGOS_DUAL_H_

#include <vector>
#include <cgogn/modeling/dll.h>
#include <cgogn/core/basic/dart_marker.h>
#include <cgogn/geometry/algos/centroid.h>


namespace cgogn
{

namespace modeling
{

template <typename MAP>
bool dual2_topo(const MAP& src, MAP& dst, bool embed_vertices = true, bool embed_edges = false, bool embed_faces = false)
{
	using Vertex = MAP::Vertex;
	using Edge = MAP::Edge;
	using Face = MAP::Face;

	dst.clear_and_remove_attributes();

	const auto& tc = src.topology_container();
	uint32 nb = tc.end();

	std::vector<Dart> corr(nb);
	MAP::Builder build(dst);

	// create a face in dst for each vertex in src and keep a table src -> dst
	src.foreach_cell([&](Vertex v)
	{
		uint32 nb = src.degree(v);
		Dart df = build.add_face_topo_fp(nb);
		src.foreach_incident_edge(v, [&](Edge e)
		{
			corr[e.dart.index] = df;
			df = dst.phi1(df);
		});
	});

	// sew new faces using table
	bool open = false;
	src.foreach_cell([&](MAP::Edge e) -> bool
	{
		Dart d1 = corr[e.dart.index];
		Dart d2 = corr[(src.phi2(e.dart)).index];
		if (src.is_boundary(d1) || src.is_boundary(d1))
		{
			cgogn_log_error("dual") << " can not compute dual of open map";
			dst.clear();
			open = true;
			return false;
		}
		build.phi2_sew(d1, d2);
		return true;
	});
	if (open)
		return false;

	if (embed_vertices)
	{
		auto face_of_src = dst.add_attribute<Face, Vertex>("FaceOfSrc");
		src.foreach_cell([&](Face f)
		{
			Vertex v(dst.phi2(corr[f.dart.index]));
			face_of_src[v] = f;
		});
	}

	if (embed_edges)
	{
		auto edge_of_src = dst.add_attribute<Edge, Edge>("EdgeOfSrc");
		src.foreach_cell([&](Edge e)
		{
			Edge ee(corr[e.dart.index]);
			edge_of_src[ee] = e;
		});
	}

	if (embed_faces)
	{
		auto vertex_of_src = dst.add_attribute<Vertex, Face>("VertexOfSrc");
		src.foreach_cell([&](Vertex v)
		{
			Face f(corr[v.dart.index]);
			vertex_of_src[f] = v;
		});
	}
	return true;
}

template <typename VEC, typename MAP>
void compute_dual2_vertices(const MAP& src, MAP& dst, const typename MAP::template VertexAttribute<VEC>& position_src)
{
	using Vertex = MAP::Vertex;
	using Face = MAP::Face;

	auto face_of_src = dst.get_attribute<Face, Vertex>("FaceOfSrc");
	if (!face_of_src.is_valid())
	{
		cgogn_log_error("compute_dual_vertices") << "attribute FaceOfSrc not found";
		return;
	}

	auto position = dst.get_attribute<Vec3, Vertex>("position");
	if (!position.is_valid())
		position = dst.add_attribute<Vec3, Vertex>("position");

	dst.foreach_cell([&](Vertex v)
	{
		Face f = Face(face_of_src[v]);
		position[v] = cgogn::geometry::centroid<Vec3>(src, f, position_src);
	});
}




} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_DUAL_H_

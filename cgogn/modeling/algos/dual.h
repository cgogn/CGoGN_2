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


namespace cgogn
{

namespace modeling
{

#ifdef GENERIC_VERSION

/**
 * @brief compute the topological dual of a CMap2
 * @param src source mesh
 * @param dst dual mesh (will be cleared)
 * @param embed_vertices vertex attribute (of Face) "FaceOfSrc" must be computed
 * @param embed_edges edge attribute (of Edge) "EdgeOfSrc" must be computed
 * @param embed_faces face attribute (of Vertex) "VertexOfSrc" must be computed
 * @return true of dual has been computed
 */
template <typename MAP>
auto dual2_topo(const MAP& src, MAP& dst, bool embed_vertices = true, bool embed_edges = false, bool embed_faces = false)
-> typename std::enable_if<MAP::DIMENSION == 2, bool>::type
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	dst.clear_and_remove_attributes();

	const auto& tc = src.topology_container();
	uint32 nb = tc.end();

	std::vector<Dart> corresp(nb);
	typename MAP::Builder build(dst);

	// create a face in dst for each vertex in src and keep a table src -> dst
	src.foreach_cell([&](Vertex v)
	{
		uint32 nb = src.degree(v);
		Dart df = build.add_face_topo_fp(nb);
		src.foreach_incident_edge(v, [&](Edge e)
		{
			corresp[e.dart.index] = df;
			df = dst.phi1(df);
		});
	});

	// sew new faces using table
	bool open = false;
	src.foreach_cell([&](Edge e) -> bool
	{
		Dart d1 = corresp[e.dart.index];
		Dart d2 = corresp[(src.phi2(e.dart)).index];
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
		auto face_of_src = dst.template add_attribute<Face, Vertex>("FaceOfSrc");
		src.foreach_cell([&](Face f)
		{
			Vertex v(dst.phi2(corresp[f.dart.index]));
			face_of_src[v] = f;
		});
	}

	if (embed_edges)
	{
		auto edge_of_src = dst.template add_attribute<Edge, Edge>("EdgeOfSrc");
		src.foreach_cell([&](Edge e)
		{
			Edge ee(corresp[e.dart.index]);
			edge_of_src[ee] = e;
		});
	}

	if (embed_faces)
	{
		auto vertex_of_src = dst.template add_attribute<Vertex, Face>("VertexOfSrc");
		src.foreach_cell([&](Vertex v)
		{
			Face f(corresp[v.dart.index]);
			vertex_of_src[f] = v;
		});
	}
	return true;
}

/**
 * @brief compute vertices of the dual mesh (centers of faces)
 * @param src source mesh
 * @param dst destination dual mesh ( topo already computed)
 * @param position_src source position attributee
 */
template <typename VEC, typename MAP>
auto compute_dual2_vertices(const MAP& src, MAP& dst, const typename MAP::template VertexAttribute<VEC>& position_src)
-> typename std::enable_if<MAP::DIMENSION == 2, void>::type
{
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;

	auto face_of_src = dst.template get_attribute<Face, Vertex>("FaceOfSrc");
	if (!face_of_src.is_valid())
	{
		cgogn_log_error("compute_dual_vertices") << "attribute FaceOfSrc not found";
		return;
	}

	auto position = dst.template get_attribute<VEC, Vertex>("position");
	if (!position.is_valid())
		position = dst.template add_attribute<VEC, Vertex>("position");

	dst.foreach_cell([&](Vertex v)
	{
		Face f = Face(face_of_src[v]);
		position[v] = cgogn::geometry::centroid<VEC>(src, f, position_src);
	});
}



template <typename MAP>
auto dual2_topo(const MAP& src, MAP& dst)
-> typename std::enable_if<MAP::DIMENSION == 2, bool>::type
{
	cgogn_message_assert(src.nb_boundaries() == 0u, "dual2_topo can only be used on maps without boundaries");
	cgogn_message_assert(src.topology_container().end() == src.topology_container().size(), "dual2_topo can only be used on compacted maps");

	// first to be sure to add in an empty map
	dst.clear_and_remove_attributes();

	// access to low level topo
	typename MAP::Builder build_dst(dst);
	typename MAP::template ChunkArray<Dart>& phi1_dst  = build_dst.ca_phi1();
	typename MAP::template ChunkArray<Dart>& phi_1_dst = build_dst.ca_phi_1();
	typename MAP::template ChunkArray<Dart>& phi2_dst  = build_dst.ca_phi2();

	src.foreach_dart([&](Dart d)
	{
		Dart e = build_dst.add_topology_element();
		phi1_dst[e.index] = src.phi2(src.phi_1(d));
		phi2_dst[e.index] = src.phi2(d);
	});

	// second pass for phi_1, because we can not affect phi_1 on a dart which does not exist
	dst.foreach_dart([&] (Dart d)
	{
		Dart dd = dst.phi1(d);
		phi_1_dst[dd.index] = d;
	});

	return true;
}


#endif



/**
 * @brief dual2_topo (work only with closed map2)
 * @param src source mesh
 * @param dst dual result mesh (overwriten)
 * @return true if computed
 */
template <typename MAP>
auto dual2_topo(const MAP& src, MAP& dst)
-> typename std::enable_if<MAP::DIMENSION == 2, bool>::type
{
	cgogn_message_assert(src.nb_boundaries() == 0u, "dual2_topo can only be used on maps without boundaries");

	// first to be sure to add in an empty map
	dst.clear_and_remove_attributes();

	// access to low level topo
	typename MAP::Builder build_dst(dst);
	auto& topo_dst = build_dst.cac_topology();
	auto& phi1_dst  = build_dst.ca_phi1();
	auto& phi_1_dst = build_dst.ca_phi_1();
	auto& phi2_dst  = build_dst.ca_phi2();

	typename MAP::Builder build_src(const_cast<MAP&>(src));
	const auto& phi2_src  = build_src.ca_phi2();
	const auto& topo_src = build_src.cac_topology();

	// init topo with same number of dart and holes ...
	topo_dst.copy_all_but_data(&topo_src);
	// copy phi2
	phi2_dst.copy_data(phi2_src);

	// set phi1/phi_1 (//)
	src.parallel_foreach_dart([&](Dart d)
	{
		Dart dd = src.phi2(src.phi_1(d));
		phi1_dst[d.index] = dd;
		phi_1_dst[dd.index] = d;
	});
	return true;
}


/**
 * @brief compute_dual2_vertices (Face of source = phi2(v.dart) of dual)
 * @param dst dual mesh (topo only)
 * @param position_dst dual vertex positions
 * @param compute lambda (Face f) -> VEC
 */
template <typename VEC, typename MAP, typename FUNC>
auto compute_dual2_vertices(MAP& dst, typename MAP::template VertexAttribute<VEC>& position_dst, const FUNC& compute)
-> typename std::enable_if<MAP::DIMENSION == 2, void>::type
{
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;

	dst.parallel_foreach_cell([&](Vertex v)
	{
		position_dst[v] = compute(Face(dst.phi2(v.dart)));
	});
}


/**
 * @brief compute_dual2_faces (Vertex of source = (f.dart) of dual)
 * @param dst dual mesh (topo only)
 * @param att_dst dual face attribute
 * @param compute lambda (Vertex v) -> ATT
 */
template <typename ATT, typename MAP, typename FUNC>
auto compute_dual2_faces(MAP& dst, typename MAP::template FaceAttribute<ATT>& att_dst, const FUNC& compute)
-> typename std::enable_if<MAP::DIMENSION == 2, void>::type
{
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;

	dst.parallel_foreach_cell([&](Face f)
	{
		att_dst[f] = compute(Vertex(f.dart));
	});
}


/**
 * @brief compute_dual2_edges (Edge of source = edge of dual)
 * @param dst dual mesh (topo only)
 * @param att_dst dual edge attribute
 * @param compute lambda (Edge v) -> ATT
 */
template <typename ATT, typename MAP, typename FUNC>
auto compute_dual2_edges(MAP& dst, typename MAP::template EdgeAttribute<ATT>& att_dst, const FUNC& compute)
-> typename std::enable_if<MAP::DIMENSION == 2, void>::type
{
	using Edge = typename MAP::Edge;

	dst.parallel_foreach_cell([&]( Edge e)
	{
		att_dst[e] = compute(e);
	});
}




} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_DUAL_H_

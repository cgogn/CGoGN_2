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
#include<cgogn/core/cmap/cmap2.h>
#include<cgogn/core/utils/type_traits.h>

namespace cgogn
{

namespace modeling
{

#ifdef GENERIC_VERSION

/**
 * @brief compute the topological dual of a Map2
 * @param src source mesh
 * @param dst dual mesh (will be cleared)
 * @param embed_vertices vertex attribute (of Face) "FaceOfSrc" must be computed
 * @param embed_edges edge attribute (of Edge) "EdgeOfSrc" must be computed
 * @param embed_faces face attribute (of Vertex) "VertexOfSrc" must be computed
 * @return true of dual has been computed
 */
template <typename MAP>
auto gen_dual2_topo(const MAP& src, MAP& dst, bool embed_vertices = true, bool embed_edges = false, bool embed_faces = false)
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
auto compute_gen_dual2_vertices(const MAP& src, MAP& dst, const typename MAP::template VertexAttribute<VEC>& position_src)
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


#endif


#ifdef SECOND_VERSION

/**
 * @brief dual2_topo (work only with closed CMap2)
 * @param src source mesh
 * @param dst dual result mesh (overwriten)
 * @return true if computed
 */
inline void dual2_topo(const CMap2& src, CMap2& dst)
{
	// first to be sure to add in an empty map
	dst.clear_and_remove_attributes();

	// access to low level topo
	CMap2::Builder build_dst(dst);
	auto& topo_dst = build_dst.cac_topology();
	auto& phi1_dst  = build_dst.ca_phi1();
	auto& phi_1_dst = build_dst.ca_phi_1();
	auto& phi2_dst  = build_dst.ca_phi2();

	CMap2::Builder build_src(const_cast<CMap2&>(src));
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
}


/**
 * @brief compute_dual2_vertices (Face of source = phi2(v.dart) of dual)
 * @param dst dual mesh (topo only)
 * @param position_dst dual vertex positions
 * @param compute lambda (Face f) -> VEC
 */
template <typename VEC, typename FUNC>
void compute_dual2_vertices(CMap2& dst, CMap2::VertexAttribute<VEC>& position_dst, const FUNC& compute)
{
	dst.parallel_foreach_cell([&](CMap2::Vertex v)
	{
		position_dst[v] = compute(CMap2::Face(dst.phi2(v.dart)));
	});
}


/**
 * @brief compute_dual2_faces (Vertex of source = (f.dart) of dual)
 * @param dst dual mesh (topo only)
 * @param att_dst dual face attribute
 * @param compute lambda (Vertex v) -> ATT
 */
template <typename ATT, typename FUNC>
void compute_dual2_faces(CMap2& dst, CMap2::FaceAttribute<ATT>& att_dst, const FUNC& compute)
{
	dst.parallel_foreach_cell([&](CMap2::Face f)
	{
		att_dst[f] = compute(CMap2::Vertex(f.dart));
	});
}


/**
 * @brief compute_dual2_edges (Edge of source = edge of dual)
 * @param dst dual mesh (topo only)
 * @param att_dst dual edge attribute
 * @param compute lambda (Edge v) -> ATT
 */
template <typename ATT, typename FUNC>
void compute_dual2_edges(CMap2& dst, CMap2::EdgeAttribute<ATT>& att_dst, const FUNC& compute)
{
	dst.parallel_foreach_cell([&]( CMap2::Edge e)
	{
		att_dst[e] = compute(e);
	});
}

/**
 * @brief mark vertices which are dual of boundarie faces
 * @param src source mesh
 * @param marker vertex dual-boundary marker
 */
inline void mark_boundaries(const CMap2& src, CMap2::CellMarker<CMap2::Vertex::ORBIT>& marker)
{
	src.foreach_cell([&](CMap2::Face f)
	{
		if (src.is_boundary(f.dart))
			marker.mark(CMap2::Vertex(f.dart));
	});
}

#endif

namespace Dual_internal
{

inline void dual2_topo(const CMap2& src, CMap2& dst)
{
	// first to be sure to add in an empty map
	dst.clear_and_remove_attributes();

	// access to low level topo
	CMap2::Builder build_dst(dst);
	auto& topo_dst = build_dst.cac_topology();
	auto& phi1_dst  = build_dst.ca_phi1();
	auto& phi_1_dst = build_dst.ca_phi_1();
	auto& phi2_dst  = build_dst.ca_phi2();

	CMap2::Builder build_src(const_cast<CMap2&>(src));
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
}


template <typename T>
struct DualType2
{};

template<>
struct DualType2<CMap2::Vertex>
{
	using type = CMap2::Face;
	static inline CMap2::Vertex good_cell(CMap2&, type c) { return CMap2::Vertex(c.dart); }
};

template<>
struct DualType2<CMap2::Face>
{
	using type = CMap2::Vertex;
	static inline CMap2::Face good_cell(CMap2& m, type c) { return CMap2::Face(m.phi2(c.dart)); }
};

template<>
struct DualType2<CMap2::Edge>
{
	using type = CMap2::Edge;
	static inline type good_cell(CMap2&, type c) { return type(c.dart); }
};



template <int N=0, typename F>
void dual(const CMap2& src, CMap2& dst, CMap2::CellMarker<CMap2::Vertex::ORBIT>* marker,
		  const std::vector<std::string>& att_name, const F& func)
{
	if  (N==0)
	{
		dual2_topo(src, dst);
		if ((marker != nullptr) && (marker->is_valid()))
		{
			// at this point dst is pure topo
			CMap2::Builder(dst).create_embedding<CMap2::Vertex::ORBIT>();
			dst.foreach_cell([&](CMap2::Vertex v)
			{
				if (src.is_boundary(v.dart))
					marker->mark(v);
			});
		}
	}

	using CELL_SRC = func_ith_parameter_type<F,0>;
	using CELL_DST = typename DualType2<CELL_SRC>::type;
	using ATTR     = func_return_type<F>;

	auto att = dst.add_attribute<ATTR, CELL_DST>(att_name[N]);

	dst.parallel_foreach_cell([&](CELL_DST c)
	{
		att[c] = func(DualType2<CELL_SRC>::good_cell(dst,c));
	});
}

}

/**
 * @brief dual
 * @param src source mesh
 * @param dst computed dual mesh
 * @param marker if is not null mark vertices that are dual of boundary faces
 * @param att_name names of attributs to create & compute
 * @param func lambdas of computing attribut (DualCell) -> attribute_type, ordrer of lambdas must be coherent with att_names
 */

template <int N=0, typename F>
void dual(const CMap2& src, CMap2& dst, CMap2::CellMarker<CMap2::Vertex::ORBIT>* marker,
		  const std::vector<std::string>& att_name, const F& func)
{
	if ((N==0) && (att_name.size()!=1))
		cgogn_log_error("dual") <<"number of attribute names must equal to number of computing lambdas";

	Dual_internal::dual<N>(src,dst,marker,att_name,func);
}

template <int N=0, typename F, typename... Args>
void dual(const CMap2& src, CMap2& dst, CMap2::CellMarker<CMap2::Vertex::ORBIT>* marker,
		  const std::vector<std::string>& att_name, const F& func, Args... args)
{
	if ((N==0) && (att_name.size()!= sizeof...(args)+1))
		cgogn_log_error("dual") <<"number of attribute names must equal to number of computing lambdas";

	Dual_internal::dual<N>(src,dst,marker,att_name,func);
	dual<N+1>(src, dst, marker, att_name, args...);
}


} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_DUAL_H_

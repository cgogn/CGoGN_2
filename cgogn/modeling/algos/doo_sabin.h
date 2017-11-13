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

#ifndef CGOGN_MODELING_ALGOS_DOO_SABIN_H_
#define CGOGN_MODELING_ALGOS_DOO_SABIN_H_

#include <vector>

#include <cgogn/modeling/dll.h>
#include <cgogn/core/basic/dart_marker.h>
#include <cgogn/core/utils/masks.h>
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/geometry/types/geometry_traits.h>

namespace cgogn
{

namespace modeling
{

/// \brief Subdivides a surface mesh with the Doo Sabin algorithm
/// \param[in, out] map the surface to be subdivded
/// \param[in, out] position the geometric position of the surface
/// \todo handle objects with open boundaries
template <typename VEC3, typename MAP>
void doo_sabin(MAP& map,
			   typename MAP::template VertexAttribute<VEC3>& position)
{
	using Scalar = typename VEC3::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;
	using Builder = typename MAP::Builder;
	using CellCache = typename MAP::CellCache;

	// storage of boundary of hole (missing vertex faces)
	std::vector<Dart>* fp = cgogn::dart_buffers()->buffer();

	Builder mbuild(map);

	CellCache initial_cache(map);
	initial_cache.template build<Edge>();
	initial_cache.template build<Face>();

	// create the edge faces
	map.foreach_cell([&] (Edge e)
	{
		Dart e2 = map.phi2(e.dart);
		mbuild.phi2_unsew(e.dart);
		Dart nf = mbuild.add_face_topo_fp(4);
		mbuild.phi2_sew(e.dart, nf);
		mbuild.phi2_sew(e2, map.phi1(map.phi1(nf)));

		///todo take care of the edge embedding
		fp->push_back(map.phi1(nf));
		fp->push_back(map.phi_1(nf));
		if(map.template is_embedded<Edge>())
		{
			mbuild.template set_orbit_embedding<Edge>(Edge(e), map.embedding(Edge(e)));
			mbuild.new_orbit_embedding(Edge(e2));
		}

		if(map.template is_embedded<Face>())
			mbuild.new_orbit_embedding(Face(nf));
	}
	, initial_cache);

	// create the new vertex faces (by filling the holes)
	for(uint32 i = 0; i < fp->size(); ++i)
	{
		const Dart e = (*fp)[i];

		if (map.phi2(e) == e)
		{
			Dart h = mbuild.close_hole_topo(e);
			if(map.codegree(Face(h)) == 2)
			{
				//handle degree 2 vertices that turns
				//into edges (degenerated two sided faces)
				Dart h2 = map.phi2(h);
				Dart h12 = map.phi2(map.phi1(h));
				mbuild.phi2_unsew(h2);
				mbuild.phi2_unsew(h12);
				mbuild.remove_face_topo_fp(h);
				mbuild.phi2_sew(h2, h12);

				if(map.template is_embedded<Edge>())
					mbuild.new_orbit_embedding(Edge(h2));
			}
			else
			{
				if(map.template is_embedded<Edge>())
				{
					map.foreach_incident_edge(Face(h), [&](Edge e)
					{
						mbuild.new_orbit_embedding(e);
					});
				}

				if(map.template is_embedded<Face>())
					mbuild.new_orbit_embedding(Face(h));
			}
		}
	}
	cgogn::dart_buffers()->release_buffer(fp);

	std::vector<VEC3> buffer;
	buffer.reserve(8);
	map.foreach_cell([&] (Face f) {
		Dart e = f.dart;

		do
		{
			buffer.push_back(position[Vertex(e)]);
			e = map.phi1(e);
		}while (e != f.dart);

		int N = buffer.size();
		for (int i = 0; i < N; ++i)
		{
			VEC3 P;
			P.setZero();
			for (int j = 0; j < N; ++j)
			{
				if (j==i)
				{
					Scalar c1 = Scalar(N+5)/Scalar(4*N);
					P += buffer[j]*c1;
				}
				else
				{
					Scalar c2 = (3.0+2.0*std::cos(2.0*M_PI*(Scalar(i-j))/Scalar(N))) /(4.0*N);
					P += c2*buffer[j];
				}
			}
			mbuild.new_orbit_embedding(Vertex(e));
			position[Vertex(e)] = P;
			e = map.phi1(e);
		}
		buffer.clear();
	},
	initial_cache);
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_DOO_SABIN_CPP_))
extern template CGOGN_MODELING_API void doo_sabin<Eigen::Vector3f, CMap2>(CMap2&, CMap2::VertexAttribute<Eigen::Vector3f>&);
extern template CGOGN_MODELING_API void doo_sabin<Eigen::Vector3d, CMap2>(CMap2&, CMap2::VertexAttribute<Eigen::Vector3d>&);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_ALGOS_DOO_SABIN_CPP_))

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_DOO_SABIN_H_

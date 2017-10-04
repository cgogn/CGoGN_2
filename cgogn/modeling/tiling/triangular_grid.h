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

#ifndef CGOGN_MODELING_TILING_TRIANGULAR_GRID_H_
#define CGOGN_MODELING_TILING_TRIANGULAR_GRID_H_

#include <cgogn/core/cmap/cmap2_builder.h>
#include <cgogn/modeling/tiling/tiling.h>

namespace cgogn
{

namespace modeling
{

template <typename MAP>
class TriangularGrid : public Tiling<MAP>
{
	using CDart = typename MAP::CDart;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;
	using Volume = typename MAP::Volume;

public:

	template <typename INNERMAP>
	class GridTopo
	{
	public:

		//@{
		//! Create a 2D grid
		/*! @param[in] x nb of squares in x
		 *  @param[in] y nb of squares in y
		 */
		GridTopo(Tiling<INNERMAP>* g, uint32 x, uint32 y)
		{
			using Vertex = typename INNERMAP::Vertex;
			using Face = typename INNERMAP::Face;

			using MapBuilder = typename INNERMAP::Builder;
			MapBuilder mbuild(g->map_);

			const uint32 nb_vertices = (x+1)*(y+1);
			const uint32 nb_faces = 2*x*y;

			g->vertex_table_.reserve(nb_vertices);
			g->face_table_.reserve(nb_faces);

			//creation of triangles and storing vertices
			for(uint32 i = 0 ; i < y ; ++i)
			{
				for(uint32 j = 1 ; j <= x ; ++j)
				{
					Dart d = mbuild.add_face_topo_fp(3);
					Dart d2 = mbuild.add_face_topo_fp(3);
					mbuild.phi2_sew(g->map_.phi1(d), g->map_.phi_1(d2));

					g->vertex_table_.push_back(Vertex(d));
					g->edge_table_.push_back(Edge(d));

					g->face_table_.push_back(Face(d));
					g->face_table_.push_back(Face(d2));

					if (j == x)
						g->vertex_table_.push_back(Vertex(d2));
				}
			}

			// store last row of vertices
			const uint32 idx = (x+1)*(y-1);
			for (uint32 i = 0; i < x; ++i)
				g->vertex_table_.push_back(Vertex(g->map_.phi_1(g->vertex_table_[idx+i].dart)));

			g->vertex_table_.push_back(Vertex(g->map_.phi1(g->vertex_table_[idx+x].dart)));

			//sewing pairs of triangles
			const uint32 nb_x = (x+1);
			for (uint32 i = 0; i < y; ++i)
			{
				for (uint32 j = 0; j < x; ++j)
				{
					if (i > 0) // sew with preceeding row
					{
						const int32 pos = i*nb_x+j;
						Dart d = g->vertex_table_[pos].dart;
						Dart e = g->vertex_table_[pos-nb_x].dart;
						e = g->map_.phi_1(g->map_.phi2(g->map_.phi1(e)));
						mbuild.phi2_sew(d, e);
					}
					if (j > 0) // sew with preceeding column
					{
						const int32 pos = i*nb_x+j;
						Dart d = g->vertex_table_[pos].dart;
						d = g->map_.phi_1(d);
						Dart e = g->vertex_table_[pos-1].dart;
						e = g->map_.phi1(g->map_.phi2(g->map_.phi1(e)));
						mbuild.phi2_sew(d, e);
					}
				}
			}
		}
		//@}
	};

	TriangularGrid(MAP& map, uint32 x, uint32 y) :
		Tiling<MAP>(map)
	{
		this->nx_ = x;
		this->ny_ = y;
		this->nz_ = UINT32_MAX;

		GridTopo<MAP>(this, x, y);

		this->dart_ = this->vertex_table_[0].dart;

		using MapBuilder = typename MAP::Builder;
		MapBuilder mbuild(this->map_);

		// close the hole
		Dart f = mbuild.close_hole_topo(this->dart_);
		mbuild.boundary_mark(Face(f));

		// embed the cells

		if (this->map_.template is_embedded<CDart>())
		{
			this->map_.foreach_dart_of_orbit(Volume(this->dart_), [&] (Dart d)
			{
				if (!this->map_.is_boundary(d))
					mbuild.new_orbit_embedding(CDart(d));
			});
		}

		if (this->map_.template is_embedded<Vertex>())
		{
			for (Vertex v : this->vertex_table_)
				mbuild.new_orbit_embedding(v);
		}

		if (this->map_.template is_embedded<Edge>())
		{
			this->map_.foreach_incident_edge(Volume(this->dart_), [&] (Edge e)
			{
				mbuild.new_orbit_embedding(e);
			});
		}

		if (this->map_.template is_embedded<Face>())
		{
			this->map_.foreach_incident_face(Volume(this->dart_), [&] (Face f)
			{
				mbuild.new_orbit_embedding(f);
			});
		}

		if (this->map_.template is_embedded<Volume>())
			mbuild.new_orbit_embedding(Volume(this->dart_));
	}

	/*! @name Embedding Operators
	 *************************************************************************/

	//@{
	//! Embed a topological grid into a geometrical grid
	/*! @param[in] attribute Attribute used to store vertices positions
	 *  @param[in] x size in X
	 *  @param[in] x size in Y
	 *  @param[in] y attribute in Z (centered on 0 by default)
	 */
	template <typename T>
	void embed_into_grid(typename MAP::template VertexAttribute<T>& attribute,
						 float32 x,
						 float32 y,
						 float32 z)
	{
		const float32 dx = x / float32(this->nx_);
		const float32 dy = y / float32(this->ny_);

		for(uint32 i = 0; i <= this->ny_; ++i)
		{
			for(uint32 j = 0; j <= this->nx_; ++j)
			{
				attribute[this->vertex_table_[i*(this->nx_+1)+j]] =
						T(dx*float32(j)+dx*0.5f*float32(i),
						  dy*float32(i)*std::sqrt(3.0f) / 2.0f,
						  z);
			}
		}
	}

	//! Embed a topological grid into a twister open ribbon
	/*! @details with turns=PI it is a Moebius strip, needs only to be closed (if model allows it)
	 *  @param[in] attribute Attribute used to store vertices positions
	 *  @param[in] radius_min
	 *  @param[in] radius_max
	 *  @param[in] turns number of turn multiplied by 2*PI
	 */
	template <typename T>
	void embed_into_twisted_strip(typename MAP::template VertexAttribute<T>& attribute,
								  float32 radius_min,
								  float32 radius_max,
								  float32 turns)
	{
		const float32 alpha = 2.0f * float32(M_PI) / this->ny_;
		const float32 beta = turns / float32(this->ny_);

		const float32 radius = (radius_max + radius_min) / 2.0f;
		const float32 rdiff = (radius_max - radius_min) / 2.0f;

		for(uint32 i = 0; i <= this->ny_; ++i)
		{
			for(uint32 j = 0; j <= this->nx_; ++j)
			{
				const float32 rw = -rdiff + float32(j) * 2.0f * rdiff / float32(this->nx_);
				const float32 r = radius + rw * std::cos(beta * float32(i));

				attribute[this->vertex_table_[i * (this->nx_ + 1) + j]] =
						T(r * std::cos(alpha * float32(i)),
						r * std::sin(alpha * float32(i)),
						rw * std::sin(beta * float32(i)));
			}
		}
	}

	//! Embed a topological grid into an helicoid
	/*! @param[in] attribute Attribute used to store vertices positions
	 *  @param[in] radius_min
	 *  @param[in] radius_max
	 *  @param[in] maxHeight height to reach
	 *  @param[in] turns number of turn
	 *  @param[in] orient
	 */
	template <typename T>
	void embed_into_helicoid(typename MAP::template VertexAttribute<T>& attribute,
							 float32 radius_min,
							 float32 radius_max,
							 float32 maxHeight,
							 float32 nbTurn,
							 int32 orient)
	{
		const float32 alpha = float32(2.0 * M_PI / this->nx_) * nbTurn;
		const float32 hS = maxHeight / float32(this->nx_);

		float32 r,x,y;
		for(uint32 i = 0; i <= this->ny_; ++i)
		{
			for(uint32 j = 0; j <= this->nx_; ++j)
			{
				r = radius_min + (radius_max - radius_min) * float32(i) / float32(this->ny_);
				x = orient * r * std::sin(alpha * float32(j));
				y = orient * r * std::cos(alpha * float32(j));

				attribute[this->vertex_table_[i * (this->nx_ + 1) + j]] = T(x, y, j * hS);
			}
		}
	}
	//@}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_TILING_TRIANGULAR_GRID_CPP_))
extern template class CGOGN_MODELING_API TriangularGrid<CMap2>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_TILING_TRIANGULAR_GRID_CPP_))

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_TILING_TRIANGULAR_GRID_H_

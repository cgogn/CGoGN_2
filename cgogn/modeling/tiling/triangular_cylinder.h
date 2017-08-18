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

#ifndef CGOGN_MODELING_TILING_TRIANGULAR_CYLINDER_H_
#define CGOGN_MODELING_TILING_TRIANGULAR_CYLINDER_H_

#include <cgogn/core/cmap/cmap2_builder.h>
#include <cgogn/modeling/tiling/triangular_grid.h>
#include <cgogn/modeling/algos/refinements.h>

namespace cgogn
{

namespace modeling
{

template <typename MAP>
class TriangularCylinder : public Tiling<MAP>
{
	using CDart = typename MAP::CDart;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;
	using Volume = typename MAP::Volume;

protected:

	bool top_closed_, bottom_closed_;
	bool top_triangulated_, bottom_triangulated_;
	Vertex top_, bottom_;

public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(TriangularCylinder);

	//! Create a subdivided 2D cylinder
	/*! @param[in] n nb of squares around circumference
	 *  @param[in] z nb of squares in height
	 */
	template <typename INNERMAP>
	class CylinderTopo : public TriangularGrid<INNERMAP>::template GridTopo<INNERMAP>
	{
	public:

		CylinderTopo(Tiling<INNERMAP>* c, uint32 n, uint32 z) :
			TriangularGrid<INNERMAP>::template GridTopo<INNERMAP>(c, n, z)
		{
			using MapBuilder = typename MAP::Builder;
			MapBuilder mbuild(c->map_);

			// just finish to sew
			const uint32 nb_x = (n+1);
			for (uint32 i = 0; i < z; ++i)
			{
				const int32 pos = i*nb_x;
				Dart d = c->vertex_table_[pos].dart;
				d = c->map_.phi_1(d);
				Dart e = c->vertex_table_[pos + (nb_x-1)].dart;
				mbuild.phi2_sew(d, e);
				c->vertex_table_[pos + (nb_x-1)] = Vertex();
			}

			//suppress the last n vertex (in y direction) from the vertex_table_
			c->vertex_table_.erase(
				std::remove_if(
					c->vertex_table_.begin(),
					c->vertex_table_.end(),
					[&] (Vertex v) -> bool { return !v.is_valid(); }
				),
				c->vertex_table_.end()
			);

			c->vertex_table_.shrink_to_fit();
		}
	};

	TriangularCylinder(MAP& map, uint32 n, uint32 z, bool top_closed, bool bottom_closed) :
		Tiling<MAP>(map),
		top_closed_(top_closed),
		bottom_closed_(bottom_closed),
		top_triangulated_(false),
		bottom_triangulated_(false)
	{
		this->nx_ = n;
		this->ny_ = z;
		this->nz_ = UINT32_MAX;

		CylinderTopo<MAP>(this, n, z);

		using MapBuilder = typename MAP::Builder;
		MapBuilder mbuild(this->map_);

		this->dart_ = this->vertex_table_[0].dart;

		// close top
		Dart f = mbuild.close_hole_topo(this->map_.phi_1(this->map_.phi2(this->map_.phi1(this->vertex_table_[(this->nx_)*(this->ny_) - 1].dart))));
		mbuild.boundary_mark(Face(f));

		//add last row of edges to table
		f = this->map_.phi2(this->map_.phi_1(this->map_.phi2(this->map_.phi1(this->edge_table_[this->edge_table_.size()-this->nx_].dart))));
		Dart it = f;
		do
		{
			this->edge_table_.push_back(Edge(it));
			it = this->map_.phi1(it);
		}while(it != f);

		// close bottom
		Dart f2 = mbuild.close_hole_topo(this->dart_);
		mbuild.boundary_mark(Face(f2));

		//embed the cells

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

		if (top_closed_)
			close_top();

		if (bottom_closed_)
			close_bottom();
	}

	TriangularCylinder(MAP& map, unsigned int n, unsigned int z):
		TriangularCylinder<MAP>(map, n, z, true, true)
	{}

	/*! @name Embedding Operators
	 *************************************************************************/

	//@{
	//! Embed a topological cylinder
	/*! @param[in] position Attribute used to store vertices positions
	 *  @param[in] bottom_radius
	 *  @param[in] top_radius
	 *  @param[in] height
	 */
	template <typename T>
	void embed_into_cylinder(
		typename MAP::template VertexAttribute<T>& attribute,
		float32 bottom_radius,
		float32 top_radius,
		float32 height)
	{
		const float32 alpha = 2.0f * float32(M_PI) / float32(this->nx_);
		const float32 dz = height / float32(this->ny_);

		for (uint32 i = 0; i <= this->ny_; ++i)
		{
			const float32 a = float32(i) / float32(this->ny_);
			const float32 radius = a*top_radius + (1.0f - a)*bottom_radius;
			for (uint32 j = 0; j < this->nx_; ++j)
			{
				const float32 x = radius * std::cos(alpha * float32(j));
				const float32 y = radius * std::sin(alpha * float32(j));
				attribute[this->vertex_table_[i * (this->nx_) + j]] = T(x, y ,-height/2.0f + dz*float32(i));
			}
		}


		if (bottom_triangulated_)
			attribute[bottom_] = T(0.0f, 0.0f, -height/2 );

		if (top_triangulated_)
			attribute[top_] = T(0.0f, 0.0f, height/2 );
	}

	//! Embed a topological sphere
	/*! @param[in] position Attribute used to store vertices positions
	 *  @param[in] radius
	 *  @param[in] height
	 */
	template <typename T>
	void embed_into_sphere(
		typename MAP::template VertexAttribute<T>& attribute,
		float32 radius)
	{
		const float32 alpha = 2.0f * float32(M_PI) / float32(this->nx_);
		const float32 beta = float32(M_PI) / float32(this->ny_ + 2.0f);

		for (uint32 i = 0; i <= this->ny_; ++i)
		{
			for (uint32 j = 0; j < this->nx_; ++j)
			{
				const float32 h = radius * std::sin(-float32(M_PI_2) + float32(i + 1) * beta);
				const float32 rad = radius * std::cos(-float32(M_PI_2) + float32(i + 1) * beta);

				const float32 x = rad * std::cos(alpha * float32(j));
				const float32 y = rad * std::sin(alpha * float32(j));

				attribute[this->vertex_table_[i * (this->nx_) + j] ] = T(x, y, h);
			}
		}

		// bottom pole
		if (bottom_triangulated_)
			attribute[bottom_] = T(0.0f, 0.0f, -radius);

		// top pole
		if (top_triangulated_)
			attribute[top_] = T(0.0f, 0.0f, radius);
	}

	//! Embed a topological cone
	/*! @param[in] position Attribute used to store vertices positions
	 *  @param[in] radius
	 *  @param[in] height
	 */
	template <typename T>
	void embed_into_cone(
		typename MAP::template VertexAttribute<T>& attribute,
		float32 radius,
		float32 height)
	{
		if (top_closed_ && top_triangulated_)
		{
			const float32 alpha = 2.0f * float32(M_PI) / float32(this->nx_);
			const float32 dz = height / float32(this->ny_ + 1);
			for (uint32 i = 0; i <= this->ny_; ++i)
			{
				for (uint32 j = 0; j < this->nx_; ++j)
				{
					const float32 rad = radius * float32(this->ny_ + 1 - i) / float32(this->ny_ + 1);
					const float32 h = -height / 2.0f + dz * float32(i);
					const float32 x = rad * std::cos(alpha * float32(j));
					const float32 y = rad * std::sin(alpha * float32(j));

					attribute[this->vertex_table_[i*(this->nx_)+j] ] = T(x, y, h);
				}
			}

			if (bottom_triangulated_)
				attribute[bottom_] = T(0.0f, 0.0f, -height/2 );

			//  top always closed in cone
			attribute[top_] = T(0.0f, 0.0f, height/2 );
		}
	}

	//@}

	/*! @name Topological Operators
	 * Tiling creation
	 *************************************************************************/

	//@{
	//! Close the top with a n-sided face
	void close_top()
	{
		using MapBuilder = typename MAP::Builder;
		MapBuilder mbuild(this->map_);
		Face f(this->map_.phi2(this->map_.phi1(this->map_.phi2(this->vertex_table_[this->nx_ * this->ny_].dart)))) ;
		mbuild.boundary_unmark(f);
		top_closed_ = true;

		if (this->map_.template is_embedded<CDart>())
		{
			this->map_.foreach_dart_of_orbit(f, [&] (Dart d)
			{
				mbuild.new_orbit_embedding(CDart(d));
			});
		}

		if (this->map_.template is_embedded<Face>())
			mbuild.new_orbit_embedding(f);
	}

	//! Close the bottom with a n-sided face
	void close_bottom()
	{
		using MapBuilder = typename MAP::Builder;
		MapBuilder mbuild(this->map_);
		Face f(this->map_.phi2(this->vertex_table_[0].dart)) ;
		mbuild.boundary_unmark(f);
		bottom_closed_ = true;

		if (this->map_.template is_embedded<CDart>())
		{
			this->map_.foreach_dart_of_orbit(f, [&] (Dart d)
			{
				mbuild.new_orbit_embedding(CDart(d));
			});
		}

		if (this->map_.template is_embedded<Face>())
			mbuild.new_orbit_embedding(f);
	}

	//! Triangulate the top face with triangles fan
	void triangule_top()
	{
		if (top_closed_)
		{
			Dart d = this->map_.phi1(this->map_.phi2(this->vertex_table_[this->nx_*this->ny_].dart));
			Face f(this->map_.phi2(d));
			cgogn_assert(this->map_.codegree(f) > 3);

			cgogn::modeling::triangule(this->map_, f);
			top_ = Vertex(this->map_.phi_1(f.dart));
			top_triangulated_ = true;
		}
	}

	//! Triangulate the bottom face with triangles fan
	void triangule_bottom()
	{
		if (bottom_closed_)
		{
			Face f(this->map_.phi2(this->vertex_table_[0].dart));
			cgogn_assert(this->map_.codegree(f) > 3);

			cgogn::modeling::triangule(this->map_, f);
			bottom_ = Vertex(this->map_.phi_1(f.dart));
			bottom_triangulated_ = true;
		}
	}
	//@}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_TILING_TRIANGULAR_CYLINDER_CPP_))
extern template class CGOGN_MODELING_API TriangularCylinder<CMap2>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_MODELING_TILING_TRIANGULAR_CYLINDER_CPP_))

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_TILING_TRIANGULAR_CYLINDER_H_

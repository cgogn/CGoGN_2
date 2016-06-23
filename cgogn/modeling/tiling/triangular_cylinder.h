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
class TriangularCylinder : protected TriangularGrid<MAP>
{
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;

protected:
	bool top_closed_, bottom_closed_;
	bool top_triangulated_, bottom_triangulated_;
	Vertex top_, bottom_;

protected:
	//@{
	//! Create a subdivided 2D cylinder
	/*! @param[in] n nb of squares around circumference
	 *  @param[in] z nb of squares in height
	 */
	void cylinder(uint32 n, uint32 z)
	{
		this->nx_ = n;
		this->ny_ = z;
		this->nz_ = -1;

		this->grid(n,z);

		using MapBuilder = typename MAP::Builder;
		MapBuilder mbuild(this->map_);

		// just finish to sew
		const uint32 nb_x = (n+1);
		for(uint32 i = 0; i < z; ++i)
		{
			const int32 pos = i*nb_x;
			Dart d = this->vertex_table_[pos].dart;
			d = this->map_.phi_1(d);
			Dart e = this->vertex_table_[pos + z].dart;
			mbuild.phi2_sew(d, e);
			this->vertex_table_[pos + z] = Vertex();
		}

		//suppress the last n vertex (in y direction) from the vertex_table_
		this->vertex_table_.erase(
					std::remove_if(this->vertex_table_.begin(), this->vertex_table_.end(),
									[&](Vertex v) -> bool { return !v.is_valid(); }),
								this->vertex_table_.end());

		this->vertex_table_.shrink_to_fit();
	}
	//@}

	TriangularCylinder(MAP& map):
		TriangularGrid<MAP>(map)
	{}

public:
	TriangularCylinder(MAP& map, uint32 n, uint32 z, bool top_closed, bool bottom_closed):
		TriangularGrid<MAP>(map),
		top_closed_(top_closed),
		bottom_closed_(bottom_closed),
		top_triangulated_(false),
		bottom_triangulated_(false)
	{
		cylinder(n,z);

		using MapBuilder = typename MAP::Builder;
		MapBuilder mbuild(this->map_);

		//close top
		Face f = mbuild.close_hole(this->map_.phi_1(this->map_.phi2(this->map_.phi1(this->vertex_table_[(this->nx_)*(this->ny_) - 1].dart)))) ;
		this->map_.boundary_mark(f);

		//close bottom
		f = mbuild.close_hole(this->vertex_table_[0].dart) ;
		this->map_.boundary_mark(f);
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
	void embed_into_cylinder(typename MAP::template VertexAttribute<T>& attribute,
							 float32 bottom_radius,
							 float32 top_radius,
							 float32 height)
	{
		float32 alpha = 2.0 * M_PI/float32(this->nx_);
		float32 dz = height/float32(this->ny_);

		for(uint32 i = 0; i <= this->ny_; ++i)
		{
			float32 a = float32(i) / float32(this->ny_);
			float32 radius = a*top_radius + (1.0f-a)*bottom_radius;
			for(uint32 j = 0; j < this->nx_; ++j)
			{
				float32 x = radius * std::cos(alpha * float32(j));
				float32 y = radius * std::sin(alpha * float32(j));
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
	void embed_into_sphere(typename MAP::template VertexAttribute<T>& attribute,
						   float32 radius)
	{
		float32 alpha = 2.0 * M_PI / float32(this->nx_);
		float32 beta = M_PI / float32(this->ny_+2);

		for(uint32 i = 0; i <= this->ny_; ++i)
		{
			for(uint32 j = 0; j < this->nx_; ++j)
			{
				float32 h = radius * std::sin(-M_PI / 2.0 + float32(i+1) * beta);
				float32 rad = radius * std::cos(-M_PI / 2.0 + float32(i+1) * beta);

				float32 x = rad * std::cos(alpha * float32(j));
				float32 y = rad * std::sin(alpha * float32(j));

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
	void embed_into_cone(typename MAP::template VertexAttribute<T>& attribute,
						 float32 radius,
						 float32 height)
	{
		if(top_closed_ && top_triangulated_)
		{
			float32 alpha = 2.0 * M_PI / float32(this->nx_);
			float32 dz = height / float32(this->ny_ + 1);
			for(uint32 i = 0; i <= this->ny_; ++i)
			{
				for(uint32 j = 0; j < this->nx_; ++j)
				{
					float32 rad = radius * float32(this->ny_+1-i) / float32(this->ny_+1);
					float32 h = -height / 2.0 + dz * float32(i);
					float32 x = rad * std::cos(alpha * float32(j));
					float32 y = rad * std::sin(alpha * float32(j));

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
		Face f(this->map_.phi2(this->map_.phi1(this->map_.phi2(this->vertex_table_[this->nx_ * this->ny_].dart)))) ;
		this->map_.boundary_unmark(f);
		top_closed_ = true;
	}

	//! Close the bottom with a n-sided face
	void close_bottom()
	{
		Face f(this->map_.phi2(this->vertex_table_[0].dart)) ;
		this->map_.boundary_unmark(f);
		bottom_closed_ = true;
	}

	//! Triangulate the top face with triangles fan
	void triangule_top()
	{
		if(top_closed_)
		{
			Dart d = this->map_.phi1(this->map_.phi2(this->vertex_table_[this->nx_*this->ny_].dart));
			Face f(this->map_.phi2(d));
			cgogn_assert(this->map_.codegree(f) > 3);

			cgogn::modeling::triangule<MAP>(this->map_, f);
			top_ = Vertex(this->map_.phi_1(f.dart));
			top_triangulated_ = true;
		}
	}

	//! Triangulate the bottom face with triangles fan
	void triangule_bottom()
	{
		if(bottom_closed_)
		{
			Face f(this->map_.phi2(this->vertex_table_[0].dart));
			cgogn_assert(this->map_.codegree(f) > 3);

			cgogn::modeling::triangule<MAP>(this->map_, f);
			bottom_ = Vertex(this->map_.phi_1(f.dart));
			bottom_triangulated_ = true;
		}
	}
	//@}
};

} //namespace modeling

} //namespace cgogn

#endif // CGOGN_MODELING_TILING_TRIANGULAR_CYLINDER_H_
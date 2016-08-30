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

#ifndef CGOGN_MODELING_TILING_SQUARE_TORE_H_
#define CGOGN_MODELING_TILING_SQUARE_TORE_H_

#include <cgogn/core/cmap/cmap2_builder.h>
#include <cgogn/modeling/tiling/square_cylinder.h>

namespace cgogn
{

namespace modeling
{

/*! \brief The class of regular tore square tiling
 */
template <typename MAP>
class SquareTore : protected Tiling<MAP>
{
	using CDart = typename MAP::CDart;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;
	using Volume = typename MAP::Volume;

	/*! @name Topological Operators
	 *************************************************************************/
	//@{
public:
	//! Create a subdivided 2D tore
	/*! @param[in] n nb of squares around big circumference
	 *  @param[in] m nb of squares around small circumference
	 */
	template <typename INNERMAP>
	class ToreTopo: public SquareCylinder<INNERMAP>::template CylinderTopo<INNERMAP>
	{
	public:
		ToreTopo(Tiling<INNERMAP>* c, uint32 n, uint32 m):
			SquareCylinder<INNERMAP>::template CylinderTopo<INNERMAP>(c,n,m)
		{
			using MapBuilder = typename MAP::Builder;
			MapBuilder mbuild(c->map_);

			// just finish to sew
			const uint32 nb_y = (m-1)*n;
			for(uint32 i = 0; i < n; ++i)
			{
				Dart d = c->vertex_table_[i].dart;
				Dart e = c->vertex_table_[i+nb_y].dart;
				e = c->map_.phi1(c->map_.phi1(e));
				mbuild.phi2_sew(d, e);
				c->vertex_table_[i+nb_y+n] = Vertex();
			}

			// remove the last row of n vertex (in x direction) that are no more necessary (sewed with n first)
			c->vertex_table_.erase(
						std::remove_if(c->vertex_table_.begin(), c->vertex_table_.end(),
										[&](Vertex v) -> bool { return !v.is_valid(); }),
									c->vertex_table_.end());

			c->vertex_table_.shrink_to_fit();
		}
	};
	//@}

public:
	SquareTore(MAP& map, uint32 n, uint32 m):
		Tiling<MAP>(map)
	{
		this->nx_ = n;
		this->ny_ = m;
		this->nz_ = UINT32_MAX;

		ToreTopo<MAP>(this, n, m);

		this->dart_ = this->vertex_table_[0].dart;

		using MapBuilder = typename MAP::Builder;
		MapBuilder mbuild(this->map_);

		/*
		if(this->map_.template is_embedded<CDart>())
			this->map_.foreach_dart_of_orbit(Volume(this->dart_), [&](CDart d)
			{
				mbuild.new_orbit_embedding(d);
			});
		*/

		//embed the vertices
		if(this->map_.template is_embedded<Vertex>())
			for(Vertex v : this->vertex_table_)
				mbuild.new_orbit_embedding(v);

		if(this->map_.template is_embedded<Edge>())
			this->map_.foreach_incident_edge(Volume(this->dart_), [&](Edge e)
			{
				mbuild.new_orbit_embedding(e);
			});

		if(this->map_.template is_embedded<Face>())
			this->map_.foreach_incident_face(Volume(this->dart_), [&](Face f)
			{
				mbuild.new_orbit_embedding(f);
			});

		if(this->map_.template is_embedded<Volume>())
			mbuild.new_orbit_embedding(Volume(this->dart_));
	}

	/*! @name Embedding Operators
	 *************************************************************************/

	//@{
	//! Embed a topological tore
	/*! @param[in] position Attribute used to store vertices positions
	 *  @param[in] big_radius
	 *  @param[in] small_radius
	 */
	template <typename T>
	void embed_into_tore(typename MAP::template VertexAttribute<T>& attribute,
					   float32 big_radius,
					   float32 small_radius)
	{
		const float32 alpha = 2.0f * float32(M_PI) / float32(this->nx_);
		const float32 beta = 2.0f * float32(M_PI) / float32(this->ny_);

		for (uint32 i = 0; i < this->ny_; ++i)
		{
			for(uint32 j = 0; j < this->nx_; ++j)
			{
				const float32 z = small_radius*std::sin(beta*float32(i));
				const float32 r = big_radius + small_radius*std::cos(beta*float32(i));
				const float32 x = r*std::cos(alpha*float32(j));
				const float32 y = r*std::sin(alpha*float32(j));
				attribute[this->vertex_table_[i*(this->nx_)+j] ] = T(x, y, z);
			}
		}
	}
	//@}
};

} //namespace modeling

} //namespace cgogn

#endif // CGOGN_MODELING_TILING_SQUARE_TORE_H_

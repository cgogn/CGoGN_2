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

#include <core/cmap/cmap2_builder.h>
#include <tiling/tiling.h>

#include <cmath>

#ifndef TILING_TRIANGULAR_H_
#define TILING_TRIANGULAR_H_

namespace cgogn
{

namespace tiling
{

template <typename MAP>
class Grid : public Tiling<MAP>
{

public:
    Grid(MAP& map, unsigned int x, unsigned int y, bool close):
		Tiling<MAP>(map, x, y, -1)
    {
		grid(x, y, close);
    }

    Grid(MAP& map, unsigned int x, unsigned int y):
		Tiling<MAP>(map, x, y, -1)
	{
		grid(x, y, true);
	}

    /*! @name Embedding Operators
     * Tiling creation
     *************************************************************************/

    //@{
    //! Embed a topological grid
    /*! @param attribute Attribute used to store vertices positions
     *  @param x size in X
     *  @param x size in Y
     *  @param y attribute in Z (centered on 0 by default)
     */
    template <typename T>
	void embed_into_grid(typename MAP::template VertexAttributeHandler<T>& attribute, float x, float y, float z)
	{
		float dx = x / static_cast<float>(this->nx_);
		float dy = y / static_cast<float>(this->ny_);

		for(unsigned int i = 0; i <= this->ny_; ++i)
	    {
	        for(unsigned int j = 0; j <= this->nx_; ++j)
			{
				typename MAP::Vertex v(this->vertex_table_[i * (this->nx_ + 1) + j]);
				attribute[v] = 
					T(dx * static_cast<float>(j) + dx * 0.5f * static_cast<float>(i), 
						dy * static_cast<float>(i) * std::sqrt(3.0f) / 2.0f, 
						z);
	        }
	    }	
	}

    //! Embed a topological grid into a twister open ribbon with turns=PI it is a Moebius strip, needs only to be closed (if model allow it)
    /*! @param attribute Attribute used to store vertices positions
     *  @param radius_min
     *  @param radius_max
     *  @param turns number of turn multiplied by 2*PI
     */
    template <typename T>
	void embed_into_twisted_strip(typename MAP::template VertexAttributeHandler<T>& attribute, float radius_min, float radius_max, float turns)
	{
		float alpha = static_cast<float>(2.0 * M_PI / this->ny_);
		float beta = turns / static_cast<float>(this->ny_);

		float radius = (radius_max + radius_min) / 2.0f;
		float rdiff = (radius_max - radius_min) / 2.0f;

		for(unsigned int i = 0; i <= this->ny_; ++i)
		{
			for(unsigned int j = 0; j <= this->nx_; ++j)
			{
				float rw = -rdiff + static_cast<float>(j) * 2.0f * rdiff / static_cast<float>(this->nx_);
				float r = radius + rw * cos(beta * static_cast<float>(i));
				T pos(r * cos(alpha * static_cast<float>(i)), 
					r * sin(alpha * static_cast<float>(i)), 
					rw * sin(beta * static_cast<float>(i)));

				typename MAP::Vertex v(this->vertex_table_[i * (this->nx_ + 1) + j]);
				attribute[v] = pos;
			}
		}
	}

    //! Embed a topological grid into a helicoid
    /*! @param attribute Attribute used to store vertices positions
     *  @param radius_min
     *  @param radius_max
     *  @param maxHeight height to reach
     *  @param turns number of turn
     */
    template <typename T>
	void embed_into_helicoid(typename MAP::template VertexAttributeHandler<T>& attribute, float radius_min,  float radius_max, float maxHeight, float nbTurn, int orient)
	{
		float alpha = static_cast<float>(2.0 * M_PI / this->nx_) * nbTurn;
		float hS = maxHeight / static_cast<float>(this->nx_);

		// 	float radius = (radius_max + radius_min)/2.0f;
		// 	float rdiff = (radius_max - radius_min)/2.0f;

		for(unsigned int i = 0; i <= this->ny_; ++i)
		{
			for(unsigned int j = 0; j <= this->nx_; ++j)
			{
				// 			float r = radius_max + radius_min*cos(beta*float(j));
				float r,x,y;
				// 			if(i==1) {
				// 				r = radius_max;
				// 			}
				// 			else {
				r = radius_min + (radius_max - radius_min) * static_cast<float>(i) / static_cast<float>(this->ny_);
				// 			}
				x = orient * r * sin(alpha * static_cast<float>(j));
				y = orient * r * cos(alpha * static_cast<float>(j));

				T pos(x, y, j * hS);
				typename MAP::Vertex v(this->vertex_table_[i * (this->nx_ + 1) + j]);
				attribute[v] = pos;
			}
		}
	}
    //@}

protected:
    /*! @name Topological Operators
     * Tiling creation
     *************************************************************************/

    //@{
    //! Create a 2D grid
    /*! @param x nb of squares in x
     *  @param y nb of squares in y
     *  @param closed close the boundary face of the 2D grid
     */
    void grid(unsigned int x, unsigned int y, bool close)
    {
		using MapBuilder = cgogn::CMap2Builder_T<typename MAP::MapTraits>;

		MapBuilder mbuild(this->map_);

       // nb vertices
		unsigned int nb_vertices = (x + 1) * (y + 1);
		unsigned int nb_faces = 2 * x * y;

		// vertices reservation
		this->vertex_table_.reserve(nb_vertices);
		this->face_table_.reserve(nb_faces);

	    // creation of triangles and storing vertices
	    for (unsigned int i = 0; i < y; ++i)
	    {
	        for (unsigned int j = 1; j <= x; ++j)
	        {
	            Dart d = mbuild.add_face_topo_parent(3);
				Dart d2 = mbuild.add_face_topo_parent(3);
				mbuild.phi2_sew(this->map_.phi1(d), this->map_.phi_1(d2));

				this->vertex_table_.push_back(d);

				this->face_table_.push_back(d);
				this->face_table_.push_back(d2);

	            if (j == x)
					this->vertex_table_.push_back(d2);
	        }
	    }

	    // store last row of vertices
	    for (unsigned int i = 0; i < x; ++i)
	    {
			this->vertex_table_.push_back(this->map_.phi_1(this->vertex_table_[(y - 1) * (x + 1) + i]));
		}
		this->vertex_table_.push_back(this->map_.phi1(this->vertex_table_[(y - 1) * (x + 1) + x]));

		//sewing pairs of triangles
	    for (unsigned int i = 0; i < y; ++i)
	    {
	        for (unsigned int j = 0; j < x; ++j)
	        {

				if (i > 0) // sew with preceeding row
				{
					int pos = i * (x + 1) + j;
					Dart d = this->vertex_table_[pos];
					Dart e = this->vertex_table_[pos - (x + 1)];
					e = this->map_.phi_1(this->map_.phi2(this->map_.phi1(e)));
					mbuild.phi2_sew(d, e);
				}
				if (j > 0) // sew with preceeding column
				{
					int pos = i * (x + 1) + j;
					Dart d = this->vertex_table_[pos];
					d = this->map_.phi_1(d);
					Dart e = this->vertex_table_[pos - 1];
					e = this->map_.phi1(this->map_.phi2(this->map_.phi1(e)));
					mbuild.phi2_sew(d, e);
				}
	        }
	    }

	    if(close)
	        mbuild.close_hole(this->vertex_table_[0]) ;

		this->dart_ = this->vertex_table_[0];
    }
    //@}
};

} // namespace tiling

} // namespace cgogn

#endif // TILING_TRIANGULAR_H_
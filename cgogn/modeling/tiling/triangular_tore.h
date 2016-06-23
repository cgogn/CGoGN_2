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

#ifndef CGOGN_MODELING_TILING_TRIANGULAR_TORE_H_
#define CGOGN_MODELING_TILING_TRIANGULAR_TORE_H_

#include <cgogn/core/cmap/cmap2_builder.h>
#include <cgogn/modeling/tiling/triangular_cylinder.h>

namespace cgogn
{

namespace modeling
{

/*! \brief The class of regular tore square tiling
 */
template <typename MAP>
class TriangularTore : public TriangularTiling<MAP>
{
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;

	/*! @name Topological Operators
	 *************************************************************************/
protected:
	TriangularTore(MAP& map):
		TriangularTiling<MAP>(map)
	{}

public:
	TriangularTore(MAP& map, uint32 n, uint32 m):
		TriangularTiling<MAP>(map)
	{
		this->tore(n,m);
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
		float32 alpha = 2.0*M_PI/float32(this->nx_);
		float32 beta = 2.0*M_PI/float32(this->ny_);

		for (uint32 i = 0; i < this->ny_; ++i)
		{
			for(uint32 j = 0; j < this->nx_; ++j)
			{
				float32 z = small_radius*std::sin(beta*float32(i));
				float32 r = big_radius + small_radius*std::cos(beta*float32(i));
				float32 x = r*std::cos(alpha*float32(j));
				float32 y = r*std::sin(alpha*float32(j));
				attribute[this->vertex_table_[i*(this->nx_)+j] ] = T(x, y, z);
			}
		}
	}
	//@}
};

} //namespace modeling

} //namespace cgogn

#endif // CGOGN_MODELING_TILING_TRIANGULAR_TORE_H_

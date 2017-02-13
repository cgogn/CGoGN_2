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

#ifndef CGOGN_MODELING_DECIMATION_EDGE_APPROXIMATOR_MID_EDGE_H_
#define CGOGN_MODELING_DECIMATION_EDGE_APPROXIMATOR_MID_EDGE_H_

#include <cgogn/modeling/decimation/edge_approximator.h>

namespace cgogn
{

namespace modeling
{

template <typename MAP, typename VEC3>
class EdgeApproximator_MidEdge : public EdgeApproximator<MAP, VEC3>
{
public:

	using Inherit = EdgeApproximator<MAP, VEC3>;
	using Self = EdgeApproximator_MidEdge<MAP, VEC3>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(EdgeApproximator_MidEdge);

	inline EdgeApproximator_MidEdge(
		const MAP& m,
		const typename MAP::template VertexAttribute<VEC3>& position
	) : Inherit(m),
		position_(position)
	{}

	virtual ~EdgeApproximator_MidEdge()
	{}

	void init()
	{}

	VEC3 operator()(Edge e) const
	{
		std::pair<Vertex,Vertex> vertices = this->map_.vertices(e);
		return Scalar(0.5) * (position_[vertices.first] + position_[vertices.second]);
	}

private:

	const typename MAP::template VertexAttribute<VEC3>& position_;
};

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_DECIMATION_EDGE_APPROXIMATOR_MID_EDGE_H_

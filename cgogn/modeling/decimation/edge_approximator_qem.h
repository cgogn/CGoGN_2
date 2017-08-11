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

#ifndef CGOGN_MODELING_DECIMATION_EDGE_APPROXIMATOR_QEM_H_
#define CGOGN_MODELING_DECIMATION_EDGE_APPROXIMATOR_QEM_H_

#include <cgogn/modeling/decimation/edge_approximator.h>
#include <cgogn/geometry/types/quadric.h>

namespace cgogn
{

namespace modeling
{

template <typename MAP, typename VEC3>
class EdgeApproximator_QEM : public EdgeApproximator<MAP, VEC3>
{
public:

	using Inherit = EdgeApproximator<MAP, VEC3>;
	using Self = EdgeApproximator_QEM<MAP, VEC3>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Face = typename MAP::Face;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(EdgeApproximator_QEM);

	inline EdgeApproximator_QEM(
		const MAP& m,
		const typename MAP::template VertexAttribute<VEC3>& position
	) : Inherit(m),
		position_(position)
	{}

	virtual ~EdgeApproximator_QEM()
	{}

	void init()
	{
		// attribute is valid only if the used EdgeTraversor is a EdgeTraversor_QEM
		quadric_ = this->map_.template get_attribute<geometry::Quadric, Vertex>("EdgeTraversor_QEM_Quadric");
	}

	VEC3 operator()(Edge e) const
	{
		geometry::Quadric q;
		std::pair<Vertex,Vertex> vertices = this->map_.vertices(e);

		if (quadric_.is_valid())
		{
			q += quadric_[vertices.first];
			q += quadric_[vertices.second];
		}
		else
		{
			geometry::Quadric q1;
			this->map_.foreach_incident_face(vertices.first, [&] (Face f)
			{
				Dart d = f.dart;
				Dart d1 = this->map_.phi1(d);
				Dart d_1 = this->map_.phi_1(d);
				q1 += geometry::Quadric(position_[Vertex(d)], position_[Vertex(d1)], position_[Vertex(d_1)]);
			});

			geometry::Quadric q2;
			this->map_.foreach_incident_face(vertices.second, [&] (Face f)
			{
				Dart d = f.dart;
				Dart d1 = this->map_.phi1(d);
				Dart d_1 = this->map_.phi_1(d);
				q2 += geometry::Quadric(position_[Vertex(d)], position_[Vertex(d1)], position_[Vertex(d_1)]);
			});

			q += q1;
			q += q2;
		}

		VEC3 res;
		bool opt = q.optimized(res);
		if (!opt)
			res = Scalar(0.5) * (position_[vertices.first] + position_[vertices.second]);

		return res;
	}

private:

	const typename MAP::template VertexAttribute<VEC3>& position_;
	typename MAP::template VertexAttribute<geometry::Quadric> quadric_;
};

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_DECIMATION_EDGE_APPROXIMATOR_QEM_H_

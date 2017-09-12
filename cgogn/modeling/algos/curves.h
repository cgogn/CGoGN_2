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

#ifndef CGOGN_MODELING_ALGOS_CURVES_H_
#define CGOGN_MODELING_ALGOS_CURVES_H_

#include <cgogn/modeling/dll.h>
#include <cgogn/core/graph/undirected_graph.h>

namespace cgogn
{

namespace modeling
{

/// \brief
/// \param[in] map
/// \param[in] position
/// \param[in] nb_samples number of points of the curve
/// \param[in] tmin minimum range of parameter t (must be a multiple of PI)
/// \param[in] tmax maximum range of parameter t (must be a multiple of PI)
template <typename VEC3, typename FUNC>
UndirectedGraph::Vertex generate_curve(UndirectedGraph& map,
					UndirectedGraph::VertexAttribute<VEC3>& position,
                    uint32 nb_samples,
                    float32 tmin,
                    float32 tmax,
                    const FUNC& f)
{
	using Vertex = UndirectedGraph::Vertex;
    const float32 dt = (tmax - tmin) / float32(nb_samples);
    float32 it = tmin;

    // uniform sampling of [tmin, tmax] with nb_samples
    Vertex vit = map.make_polyline(nb_samples);
	Vertex vr = vit;

    for(uint32 i = 0 ; i < nb_samples ; ++i)
	{
		VEC3 p = f(it);
		position[vit] = p;
        vit = Vertex(map.alpha1(map.alpha0(vit.dart)));
		it += dt;
    }

	return vr;
}

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_MODELING_ALGOS_CURVES_H_

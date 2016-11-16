/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* version 0.1                                                                  *
* Copyright (C) 2009-2012, IGG Team, LSIIT, University of Strasbourg           *
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

#include <cgogn/core/utils/numerics.h>
#include <OpenNL_psm.h>
#include <cgogn/geometry/types/geometry_traits.h>


namespace cgogn {


///
/// Variables setup
///
template <typename T, typename MAP>
void setup_variables(
        const MAP& map,
        const typename MAP::template VertexAttribute<uint32>& index,
		const typename MAP::DartMarker& locked_marker,
        const typename MAP::template VertexAttribute<T>& attr)
{
    using Vertex = typename MAP::Vertex;

    map.foreach_cell([&](Vertex v)
    {
        nlSetVariable(index[v], attr[v]);
		if(locked_marker.is_marked(v.dart))
            nlLockVariable(index[v]);
    });
}

template <typename T, typename MAP>
void setup_variables(
        MAP& map,
        const typename MAP::template VertexAttribute<uint32>& index,
        const typename MAP::template CellMarker<MAP::Vertex::ORBIT>& free_marker,
        const typename MAP::template VertexAttribute<T>& attr,
        unsigned int coord)
{
    using Vertex = typename MAP::Vertex;

    map.foreach_cell([&](Vertex v)
    {
        nlSetVariable(index[v], attr[v][coord]);
		if(!free_marker.is_marked(v))
            nlLockVariable(index[v]);
    });
}

///
/// Matrix setup : laplacian topo
///
template <typename Scalar, typename MAP>
void add_rows_laplacian_topo(
        const MAP& map,
        const typename MAP::template VertexAttribute<uint32>& index)
{
    using Vertex = typename MAP::Vertex;

    nlEnable(NL_NORMALIZE_ROWS) ;

    map.foreach_cell([&] (Vertex v)
    {
        nlRightHandSide(0) ;
        nlBegin(NL_ROW);
		Scalar aii(0) ;
        map.foreach_adjacent_vertex_through_edge(v, [&](Vertex vaij)
        {
			Scalar aij(1);
			aii += aij;
			nlCoefficient(index[vaij], aij);
        });
        nlCoefficient(index[v], -aii) ;
        nlEnd(NL_ROW) ;
    });

    nlDisable(NL_NORMALIZE_ROWS) ;
}

///
/// Get results
///
template <typename T, typename MAP>
void result(
        const MAP& map,
        const typename MAP::template VertexAttribute<uint32>& index,
		typename MAP::template VertexAttribute<T>& attr)
{
    using Vertex = typename MAP::Vertex;

    map.foreach_cell([&] (Vertex v)
    {
        attr[v] = T(nlGetVariable(index[v]));
    });
}

template <typename VEC, typename MAP>
void result(
        const MAP& map,
        const typename MAP::template VertexAttribute<uint32>& index,
		typename MAP::template VertexAttribute<VEC>& attr,
        uint32 coord)
{
    using Vertex = typename MAP::Vertex;
	using Scalar = typename cgogn::geometry::vector_traits<VEC>::Scalar;

    map.foreach_cell([&](Vertex v)
    {
        (attr[v])[coord] = Scalar(nlGetVariable(index[v]));
    });
}

} //end namespace cgogn

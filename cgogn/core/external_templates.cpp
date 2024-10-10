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
#define CGOGN_CORE_EXTERNAL_TEMPLATES_CPP_

#include <cgogn/core/container/chunk_stack.h>

namespace cgogn
{

template class CGOGN_CORE_EXPORT ChunkArrayGen<CGOGN_CHUNK_SIZE>;
template class CGOGN_CORE_EXPORT ChunkArray<CGOGN_CHUNK_SIZE, uint32>;
template class CGOGN_CORE_EXPORT ChunkArray<CGOGN_CHUNK_SIZE, uint8>;
template class CGOGN_CORE_EXPORT ChunkArray<CGOGN_CHUNK_SIZE, std::array<float32, 3>>;
template class CGOGN_CORE_EXPORT ChunkArray<CGOGN_CHUNK_SIZE, std::array<float64, 3>>;
template class CGOGN_CORE_EXPORT ChunkArrayBool<CGOGN_CHUNK_SIZE>;
template class CGOGN_CORE_EXPORT ChunkStack<CGOGN_CHUNK_SIZE, uint32>;

} // namespace cgogn


#include <cgogn/core/container/chunk_array_container.h>

namespace cgogn
{

template class CGOGN_CORE_EXPORT ChunkArrayContainer<CGOGN_CHUNK_SIZE, uint32>;
template class CGOGN_CORE_EXPORT ChunkArrayContainer<CGOGN_CHUNK_SIZE, uint8>;
template class CGOGN_CORE_EXPORT ChunkArrayFactory<CGOGN_CHUNK_SIZE>;
template CGOGN_CORE_EXPORT ChunkArrayFactory<CGOGN_CHUNK_SIZE>& chunk_array_factory<CGOGN_CHUNK_SIZE>();

} // namespace cgogn




#include <cgogn/core/cmap/cmap0.h>

namespace cgogn
{

template class CGOGN_CORE_EXPORT CMap0_T<CMap0Type>;
template class CGOGN_CORE_EXPORT DartMarker<CMap0>;
template class CGOGN_CORE_EXPORT DartMarkerStore<CMap0>;
template class CGOGN_CORE_EXPORT DartMarkerNoUnmark<CMap0>;
template class CGOGN_CORE_EXPORT CellMarker<CMap0, CMap0::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap0, CMap0::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap0, CMap0::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT QuickTraversor<CMap0>;

} // namespace cgogn


#include <cgogn/core/cmap/cmap1.h>

namespace cgogn
{

template class CGOGN_CORE_EXPORT CMap1_T<CMap1Type>;
template class CGOGN_CORE_EXPORT DartMarker<CMap1>;
template class CGOGN_CORE_EXPORT DartMarkerStore<CMap1>;
template class CGOGN_CORE_EXPORT DartMarkerNoUnmark<CMap1>;
template class CGOGN_CORE_EXPORT CellMarker<CMap1, CMap1::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap1, CMap1::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap1, CMap1::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap1, CMap1::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap1, CMap1::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap1, CMap1::Face::ORBIT>;
template class CGOGN_CORE_EXPORT QuickTraversor<CMap1>;
} // namespace cgogn

#include <cgogn/core/cmap/cmap2.h>

namespace cgogn
{

template class CGOGN_CORE_EXPORT CMap2_T<CMap2Type>;
template class CGOGN_CORE_EXPORT DartMarker<CMap2>;
template class CGOGN_CORE_EXPORT DartMarkerStore<CMap2>;
template class CGOGN_CORE_EXPORT DartMarkerNoUnmark<CMap2>;
template class CGOGN_CORE_EXPORT CellMarker<CMap2, CMap2::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap2, CMap2::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap2, CMap2::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap2, CMap2::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap2, CMap2::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap2, CMap2::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap2, CMap2::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap2, CMap2::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap2, CMap2::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap2, CMap2::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap2, CMap2::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap2, CMap2::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellCache<CMap2>;
template class CGOGN_CORE_EXPORT BoundaryCache<CMap2>;
template class CGOGN_CORE_EXPORT QuickTraversor<CMap2>;

} // namespace cgogn

#include <cgogn/core/cmap/cmap2_quad.h>

namespace cgogn
{

template class CGOGN_CORE_EXPORT DartMarker<CMap2Quad>;
template class CGOGN_CORE_EXPORT DartMarkerStore<CMap2Quad>;
template class CGOGN_CORE_EXPORT DartMarkerNoUnmark<CMap2Quad>;
template class CGOGN_CORE_EXPORT CellMarker<CMap2Quad, CMap2Quad::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap2Quad, CMap2Quad::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap2Quad, CMap2Quad::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap2Quad, CMap2Quad::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap2Quad, CMap2Quad::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap2Quad, CMap2Quad::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap2Quad, CMap2Quad::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap2Quad, CMap2Quad::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap2Quad, CMap2Quad::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap2Quad, CMap2Quad::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap2Quad, CMap2Quad::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap2Quad, CMap2Quad::Volume::ORBIT>;


} // namespace cgogn

#include <cgogn/core/cmap/cmap2_tri.h>
namespace cgogn
{

template class CGOGN_CORE_EXPORT DartMarker<CMap2Tri>;
template class CGOGN_CORE_EXPORT DartMarkerStore<CMap2Tri>;
template class CGOGN_CORE_EXPORT DartMarkerNoUnmark<CMap2Tri>;
template class CGOGN_CORE_EXPORT CellMarker<CMap2Tri, CMap2Tri::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap2Tri, CMap2Tri::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap2Tri, CMap2Tri::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap2Tri, CMap2Tri::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap2Tri, CMap2Tri::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap2Tri, CMap2Tri::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap2Tri, CMap2Tri::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap2Tri, CMap2Tri::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap2Tri, CMap2Tri::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap2Tri, CMap2Tri::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap2Tri, CMap2Tri::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap2Tri, CMap2Tri::Volume::ORBIT>;


} // namespace cgogn

#include <cgogn/core/cmap/cmap3.h>

namespace cgogn
{

template class CGOGN_CORE_EXPORT CMap3_T<CMap3Type>;
template class CGOGN_CORE_EXPORT DartMarker<CMap3>;
template class CGOGN_CORE_EXPORT DartMarkerStore<CMap3>;
template class CGOGN_CORE_EXPORT DartMarkerNoUnmark<CMap3>;
template class CGOGN_CORE_EXPORT CellMarker<CMap3, CMap3::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap3, CMap3::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap3, CMap3::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap3, CMap3::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap3, CMap3::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap3, CMap3::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap3, CMap3::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap3, CMap3::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap3, CMap3::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap3, CMap3::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap3, CMap3::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap3, CMap3::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellCache<CMap3>;
template class CGOGN_CORE_EXPORT BoundaryCache<CMap3>;
template class CGOGN_CORE_EXPORT QuickTraversor<CMap3>;

} // namespace cgogn

#include <cgogn/core/cmap/cmap3_hexa.h>

namespace cgogn
{

template class CGOGN_CORE_EXPORT DartMarker<CMap3Hexa>;
template class CGOGN_CORE_EXPORT DartMarkerStore<CMap3Hexa>;
template class CGOGN_CORE_EXPORT DartMarkerNoUnmark<CMap3Hexa>;
template class CGOGN_CORE_EXPORT CellMarker<CMap3Hexa, CMap3Hexa::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap3Hexa, CMap3Hexa::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap3Hexa, CMap3Hexa::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap3Hexa, CMap3Hexa::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap3Hexa, CMap3Hexa::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap3Hexa, CMap3Hexa::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap3Hexa, CMap3Hexa::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap3Hexa, CMap3Hexa::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap3Hexa, CMap3Hexa::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap3Hexa, CMap3Hexa::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap3Hexa, CMap3Hexa::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap3Hexa, CMap3Hexa::Volume::ORBIT>;

} // namespace cgogn

#include <cgogn/core/cmap/cmap3_tetra.h>

namespace cgogn
{

template class CGOGN_CORE_EXPORT DartMarker<CMap3Tetra>;
template class CGOGN_CORE_EXPORT DartMarkerStore<CMap3Tetra>;
template class CGOGN_CORE_EXPORT DartMarkerNoUnmark<CMap3Tetra>;
template class CGOGN_CORE_EXPORT CellMarker<CMap3Tetra, CMap3Tetra::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap3Tetra, CMap3Tetra::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap3Tetra, CMap3Tetra::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<CMap3Tetra, CMap3Tetra::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap3Tetra, CMap3Tetra::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap3Tetra, CMap3Tetra::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap3Tetra, CMap3Tetra::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<CMap3Tetra, CMap3Tetra::Volume::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap3Tetra, CMap3Tetra::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap3Tetra, CMap3Tetra::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap3Tetra, CMap3Tetra::Face::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<CMap3Tetra, CMap3Tetra::Volume::ORBIT>;

} // namespace cgogn

#include <cgogn/core/graph/undirected_graph.h>

namespace cgogn
{

template class CGOGN_CORE_EXPORT UndirectedGraph_T<UndirectedGraphType>;
template class CGOGN_CORE_EXPORT DartMarker<UndirectedGraph>;
template class CGOGN_CORE_EXPORT DartMarkerStore<UndirectedGraph>;
template class CGOGN_CORE_EXPORT DartMarkerNoUnmark<UndirectedGraph>;
template class CGOGN_CORE_EXPORT CellMarker<UndirectedGraph, UndirectedGraph::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarker<UndirectedGraph, UndirectedGraph::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<UndirectedGraph, UndirectedGraph::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerNoUnmark<UndirectedGraph, UndirectedGraph::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<UndirectedGraph, UndirectedGraph::Vertex::ORBIT>;
template class CGOGN_CORE_EXPORT CellMarkerStore<UndirectedGraph, UndirectedGraph::Edge::ORBIT>;
template class CGOGN_CORE_EXPORT CellCache<UndirectedGraph>;
template class CGOGN_CORE_EXPORT BoundaryCache<UndirectedGraph>;
template class CGOGN_CORE_EXPORT QuickTraversor<UndirectedGraph>;

} // namespace cgogn


#include <cgogn/core/cmap/cmap0_builder.h>
#include <cgogn/core/cmap/cmap1_builder.h>
#include <cgogn/core/cmap/cmap2_builder.h>
#include <cgogn/core/cmap/cmap3_builder.h>
#include <cgogn/core/graph/undirected_graph_builder.h>

namespace cgogn
{

template class CGOGN_CORE_EXPORT CMap0Builder_T<CMap0>;
template class CGOGN_CORE_EXPORT CMap1Builder_T<CMap1>;
template class CGOGN_CORE_EXPORT CMap2Builder_T<CMap2>;
template class CGOGN_CORE_EXPORT CMap2Builder_T<CMap2Tri>;
template class CGOGN_CORE_EXPORT CMap2Builder_T<CMap2Quad>;
template class CGOGN_CORE_EXPORT CMap3Builder_T<CMap3>;
//template class CGOGN_CORE_EXPORT CMap3Builder_T<CMap3Hexa>; // TODO : fix compilation
//template class CGOGN_CORE_EXPORT CMap3Builder_T<CMap3Tetra>;// TODO : fix compilation
template class CGOGN_CORE_EXPORT UndirectedGraphBuilder_T<UndirectedGraph>;
} // namespace cgogn


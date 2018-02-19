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

#include <cgogn/core/container/chunk_array_container.h>
#include <cgogn/core/container/chunk_array_factory.h>

#include <cgogn/core/cmap/cmap2_quad.h>
#include <cgogn/core/cmap/cmap2_tri.h>
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/cmap/cmap3_hexa.h>
#include <cgogn/core/cmap/cmap3_tetra.h>

#include <cgogn/core/graph/undirected_graph.h>

namespace cgogn
{
/// CONTAINER
template class CGOGN_CORE_API ChunkArrayGen<CGOGN_CHUNK_SIZE>;

template class CGOGN_CORE_API ChunkArray<CGOGN_CHUNK_SIZE, uint32>;
template class CGOGN_CORE_API ChunkArray<CGOGN_CHUNK_SIZE, uint8>;
template class CGOGN_CORE_API ChunkArray<CGOGN_CHUNK_SIZE, std::array<float32, 3>>;
template class CGOGN_CORE_API ChunkArray<CGOGN_CHUNK_SIZE, std::array<float64, 3>>;
template class CGOGN_CORE_API ChunkArrayBool<CGOGN_CHUNK_SIZE>;

template class CGOGN_CORE_API ChunkArrayContainer<CGOGN_CHUNK_SIZE, uint32>;
template class CGOGN_CORE_API ChunkArrayContainer<CGOGN_CHUNK_SIZE, uint8>;

template class CGOGN_CORE_API ChunkArrayFactory<CGOGN_CHUNK_SIZE>;
template CGOGN_CORE_API ChunkArrayFactory<CGOGN_CHUNK_SIZE>& chunk_array_factory<CGOGN_CHUNK_SIZE>();

template class CGOGN_CORE_API ChunkStack<CGOGN_CHUNK_SIZE, uint32>;

/// CMAP0
template class CGOGN_CORE_API CMap0_T<CMap0Type>;
template class CGOGN_CORE_API DartMarker<CMap0>;
template class CGOGN_CORE_API DartMarkerStore<CMap0>;
template class CGOGN_CORE_API DartMarkerNoUnmark<CMap0>;
template class CGOGN_CORE_API CellMarker<CMap0, CMap0::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap0, CMap0::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap0, CMap0::Vertex::ORBIT>;
template class CGOGN_CORE_API QuickTraversor<CMap0>;

/// CMAP1
template class CGOGN_CORE_API CMap1_T<CMap1Type>;
template class CGOGN_CORE_API DartMarker<CMap1>;
template class CGOGN_CORE_API DartMarkerStore<CMap1>;
template class CGOGN_CORE_API DartMarkerNoUnmark<CMap1>;
template class CGOGN_CORE_API CellMarker<CMap1, CMap1::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap1, CMap1::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap1, CMap1::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap1, CMap1::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap1, CMap1::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap1, CMap1::Face::ORBIT>;
template class CGOGN_CORE_API QuickTraversor<CMap1>;

/// CMAP2
template class CGOGN_CORE_API CMap2_T<CMap2Type>;
template class CGOGN_CORE_API CMap2Builder_T<CMap2>;
template class CGOGN_CORE_API DartMarker<CMap2>;
template class CGOGN_CORE_API DartMarkerStore<CMap2>;
template class CGOGN_CORE_API DartMarkerNoUnmark<CMap2>;
template class CGOGN_CORE_API CellMarker<CMap2, CMap2::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap2, CMap2::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap2, CMap2::Face::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap2, CMap2::Volume::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2, CMap2::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2, CMap2::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2, CMap2::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2, CMap2::Volume::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap2, CMap2::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap2, CMap2::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap2, CMap2::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap2, CMap2::Volume::ORBIT>;
template class CGOGN_CORE_API CellCache<CMap2>;
template class CGOGN_CORE_API BoundaryCache<CMap2>;
template class CGOGN_CORE_API QuickTraversor<CMap2>;

/// CMAP2_QUAD
template class CGOGN_CORE_API CMap2Builder_T<CMap2Quad>;
template class CGOGN_CORE_API DartMarker<CMap2Quad>;
template class CGOGN_CORE_API DartMarkerStore<CMap2Quad>;
template class CGOGN_CORE_API DartMarkerNoUnmark<CMap2Quad>;
template class CGOGN_CORE_API CellMarker<CMap2Quad, CMap2Quad::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap2Quad, CMap2Quad::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap2Quad, CMap2Quad::Face::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap2Quad, CMap2Quad::Volume::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Quad, CMap2Quad::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Quad, CMap2Quad::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Quad, CMap2Quad::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Quad, CMap2Quad::Volume::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap2Quad, CMap2Quad::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap2Quad, CMap2Quad::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap2Quad, CMap2Quad::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap2Quad, CMap2Quad::Volume::ORBIT>;

/// CMAP2_TRI
template class CGOGN_CORE_API CMap2Builder_T<CMap2Tri>;
template class CGOGN_CORE_API DartMarker<CMap2Tri>;
template class CGOGN_CORE_API DartMarkerStore<CMap2Tri>;
template class CGOGN_CORE_API DartMarkerNoUnmark<CMap2Tri>;
template class CGOGN_CORE_API CellMarker<CMap2Tri, CMap2Tri::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap2Tri, CMap2Tri::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap2Tri, CMap2Tri::Face::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap2Tri, CMap2Tri::Volume::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Tri, CMap2Tri::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Tri, CMap2Tri::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Tri, CMap2Tri::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap2Tri, CMap2Tri::Volume::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap2Tri, CMap2Tri::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap2Tri, CMap2Tri::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap2Tri, CMap2Tri::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap2Tri, CMap2Tri::Volume::ORBIT>;

/// CMAP3
template class CGOGN_CORE_API CMap3_T<CMap3Type>;
template class CGOGN_CORE_API CMap3Builder_T<CMap3>;
template class CGOGN_CORE_API DartMarker<CMap3>;
template class CGOGN_CORE_API DartMarkerStore<CMap3>;
template class CGOGN_CORE_API DartMarkerNoUnmark<CMap3>;
template class CGOGN_CORE_API CellMarker<CMap3, CMap3::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap3, CMap3::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap3, CMap3::Face::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap3, CMap3::Volume::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3, CMap3::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3, CMap3::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3, CMap3::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3, CMap3::Volume::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap3, CMap3::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap3, CMap3::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap3, CMap3::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap3, CMap3::Volume::ORBIT>;
template class CGOGN_CORE_API CellCache<CMap3>;
template class CGOGN_CORE_API BoundaryCache<CMap3>;
template class CGOGN_CORE_API QuickTraversor<CMap3>;

/// CMAP3_HEXA
template class CGOGN_CORE_API CMap3Builder_T<CMap3Hexa>;
template class CGOGN_CORE_API DartMarker<CMap3Hexa>;
template class CGOGN_CORE_API DartMarkerStore<CMap3Hexa>;
template class CGOGN_CORE_API DartMarkerNoUnmark<CMap3Hexa>;
template class CGOGN_CORE_API CellMarker<CMap3Hexa, CMap3Hexa::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap3Hexa, CMap3Hexa::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap3Hexa, CMap3Hexa::Face::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap3Hexa, CMap3Hexa::Volume::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3Hexa, CMap3Hexa::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3Hexa, CMap3Hexa::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3Hexa, CMap3Hexa::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3Hexa, CMap3Hexa::Volume::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap3Hexa, CMap3Hexa::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap3Hexa, CMap3Hexa::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap3Hexa, CMap3Hexa::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap3Hexa, CMap3Hexa::Volume::ORBIT>;

/// CMAP3_TETRA
template class CGOGN_CORE_API CMap3Builder_T<CMap3Tetra>;
template class CGOGN_CORE_API DartMarker<CMap3Tetra>;
template class CGOGN_CORE_API DartMarkerStore<CMap3Tetra>;
template class CGOGN_CORE_API DartMarkerNoUnmark<CMap3Tetra>;
template class CGOGN_CORE_API CellMarker<CMap3Tetra, CMap3Tetra::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap3Tetra, CMap3Tetra::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap3Tetra, CMap3Tetra::Face::ORBIT>;
template class CGOGN_CORE_API CellMarker<CMap3Tetra, CMap3Tetra::Volume::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3Tetra, CMap3Tetra::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3Tetra, CMap3Tetra::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3Tetra, CMap3Tetra::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<CMap3Tetra, CMap3Tetra::Volume::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap3Tetra, CMap3Tetra::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap3Tetra, CMap3Tetra::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap3Tetra, CMap3Tetra::Face::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<CMap3Tetra, CMap3Tetra::Volume::ORBIT>;

/// UNDIRECTED GRAPH
template class CGOGN_CORE_API UndirectedGraph_T<UndirectedGraphType>;
template class CGOGN_CORE_API UndirectedGraphBuilder_T<UndirectedGraph>;
template class CGOGN_CORE_API DartMarker<UndirectedGraph>;
template class CGOGN_CORE_API DartMarkerStore<UndirectedGraph>;
template class CGOGN_CORE_API DartMarkerNoUnmark<UndirectedGraph>;
template class CGOGN_CORE_API CellMarker<UndirectedGraph, UndirectedGraph::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarker<UndirectedGraph, UndirectedGraph::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<UndirectedGraph, UndirectedGraph::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerNoUnmark<UndirectedGraph, UndirectedGraph::Edge::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<UndirectedGraph, UndirectedGraph::Vertex::ORBIT>;
template class CGOGN_CORE_API CellMarkerStore<UndirectedGraph, UndirectedGraph::Edge::ORBIT>;
template class CGOGN_CORE_API CellCache<UndirectedGraph>;
template class CGOGN_CORE_API BoundaryCache<UndirectedGraph>;
template class CGOGN_CORE_API QuickTraversor<UndirectedGraph>;

} // namespace cgogn


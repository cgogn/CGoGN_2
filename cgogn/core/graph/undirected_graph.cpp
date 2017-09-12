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

#define CGOGN_CORE_GRAPH_UNDIRECTED_GRAPH_CPP_

#include <cgogn/core/graph/undirected_graph.h>

namespace cgogn
{

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

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

#include <cgogn/io/graph_import.h>

namespace cgogn
{

namespace io
{

uint32 GraphImport::insert_line_vertex_container()
{
	return vertex_attributes_.template insert_lines<1>();
}

void GraphImport::reserve(uint32 nb_edges)
{
	edges_nb_vertices_.reserve(nb_edges);
	edges_vertex_indices_.reserve(nb_edges);
}

void GraphImport::add_edge(uint32 p0, uint32 p1)
{
	edges_nb_vertices_.push_back(2);
	edges_vertex_indices_.push_back(p0);
	edges_vertex_indices_.push_back(p1);
}

void GraphImport::add_edge_attribute(const DataInputGen& in_data, const std::string& att_name)
{
	ChunkArrayGen* att = in_data.add_attribute(edge_attributes_, att_name);
	in_data.to_chunk_array(att);
}

uint32 GraphImport::nb_edges() const
{
	return uint32(edges_nb_vertices_.size());
}

} // namespace io

} // namespace cgogn

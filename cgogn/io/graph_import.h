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

#ifndef CGOGN_IO_GRAPH_IMPORT_H_
#define CGOGN_IO_GRAPH_IMPORT_H_

#include <cgogn/core/cmap/map_base_data.h>

#include <cgogn/io/cgogn_io_export.h>

#include <cgogn/io/mesh_io_gen.h>
#include <cgogn/io/data_io.h>
#include <cgogn/io/cgogn_io_export.h>

namespace cgogn
{

namespace io
{

template <typename MAP>
class GraphImport
{
public:

	static_assert(MAP::DIMENSION == 1, "Must use map of dimension 1 in surface import");

	using Self = GraphImport<MAP>;

	using ChunkArrayContainer = MapBaseData::ChunkArrayContainer<uint32>;
	using ChunkArrayGen = MapBaseData::ChunkArrayGen;
	template <typename T>
	using ChunkArray = MapBaseData::ChunkArray<T>;

	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using MapBuilder = typename MAP::Builder;

	template <typename T, Orbit ORBIT>
	using Attribute = Attribute<T, ORBIT>;

	using DataInputGen = cgogn::io::DataInputGen;

	inline GraphImport(MAP& map) :
		edges_vertex_indices_(),
		vertex_attributes_(),
		map_(map),
		mbuild_(map)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(GraphImport);
	virtual ~GraphImport() {}

	void create_map()
	{
		if (vertex_attributes_.size() == 0u)
			return;

		mbuild_.template create_embedding<Vertex::ORBIT>();
		mbuild_.template swap_chunk_array_container<Vertex::ORBIT>(this->vertex_attributes_);

		auto vertex_dart = map_.template add_attribute<Dart, Vertex>("__vertex_dart");

		for (uint32 i = vertex_attributes_.begin(); i != vertex_attributes_.end(); vertex_attributes_.next(i))
		{
			Dart d = mbuild_.add_vertex_topo();
			mbuild_.template set_embedding<Vertex>(d, i);
			vertex_dart[i] = d;
		}

		for (uint32 i = 0; i < edges_vertex_indices_.size(); i += 2)
			mbuild_.connect_vertices_topo(edges_vertex_indices_[i], edges_vertex_indices_[i+1]);

		map_.remove_attribute(vertex_dart);
	}

	uint32 insert_line_vertex_container()
	{
		return vertex_attributes_.template insert_lines<1>();
	}

	void reserve(uint32 nb_edges)
	{
		edges_vertex_indices_.reserve(nb_edges);
	}

	void add_edge(uint32 p0, uint32 p1)
	{
		edges_vertex_indices_.push_back(p0);
		edges_vertex_indices_.push_back(p1);
	}

	inline ChunkArrayGen* add_vertex_attribute(const DataInputGen& in_data, const std::string& att_name)
	{
		ChunkArrayGen* att = in_data.add_attribute(vertex_attributes_, att_name);
		in_data.to_chunk_array(att);
		return att;
	}

	template <typename T>
	inline ChunkArray<T>* add_vertex_attribute(const std::string& att_name)
	{
		return vertex_attributes_.template add_chunk_array<T>(att_name);
	}

protected:

	std::vector<uint32> edges_vertex_indices_;
	ChunkArrayContainer vertex_attributes_;

	MAP& map_;
	MapBuilder mbuild_;
};

///
/// \class GraphFileImport
/// Imports a skeleton from a file
///
template <typename MAP>
class GraphFileImport : public GraphImport<MAP>, public FileImport
{
	using Self = GraphFileImport<MAP>;
	using Inherit_Import = GraphImport<MAP>;
	using Inherit_File = FileImport;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(GraphFileImport);

public:

	inline GraphFileImport(MAP& map) : Inherit_Import(map), Inherit_File() {}
	virtual ~GraphFileImport() {}
};

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_GRAPH_IMPORT_H_

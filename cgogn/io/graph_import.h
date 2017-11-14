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

#include <cgogn/io/dll.h>

#include <cgogn/io/mesh_io_gen.h>
#include <cgogn/io/data_io.h>

namespace cgogn
{

namespace io
{

class CGOGN_IO_API GraphImport
{
public:

	using Self = GraphImport;

	using ChunkArrayContainer = MapBaseData::ChunkArrayContainer<uint32>;
	using ChunkArrayGen = MapBaseData::ChunkArrayGen;
	template <typename T>
	using ChunkArray = MapBaseData::ChunkArray<T>;

	template <typename T, Orbit ORBIT>
	using Attribute = Attribute<T, ORBIT>;

	using DataInputGen = cgogn::io::DataInputGen;

	inline GraphImport():
		edges_nb_vertices_(),
		edges_vertex_indices_(),
		vertex_attributes_(),
		edge_attributes_()
	{}

	virtual ~GraphImport()
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(GraphImport);

	template <typename Map>
	void create(Map& map)
	{
		using Vertex = typename Map::Vertex;
		using MapBuilder = typename Map::Builder;

		if (nb_edges() == 0u)
			return;

		MapBuilder mbuild(map);
		map.clear_and_remove_attributes();

		mbuild.template create_embedding<Vertex::ORBIT>();
		mbuild.template swap_chunk_array_container<Vertex::ORBIT>(this->vertex_attributes_);

		auto darts_per_vertex = map.template add_attribute<std::vector<Dart>, Vertex>("darts_per_vertex");

		uint32 edges_vertex_index = 0;
		std::vector<uint32> edges_buffer;
		edges_buffer.reserve(16);

		for (uint32 i = 0, end = nb_edges(); i < end; ++i)
		{
			uint32 nbe = this->edges_nb_vertices_[i];

			edges_buffer.clear();
			for (uint32 j = 0u; j < nbe; ++j)
			{
				uint32 idx = this->edges_vertex_indices_[edges_vertex_index++];
				edges_buffer.push_back(idx);
			}

			Dart d = mbuild.add_edge_topo();

			for (uint32 j = 0u; j < nbe; ++j)
			{
				const uint32 vertex_index = edges_buffer[j];
				mbuild.template set_embedding<Vertex>(d, vertex_index);
				darts_per_vertex[vertex_index].push_back(d);
				d = map.alpha0(d);
			}
		}

		typename Map::DartMarker treated(map);
		map.foreach_dart([&] (Dart d)
		{
			if (!treated.is_marked(d))
			{
				uint32 emb = map.embedding(Vertex(d));
				std::vector<Dart>& per_vertex = darts_per_vertex[emb];
				treated.mark(d);

				for (auto it = per_vertex.begin(); it != per_vertex.end(); ++it)
				{
					mbuild.alpha1_sew(d, *it);
					treated.mark(*it);
				}
			}
		});
	}

	uint32 insert_line_vertex_container();

	void reserve(uint32 nb_edges);

	void add_edge(uint32 p0, uint32 p1);

	inline ChunkArrayGen* add_vertex_attribute(const DataInputGen& in_data, const std::string& att_name)
	{
		ChunkArrayGen* att = in_data.add_attribute(vertex_attributes_, att_name);
		in_data.to_chunk_array(att);
		return att;
	}

	void add_edge_attribute(const DataInputGen& in_data, const std::string& att_name);

	template <typename T>
	inline ChunkArray<T>* add_vertex_attribute(const std::string& att_name)
	{
		return vertex_attributes_.template add_chunk_array<T>(att_name);
	}

	template <typename T>
	inline ChunkArray<T>* add_edge_attribute(const std::string& att_name)
	{
		return edge_attributes_.template add_chunk_array<T>(att_name);
	}

private:

	uint32 nb_edges() const;

protected:

	std::vector<uint32> edges_nb_vertices_;
	std::vector<uint32> edges_vertex_indices_;
	ChunkArrayContainer vertex_attributes_;
	ChunkArrayContainer edge_attributes_;
};

///
/// \class GraphFileImport
/// Imports a skeleton from a file
///
class GraphFileImport : public GraphImport, public FileImport
{
	using Self = GraphFileImport;
	using Inherit1 = GraphImport;
	using Inherit2 = FileImport;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(GraphFileImport);

public:

	inline GraphFileImport() : Inherit1(), Inherit2()
	{}

	virtual ~GraphFileImport()
	{}
};

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_GRAPH_IMPORT_H_

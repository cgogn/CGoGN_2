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

#include <cgogn/io/mesh_io_gen.h>
#include <cgogn/io/data_io.h>

namespace cgogn
{

namespace io
{

template <typename VEC3>
class GraphImport
{
public:

	using Self = GraphImport<VEC3>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;

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
		edge_attributes_(),
		position_attribute_(nullptr),
		radius_attribute_(nullptr)
	{}

	virtual ~GraphImport()
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(GraphImport);

	virtual void clear()
	{
		edges_nb_vertices_.clear();
		edges_vertex_indices_.clear();
		vertex_attributes_.remove_chunk_arrays();
		edge_attributes_.remove_chunk_arrays();
		position_attribute_ = nullptr;
		radius_attribute_ = nullptr;
	}

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
			uint nbe = this->edges_nb_vertices_[i];

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

		//PRIMALE VERSION
		/*
		if (nb_vertices() == 0u)
			return;

		MapBuilder mbuild(map);
		map.clear_and_remove_attributes();

		mbuild.template create_embedding<Vertex::ORBIT>();
		mbuild.template create_embedding<CDart::ORBIT>();
		mbuild.template swap_chunk_array_container<Vertex::ORBIT>(this->vertex_attributes_);

		auto darts_per_vertex = map.template add_attribute<std::vector<Dart>, Vertex>("darts_per_vertex");
		auto opposite_vertex = map.template add_attribute<uint32, CDart>("opposite_vertex");

		uint32 edges_vertex_index = 0;
		std::vector<uint32> edges_buffer;
		edges_buffer.reserve(16);

		for (uint32 i = 0, end = nb_edges(); i < end; ++i)
		{
			uint nbe = this->edges_nb_vertices_[i];

			edges_buffer.clear();
			for (uint32 j = 0u; j < nbe; ++j)
			{
				uint32 idx = this->edges_vertex_indices_[edges_vertex_index++];
				edges_buffer.push_back(idx);
			}

			Dart d = mbuild.add_vertex_topo(nbe);
			map.foreach_dart_of_orbit(Vertex(d), [&mbuild, &i, &darts_per_vertex] (Dart e)
			{
				mbuild.template new_orbit_embedding(CDart(e));
				mbuild.template set_embedding<Vertex>(e, i);
				darts_per_vertex[i].push_back(e);
			});

			for (uint32 j = 0u; j < nbe; ++j)
			{
				const uint32 vertex_index = edges_buffer[j];
				opposite_vertex[d] = vertex_index;
				d = map.alpha1(d);
			}
		}

		typename Map::DartMarker treated(map);
		map.foreach_dart([&] (Dart d)
		{
			if (!treated.is_marked(d))
			{
				uint32 emb = opposite_vertex[d];
				std::vector<Dart>& per_vertex = darts_per_vertex[emb];
				treated.mark(d);

				for (auto it = per_vertex.begin();
					 it != per_vertex.end();
					 ++it)
				{
					if (opposite_vertex[CDart(*it)] == map.embedding(Vertex(d)))
					{
						mbuild.alpha0_sew(d, *it);
						treated.mark(*it);
					}
				}
			}
		});

		*/
	}

	inline uint32 insert_line_vertex_container()
	{
		return vertex_attributes_.template insert_lines<1>();
	}

	inline void reserve(uint32 nb_edges)
	{
		edges_nb_vertices_.reserve(nb_edges);
		edges_vertex_indices_.reserve(nb_edges);
	}

	inline void add_edge(uint32 p0, uint32 p1)
	{
		edges_nb_vertices_.push_back(2);
		edges_vertex_indices_.push_back(p0);
		edges_vertex_indices_.push_back(p1);
	}

	inline void add_vertex_attribute(const DataInputGen& in_data, const std::string& att_name)
	{
		ChunkArrayGen* att = in_data.add_attribute(vertex_attributes_, att_name);
		in_data.to_chunk_array(att);
		if (att_name == "position")
			position_attribute_ = dynamic_cast<ChunkArray<VEC3>*>(att);
		else if (att_name == "radius")
			radius_attribute_ = dynamic_cast<ChunkArray<Scalar>*>(att);
	}

	inline void add_edge_attribute(const DataInputGen& in_data, const std::string& att_name)
	{
		ChunkArrayGen* att = in_data.add_attribute(edge_attributes_, att_name);
		in_data.to_chunk_array(att);
	}

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

	inline ChunkArray<VEC3>* position_attribute()
	{
		if (position_attribute_ == nullptr)
			return (position_attribute_ = add_vertex_attribute<VEC3>("position"));
		else
			return position_attribute_;
	}

	inline ChunkArray<Scalar>* radius_attribute()
	{
		if (radius_attribute_ == nullptr)
			return (radius_attribute_ = add_vertex_attribute<Scalar>("radius"));
		else
			return radius_attribute_;
	}

private:

	inline uint32 nb_edges() const
	{
		return uint32(edges_nb_vertices_.size());
	}

protected:

	std::vector<uint32> edges_nb_vertices_;
	std::vector<uint32> edges_vertex_indices_;
	ChunkArrayContainer vertex_attributes_;
	ChunkArrayContainer edge_attributes_;
	ChunkArray<VEC3>* position_attribute_;
	ChunkArray<Scalar>* radius_attribute_;
};

///
/// \class GraphFileImport
/// Imports a skeleton from a file
///
template <typename VEC3>
class GraphFileImport : public GraphImport<VEC3>, public FileImport
{
	using Self = GraphFileImport<VEC3>;
	using Inherit1 = GraphImport<VEC3>;
	using Inherit2 = FileImport;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(GraphFileImport);

public:

	inline GraphFileImport() : Inherit1(), Inherit2()
	{}

	virtual ~GraphFileImport()
	{}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_GRAPH_IMPORT_CPP_))
extern template class CGOGN_IO_API GraphImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API GraphImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API GraphFileImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API GraphFileImport<Eigen::Vector3d>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_GRAPH_IMPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_GRAPH_IMPORT_H_

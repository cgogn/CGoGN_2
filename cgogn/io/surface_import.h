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

#ifndef IO_SURFACE_IMPORT_H_
#define IO_SURFACE_IMPORT_H_

#include <istream>
#include <sstream>

#include <core/utils/endian.h>
#include <core/utils/name_types.h>
#include <core/utils/string.h>
#include <core/container/chunk_array_container.h>
#include <core/cmap/cmap2.h>
#include <core/cmap/cmap2_builder.h>

#include <io/dll.h>
#include <io/c_locale.h>
#include <io/mesh_io_gen.h>

namespace cgogn
{

namespace io
{


template <typename MAP_TRAITS>
class SurfaceImport : public MeshImportGen
{
public:
	using Self = SurfaceImport<MAP_TRAITS>;
	using Inherit = MeshImportGen;
	using Map = CMap2<MAP_TRAITS>;

	static const unsigned int CHUNK_SIZE = MAP_TRAITS::CHUNK_SIZE;

	template <typename T>
	using ChunkArray = cgogn::ChunkArray<CHUNK_SIZE, T>;
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNK_SIZE, unsigned int>;
	template <typename T, Orbit ORBIT>
	using AttributeHandler = AttributeHandler<MAP_TRAITS, T, ORBIT>;


protected:
	unsigned int nb_vertices_;
	unsigned int nb_edges_;
	unsigned int nb_faces_;

	std::vector<unsigned int> faces_nb_edges_;
	std::vector<unsigned int> faces_vertex_indices_;

	ChunkArrayContainer vertex_attributes_;
	ChunkArrayContainer face_attributes_;

public:

	SurfaceImport() :
		nb_vertices_(0u)
	  ,nb_edges_(0u)
	  ,nb_faces_(0u)
	  ,faces_nb_edges_()
	  ,faces_vertex_indices_()
	{}

	virtual ~SurfaceImport() override
	{}

	SurfaceImport(const Self&) = delete;
	SurfaceImport(Self&&) = delete;
	Self& operator=(const Self&) = delete;
	Self& operator=(Self&&) = delete;

	virtual void clear() override
	{
		nb_vertices_ = 0;
		nb_edges_ = 0;
		nb_faces_ = 0;
		faces_nb_edges_.clear();
		faces_vertex_indices_.clear();
		vertex_attributes_.remove_attributes();
		face_attributes_.remove_attributes();
	}

	inline void create_map(Map& map)
	{
		using Vertex = typename Map::Vertex;
		using Edge = typename Map::Edge;
		using Face = typename Map::Face;
		using MapBuilder = cgogn::CMap2Builder_T<typename Map::MapTraits>;

		if (this->nb_vertices_ == 0u)
			return;

		MapBuilder mbuild(map);
		map.clear_and_remove_attributes();

		mbuild.template create_embedding<Vertex::ORBIT>();
		mbuild.template swap_chunk_array_container<Vertex::ORBIT>(this->vertex_attributes_);

		typename Map::template VertexAttributeHandler<std::vector<Dart>> darts_per_vertex =
				map.template add_attribute<std::vector<Dart>, Vertex::ORBIT>("darts_per_vertex");

		unsigned int faces_vertex_index = 0;
		std::vector<unsigned int> vertices_buffer;
		vertices_buffer.reserve(16);

		for (unsigned int i = 0; i < this->nb_faces_; ++i)
		{
			unsigned int nbe = this->faces_nb_edges_[i];

			vertices_buffer.clear();
			unsigned int prev = std::numeric_limits<unsigned int>::max();

			for (unsigned int j = 0; j < nbe; ++j)
			{
				unsigned int idx = this->faces_vertex_indices_[faces_vertex_index++];
				if (idx != prev)
				{
					prev = idx;
					vertices_buffer.push_back(idx);
				}
			}
			if (vertices_buffer.front() == vertices_buffer.back())
				vertices_buffer.pop_back();

			nbe = static_cast<unsigned short>(vertices_buffer.size());
			if (nbe > 2)
			{
				Dart d = mbuild.add_face_topo_parent(nbe);
				for (unsigned int j = 0u; j < nbe; ++j)
				{
					const unsigned int vertex_index = vertices_buffer[j];
					mbuild.template set_embedding<Vertex>(d, vertex_index);
					darts_per_vertex[vertex_index].push_back(d);
					d = map.phi1(d);
				}
			}
		}

		bool need_vertex_unicity_check = false;
		unsigned int nb_boundary_edges = 0;

		for (Dart d : map)
		{
			if (map.phi2(d) == d)
			{
				unsigned int vertex_index = map.get_embedding(Vertex(d));

				std::vector<Dart>& next_vertex_darts = darts_per_vertex[Vertex(map.phi1(d))];
				bool phi2_found = false;
				bool first_OK = true;

				for (auto it = next_vertex_darts.begin();
					 it != next_vertex_darts.end() && !phi2_found;
					 ++it)
				{
					if (map.get_embedding(Vertex(map.phi1(*it))) == vertex_index)
					{
						if (map.phi2(*it) == *it)
						{
							mbuild.phi2_sew(d, *it);
							phi2_found = true;
						}
						else
						{
							first_OK = false;
						}
					}
				}

				if (!phi2_found)
					++nb_boundary_edges;

				if (!first_OK)
					need_vertex_unicity_check = true;
			}
		}

		if (nb_boundary_edges > 0)
			mbuild.close_map();

		if (need_vertex_unicity_check)
			map.template enforce_unique_orbit_embedding<Vertex::ORBIT>();

		if (this->face_attributes_.get_nb_attributes() > 0)
		{
			mbuild.template create_embedding<Face::ORBIT>();
			mbuild.template swap_chunk_array_container<Face::ORBIT>(this->face_attributes_);
		}

		map.remove_attribute(darts_per_vertex);
		this->clear();
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_SURFACE_IMPORT_CPP_))
extern template class CGOGN_IO_API SurfaceImport<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_SURFACE_IMPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // IO_SURFACE_IMPORT_H_

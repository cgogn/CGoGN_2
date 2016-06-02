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

#ifndef CGOGN_IO_SURFACE_IMPORT_H_
#define CGOGN_IO_SURFACE_IMPORT_H_

#include <istream>
#include <sstream>

#include <cgogn/core/utils/endian.h>
#include <cgogn/core/utils/name_types.h>
#include <cgogn/core/utils/string.h>
#include <cgogn/core/container/chunk_array_container.h>
#include <cgogn/core/cmap/map_base.h>


#include <cgogn/io/dll.h>
#include <cgogn/io/c_locale.h>
#include <cgogn/io/mesh_io_gen.h>

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

	static const uint32 CHUNK_SIZE = MAP_TRAITS::CHUNK_SIZE;

	template <typename T>
	using ChunkArray = cgogn::ChunkArray<CHUNK_SIZE, T>;
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNK_SIZE, uint32>;
	template <typename T, Orbit ORBIT>
	using Attribute = Attribute<MAP_TRAITS, T, ORBIT>;

protected:

	uint32 nb_vertices_;
	uint32 nb_edges_;
	uint32 nb_faces_;

	std::vector<uint32> faces_nb_edges_;
	std::vector<uint32> faces_vertex_indices_;

	ChunkArrayContainer vertex_attributes_;
	ChunkArrayContainer face_attributes_;

public:

	inline SurfaceImport() :
		nb_vertices_(0u)
	  ,nb_edges_(0u)
	  ,nb_faces_(0u)
	  ,faces_nb_edges_()
	  ,faces_vertex_indices_()
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(SurfaceImport);

	virtual ~SurfaceImport() override
	{}

	virtual void clear() override
	{
		nb_vertices_ = 0;
		nb_edges_ = 0;
		nb_faces_ = 0;
		faces_nb_edges_.clear();
		faces_vertex_indices_.clear();
		vertex_attributes_.remove_chunk_arrays();
		face_attributes_.remove_chunk_arrays();
	}

	template <typename Map>
	void create_map(Map& map)
	{
		static_assert(Map::DIMENSION == 2, "must use map of dim 2 in surface import");

		using Vertex = typename Map::Vertex;
		using Edge = typename Map::Edge;
		using Face = typename Map::Face;
		using MapBuilder = typename Map::Builder;

		if (this->nb_vertices_ == 0u)
			return;

		MapBuilder mbuild(map);
		map.clear_and_remove_attributes();

		mbuild.template create_embedding<Vertex::ORBIT>();
		mbuild.template swap_chunk_array_container<Vertex::ORBIT>(this->vertex_attributes_);

		typename Map::template VertexAttribute<std::vector<Dart>> darts_per_vertex =
			map.template add_attribute<std::vector<Dart>, Vertex::ORBIT>("darts_per_vertex");

		uint32 faces_vertex_index = 0;
		std::vector<uint32> vertices_buffer;
		vertices_buffer.reserve(16);

		for (uint32 i = 0; i < this->nb_faces_; ++i)
		{
			uint32 nbe = this->faces_nb_edges_[i];

			vertices_buffer.clear();
			uint32 prev = std::numeric_limits<uint32>::max();

			for (uint32 j = 0; j < nbe; ++j)
			{
				uint32 idx = this->faces_vertex_indices_[faces_vertex_index++];
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
				for (uint32 j = 0u; j < nbe; ++j)
				{
					const uint32 vertex_index = vertices_buffer[j];
					mbuild.template set_embedding<Vertex>(d, vertex_index);
					darts_per_vertex[vertex_index].push_back(d);
					d = map.phi1(d);
				}
			}
		}

		bool need_vertex_unicity_check = false;
		uint32 nb_boundary_edges = 0;

		map.foreach_dart([&] (Dart d)
		{
			if (map.phi2(d) == d)
			{
				uint32 vertex_index = map.embedding(Vertex(d));

				std::vector<Dart>& next_vertex_darts = darts_per_vertex[Vertex(map.phi1(d))];
				bool phi2_found = false;
				bool first_OK = true;

				for (auto it = next_vertex_darts.begin();
					 it != next_vertex_darts.end() && !phi2_found;
					 ++it)
				{
					if (map.embedding(Vertex(map.phi1(*it))) == vertex_index)
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
		});

		if (nb_boundary_edges > 0)
		{
			uint32 nb_holes = mbuild.close_map();
			cgogn_log_info("create_map") << nb_holes << " hole(s) have been closed";
		}

		if (need_vertex_unicity_check)
		{
			map.template enforce_unique_orbit_embedding<Vertex::ORBIT>();
			cgogn_log_warning("create_map") << "Import Surface: non manifold vertices detected and corrected";
		}

		if (this->face_attributes_.nb_chunk_arrays() > 0)
		{
			mbuild.template create_embedding<Face::ORBIT>();
			mbuild.template swap_chunk_array_container<Face::ORBIT>(this->face_attributes_);
		}

		map.remove_attribute(darts_per_vertex);
		this->clear();
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_SURFACE_IMPORT_CPP_))
extern template class CGOGN_IO_API SurfaceImport<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_SURFACE_IMPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_SURFACE_IMPORT_H_

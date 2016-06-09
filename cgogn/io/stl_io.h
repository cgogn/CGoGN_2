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

#ifndef CGOGN_IO_STL_IO_H_
#define CGOGN_IO_STL_IO_H_

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/surface_import.h>
#include <cgogn/io/surface_export.h>

#include <iomanip>

namespace cgogn
{

namespace io
{

template <typename MAP_TRAITS, typename VEC3>
class StlSurfaceImport : public SurfaceImport<MAP_TRAITS>
{
public:

	using Self = StlSurfaceImport<MAP_TRAITS, VEC3>;
	using Inherit = SurfaceImport<MAP_TRAITS>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline StlSurfaceImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(StlSurfaceImport);
	virtual ~StlSurfaceImport() override
	{}

protected:

	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename, std::ios::in);
		ChunkArray<VEC3>* position = this->vertex_attributes_.template add_chunk_array<VEC3>("position");
		ChunkArray<VEC3>* normal = this->face_attributes_.template add_chunk_array<VEC3>("normal");
		std::string word;
		fp >> word;
		fp.close();
		if (to_lower(word) == "solid")
			return this->import_ascii(filename, position, normal);
		else
			return this->import_binary(filename, position, normal);
	}
private:
	bool import_ascii(const std::string& filename, ChunkArray<VEC3>* position, ChunkArray<VEC3>* normal)
	{
		std::ifstream fp(filename, std::ios::in);

		std::vector<VEC3> vertices_set;
		vertices_set.reserve(1024u);
		std::vector<uint32> indices_set;
		indices_set.reserve(vertices_set.size());

		std::string line;
		std::getline(fp, line); // 1st line : solid name

		while (fp.good())
		{
			std::getline(fp, line); // facet normal ni nj nk
			std::stringstream stream_normal(line);
			VEC3 norm;
			stream_normal >> line;
			if (to_lower(line) == "endsolid")
				break;
			stream_normal >> line >> norm[0] >> norm[1] >> norm[2];
			const uint32 face_id = this->face_attributes_.template insert_lines<1>();
			(*normal)[face_id] = norm;
			std::getline(fp, line); // outer loop

			std::array<VEC3, 3> pos;
			for (uint32 i = 0u; i < 3; ++i)
			{
				std::getline(fp, line);
				std::stringstream stream_pos(line);
				stream_pos >> line >> pos[i][0] >> pos[i][1] >> pos[i][2];
				auto it = std::find(vertices_set.begin(), vertices_set.end(), pos[i]);
				uint32 vertex_id{UINT32_MAX};

				if (it == vertices_set.end())
				{
					vertex_id = this->vertex_attributes_.template insert_lines<1>();
					vertices_set.push_back(pos[i]);
					indices_set.push_back(vertex_id);
					this->nb_vertices_++;
				} else {
					auto id_it = indices_set.begin();
					std::advance(id_it, std::distance(vertices_set.begin(), it));
					vertex_id = *id_it;
				}
				(*position)[vertex_id] = pos[i];
				this->faces_vertex_indices_.push_back(vertex_id);
			}
			this->faces_nb_edges_.push_back(3u);
			this->nb_faces_++;
			std::getline(fp, line); // endloop
			std::getline(fp, line); // endfacet
		}

		return true;
	}
	bool import_binary(const std::string& filename, ChunkArray<VEC3>* position, ChunkArray<VEC3>* normal)
	{
		std::ifstream fp(filename, std::ios::in | std::ios::binary);

		std::array<uint32, 21> header;
		fp.read(reinterpret_cast<char*>(&header[0]), 21u* sizeof(uint32));
		this->nb_faces_ = swap_endianness_native_little(*reinterpret_cast<uint32*>(&header[20]));

		std::array<float32, 3> normal_buffer;
		std::array<float32, 9> position_buffer;
		std::vector<VEC3> vertices_set;
		vertices_set.reserve(2u*this->nb_faces_);
		std::vector<uint32> indices_set;
		indices_set.reserve(vertices_set.size());

		for(uint32 i = 0u; i < this->nb_faces_; ++i)
		{
			fp.read(reinterpret_cast<char*>(&normal_buffer[0]), 3u*sizeof(float32));
			fp.read(reinterpret_cast<char*>(&position_buffer[0]), 3u*3u*sizeof(float32));
			fp.ignore(sizeof(int16));
			for (auto& x : normal_buffer)
				x = swap_endianness_native_little(x);

			for (auto& x : position_buffer)
				x = swap_endianness_native_little(x);

			const uint32 face_id = this->face_attributes_.template insert_lines<1>();
			const VEC3 norm{Scalar(normal_buffer[0]), Scalar(normal_buffer[1]), Scalar(normal_buffer[2])};
			(*normal)[face_id] = norm;

			for(uint32 vid = 0u; vid < 3u ; ++vid)
			{
				const VEC3 pos{Scalar(position_buffer[3u*vid]), Scalar(position_buffer[3u*vid + 1u]), Scalar(position_buffer[3u*vid + 2u])};
				auto it = std::find(vertices_set.begin(), vertices_set.end(), pos);
				uint32 vertex_id{UINT32_MAX};

				if (it == vertices_set.end())
				{
					vertex_id = this->vertex_attributes_.template insert_lines<1>();
					vertices_set.push_back(pos);
					indices_set.push_back(vertex_id);

					this->nb_vertices_++;
				} else {
					auto id_it = indices_set.begin();
					std::advance(id_it, std::distance(vertices_set.begin(), it));
					vertex_id = *id_it;
				}
				(*position)[vertex_id] = pos;
				this->faces_vertex_indices_.push_back(vertex_id);
			}
			this->faces_nb_edges_.push_back(3u);
		}
		return true;
	}
};

template <typename MAP>
class StlSurfaceExport : public SurfaceExport<MAP>
{
public:

	using Inherit = SurfaceExport<MAP>;
	using Self = StlSurfaceExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;
	using Face = typename Inherit::Face;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;
	template<typename T>
	using VertexAttribute = typename Inherit::template VertexAttribute<T>;
	using ChunkArrayContainer = typename Inherit::ChunkArrayContainer;

protected:

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& option) override
	{
		ChunkArrayGen* normal_attribute(nullptr);
		const ChunkArrayContainer& face_cac = map.template const_attribute_container<Face::ORBIT>();

		for (const auto& pair : option.attributes_to_export_)
			if (pair.first == Face::ORBIT && (to_lower(pair.second) == "normal" || to_lower(pair.second) == "normals"))
			{
				normal_attribute = face_cac.get_chunk_array(pair.second);
				break;
			}
		if (normal_attribute == nullptr)
			normal_attribute = face_cac.get_chunk_array("normal");

		if (normal_attribute == nullptr)
		{
			cgogn_log_warning("StlSurfaceExport::export_file_impl") << "A face normal attribute has to be provided before exporting a STL file.";
			return;
		}

		if (option.binary_)
			this->export_binary(map, output, option, normal_attribute);
		else
			this->export_ascii(map, output, option, normal_attribute);

	}
private:
	void export_ascii(const Map& map, std::ofstream& output, const ExportOptions& option, ChunkArrayGen* normal_attribute)
	{
		// set precision for float output
		output << std::setprecision(12);

		output << "solid " << remove_extension(option.filename_)  << std::endl;

		map.foreach_cell([&] (Face f)
		{
			if (map.codegree(f) == 3)
			{
				output << "facet normal ";
				normal_attribute->export_element(map.embedding(f), output, false, false);
				output << std::endl;
				output << "outer loop" << std::endl;
				map.template foreach_incident_vertex(f, [&] (Vertex v)
				{
					output << "vertex ";
					this->position_attribute_->export_element(map.embedding(v), output, false, false);
					output << std::endl;
				});
				output << "endloop" << std::endl;
				output << "endfacet" << std::endl;
			} else
				cgogn_log_warning("StlSurfaceExport::export_ascii") << "STL doesn't support non-triangular faces.";
		}, *(this->cell_cache_));

		output << "endsolid " << remove_extension(option.filename_) << std::endl;
	}

	void export_binary(const Map& map, std::ofstream& output, const ExportOptions& option, ChunkArrayGen* normal_attribute)
	{
		// header + nb triangles
		std::array<uint32, 21> header;
		header.fill(0u);
		header[20] = map.template nb_cells<Face::ORBIT>();
		output.write(reinterpret_cast<char*>(&header[0]),21*sizeof(uint32));

		uint16 attribute_byte_count{0u};
		uint32 nb_tri{0u};


		// write face cutted in triangle if necessary
		map.foreach_cell([&] (Face f)
		{
			if (map.codegree(f) == 3)
			{
				normal_attribute->export_element(map.embedding(f), output, true, true, 4ul);
				map.template foreach_incident_vertex(f, [&] (Vertex v)
				{
					this->position_attribute_->export_element(map.embedding(v), output, true, true, 4ul);
				});
				output.write(reinterpret_cast<char*>(&attribute_byte_count),sizeof(uint16));
				++nb_tri;
			}
			else
				cgogn_log_warning("StlSurfaceExport::export_ascii") << "STL doesn't support non-triangular faces.";
		}, *(this->cell_cache_));

		// update nb of triangles in file if necessary
		if (nb_tri != header[20])
		{
			output.seekp(80);
			output.write(reinterpret_cast<char*>(&nb_tri),sizeof(uint32));
		}
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_STL_IO_CPP_))
extern template class CGOGN_IO_API StlSurfaceExport<CMap2<DefaultMapTraits>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_STL_IO_CPP_))

} // namespace io

} // namespace cgogn


#endif // CGOGN_IO_STL_IO_H_

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

#ifndef CORE_IO_SURFACE_IMPORT_H_
#define CORE_IO_SURFACE_IMPORT_H_

#include <istream>

#include <core/container/chunk_array_container.h>
#include <core/map/cmap2.h>
#include <core/map/cmap2_builder.h>

namespace cgogn
{

namespace io
{

enum SurfaceFileType
{
	SurfaceFileType_UNKNOWN = 0,
	SurfaceFileType_OFF,
	SurfaceFileType_OBJ
};

inline SurfaceFileType get_file_type(const std::string& filename)
{
	if (filename.rfind(".off") != std::string::npos || filename.rfind(".OFF") != std::string::npos)
		return SurfaceFileType_OFF;
	if (filename.rfind(".obj") != std::string::npos || filename.rfind(".OBJ") != std::string::npos)
		return SurfaceFileType_OBJ;
	return SurfaceFileType_UNKNOWN;
}

template <typename MAP_TRAITS>
class SurfaceImport
{
public:

	using Self = SurfaceImport<MAP_TRAITS>;
	using Map = CMap2<MAP_TRAITS>;

	static const unsigned int CHUNK_SIZE = MAP_TRAITS::CHUNK_SIZE;

	template<typename T>
	using ChunkArray = ChunkArray<CHUNK_SIZE, T>;
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNK_SIZE, unsigned int>;

	template<typename T, Orbit ORBIT>
	using AttributeHandler = AttributeHandler<MAP_TRAITS, T, ORBIT>;
	template<typename T>
	using VertexAttributeHandler = AttributeHandler<T, Map::VERTEX>;

	using Vec3 = typename MAP_TRAITS::Vec3;

	unsigned int nb_vertices_;
	unsigned int nb_edges_;
	unsigned int nb_faces_;

	std::vector<unsigned short> faces_nb_edges_;
	std::vector<unsigned int> faces_vertex_indices_;

	ChunkArrayContainer vertex_attributes_;

	SurfaceImport() :
		nb_vertices_(0u)
		,nb_edges_(0u)
		,nb_faces_(0u)
		,faces_nb_edges_()
		,faces_vertex_indices_()
	{}

	~SurfaceImport()
	{}

	SurfaceImport(const Self&) = delete;
	SurfaceImport(Self&&) = delete;
	Self& operator=(const Self&) = delete;
	Self& operator=(Self&&) = delete;

	void clear()
	{
		nb_vertices_ = 0;
		nb_edges_ = 0;
		nb_faces_ = 0;
		faces_nb_edges_.clear();
		faces_vertex_indices_.clear();
		vertex_attributes_.remove_attributes();
	}

	bool import_file(const std::string& filename)
	{
		return import_file(filename, get_file_type(filename));
	}

	bool import_file(const std::string& filename, SurfaceFileType type)
	{
		clear();

		std::ifstream fp(filename.c_str(), std::ios::in);
		if (!fp.good())
		{
			std::cout << "Unable to open file " << filename << std::endl;
			return false;
		}

		bool result = false;
		switch (type)
		{
		case SurfaceFileType_UNKNOWN :
			std::cout << "Unknown file type " << filename << std::endl;
			result = false;
			break;
		case SurfaceFileType_OFF :
			result = import_OFF(fp);
			break;
		case SurfaceFileType_OBJ :
			result = import_OBJ(fp);
			break;
		}

		if (!result)
			this->clear();

		return result;
	}

	void create_map(Map& map)
	{
		using MapBuilder = cgogn::CMap2Builder_T<typename Map::MapTraits>;

		if (this->nb_vertices_ == 0u)
			return;

		MapBuilder mbuild(map);
		const Orbit VERTEX = Map::VERTEX;
		map.clear_and_remove_attributes();

		map.template create_embedding<VERTEX>();
		mbuild.template swapChunkArrayContainer<VERTEX>(this->vertex_attributes_);

		VertexAttributeHandler<std::vector<Dart>> darts_per_vertex =
		map.template add_attribute<std::vector<Dart>, VERTEX>("darts_per_vertex");

		unsigned int faces_vertex_index = 0;
		std::vector<unsigned int> vertices_buffer;
		vertices_buffer.reserve(16);

		for (unsigned int i = 0; i < this->nb_faces_; ++i)
		{
			unsigned short nbe = this->faces_nb_edges_[i];

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
			Dart d = mbuild.add_face_topo(nbe);
			for (unsigned int j = 0; j < nbe; ++j)
			{
				unsigned int vertex_index = vertices_buffer[j];
				map.template init_embedding<VERTEX>(d, vertex_index);
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
			unsigned int vertex_index = map.template get_embedding<VERTEX>(d);

			std::vector<Dart>& next_vertex_darts = darts_per_vertex[map.phi1(d)];
			bool phi2_found = false;
			bool first_OK = true;

			for (auto it = next_vertex_darts.begin();
			it != next_vertex_darts.end() && !phi2_found;
			++it)
			{
				if (map.template get_embedding<VERTEX>(map.phi1(*it)) == vertex_index)
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
			map.template unique_orbit_embedding<VERTEX>();

		map.remove_attribute(darts_per_vertex);
		this->clear();
	}


protected:

	bool import_OFF(std::ifstream& fp)
	{
		std::string line;

		// read OFF header
		std::getline(fp, line);
		if (line.rfind("OFF") == std::string::npos)
		{
			std::cout << "Problem reading off file: not an off file" << std::endl;
			std::cout << line << std::endl;
			return false;
		}

		// read number of vertices, edges, faces
		do
		{
			std::getline(fp, line);
		} while (line.size() == 0);
		{ // limit scope of oss
			std::stringstream oss(line);

			oss >> nb_vertices_;
			oss >> nb_faces_;
			oss >> nb_edges_;
		}
		ChunkArray<Vec3>* position =
			vertex_attributes_.template add_attribute<Vec3>("position");

		// read vertices position
		std::vector<unsigned int> vertices_id;
		vertices_id.reserve(nb_vertices_);

		for (unsigned int i = 0; i < nb_vertices_; ++i)
		{
			do
			{
				std::getline(fp, line);
			} while (line.size() == 0);

			std::stringstream oss(line);

			double x, y, z;
			oss >> x;
			oss >> y;
			oss >> z;

			Vec3 pos{x, y, z};

			unsigned int vertex_id = vertex_attributes_.template insert_lines<1>();
			(*position)[vertex_id] = pos;

			vertices_id.push_back(vertex_id);
		}

		// read faces (vertex indices)
		faces_nb_edges_.reserve(nb_faces_);
		faces_vertex_indices_.reserve(nb_vertices_ * 8);
		for (unsigned int i = 0; i < nb_faces_; ++i)
		{
			do
			{
				std::getline(fp, line);
			} while (line.size() == 0);

			std::stringstream oss(line);

			unsigned short n;
			oss >> n;
			faces_nb_edges_.push_back(n);
			for (unsigned int j = 0; j < n; ++j)
			{
				unsigned int index;
				oss >> index;
				faces_vertex_indices_.push_back(vertices_id[index]);
			}
		}

		return true;
	}

	bool import_OBJ(std::ifstream& fp)
	{
		ChunkArray<Vec3>* position =
			vertex_attributes_.template add_attribute<Vec3>("position");

		std::string line, tag;

		do
		{
			fp >> tag;
			std::getline(fp, line);
		} while (tag != std::string("v"));

		// lecture des sommets
		std::vector<unsigned int> vertices_id;
		vertices_id.reserve(102400);

		unsigned int i = 0;
		do
		{
			if (tag == std::string("v"))
			{
				std::stringstream oss(line);

				double x, y, z;
				oss >> x;
				oss >> y;
				oss >> z;

				Vec3 pos{x, y, z};

				unsigned int vertex_id = vertex_attributes_.template insert_lines<1>();
				(*position)[vertex_id] = pos;

				vertices_id.push_back(vertex_id);
				i++;
			}

			fp >> tag;
			std::getline(fp, line);
		} while (!fp.eof());

		nb_vertices_ = static_cast<unsigned int>(vertices_id.size());

		fp.clear();
		fp.seekg(0, std::ios::beg);

		do
		{
			fp >> tag;
			std::getline(fp, line);
		} while (tag != std::string("f"));

		faces_nb_edges_.reserve(vertices_id.size() * 2);
		faces_vertex_indices_.reserve(vertices_id.size() * 8);

		std::vector<unsigned int> table;
		table.reserve(64);
		do
		{
			if (tag == std::string("f")) // lecture d'une face
			{
				std::stringstream oss(line);

				table.clear();
				while (!oss.eof())  // lecture de tous les indices
				{
					std::string str;
					oss >> str;

					unsigned int ind = 0;

					while ((ind < str.length()) && (str[ind] != '/'))
					ind++;

					if (ind > 0)
					{
						unsigned int index;
						std::stringstream iss(str.substr(0, ind));
						iss >> index;
						table.push_back(index);
					}
				}

				unsigned int n = static_cast<unsigned int>(table.size());
				faces_nb_edges_.push_back(static_cast<unsigned short>(n));
				for (unsigned int j = 0; j < n; ++j)
				{
					unsigned int index = table[j] - 1; // indices start at 1
					faces_vertex_indices_.push_back(vertices_id[index]);
				}
				nb_faces_++;
			}
			fp >> tag;
			std::getline(fp, line);
		} while (!fp.eof());

		return true;
	}
};

} // namespace io

} // namespace cgogn

#endif // CORE_IO_SURFACE_IMPORT_H_

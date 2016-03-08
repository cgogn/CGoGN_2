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

#include <core/utils/endian.h>
#include <core/container/chunk_array_container.h>
#include <core/cmap/cmap2.h>
#include <core/cmap/cmap2_builder.h>
#include <core/utils/string.h>
#include <io/import_ply_data.h>

#include <io/c_locale.h>
#include <io/dll.h>

namespace cgogn
{

namespace io
{

enum SurfaceFileType
{
	SurfaceFileType_UNKNOWN = 0,
	SurfaceFileType_OFF,
	SurfaceFileType_OBJ,
	SurfaceFileType_PLY
};

inline SurfaceFileType get_file_type(const std::string& filename)
{
	const std::string& extension = to_lower(get_extension(filename));
	if (extension == "off")
		return SurfaceFileType_OFF;
	if (extension == "obj")
		return SurfaceFileType_OBJ;
	if (extension == "ply")
		return SurfaceFileType_PLY;
	return SurfaceFileType_UNKNOWN;
}

template <typename MAP_TRAITS>
class SurfaceImport
{
public:

	using Self = SurfaceImport<MAP_TRAITS>;
	using Map = CMap2<MAP_TRAITS>;
	using Vertex = typename Map::Vertex;

	static const unsigned int CHUNK_SIZE = MAP_TRAITS::CHUNK_SIZE;

	template <typename T>
	using ChunkArray = ChunkArray<CHUNK_SIZE, T>;
	using ChunkArrayContainer = cgogn::ChunkArrayContainer<CHUNK_SIZE, unsigned int>;

	template <typename T, Orbit ORBIT>
	using AttributeHandler = AttributeHandler<MAP_TRAITS, T, ORBIT>;
	template <typename T>

	using VertexAttributeHandler = typename Map::template VertexAttributeHandler<T>;

	unsigned int nb_vertices_;
	unsigned int nb_edges_;
	unsigned int nb_faces_;

	std::vector<unsigned int> faces_nb_edges_;
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

	template <typename VEC3>
	bool import_file(const std::string& filename)
	{
		return import_file<VEC3>(filename, get_file_type(filename));
	}

	template <typename VEC3>
	bool import_file(const std::string& filename, SurfaceFileType type)
	{
		//ensure that locale are set to C for reading files
		Scoped_C_Locale loc;

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
			result = import_OFF<VEC3>(fp);
			break;
		case SurfaceFileType_OBJ :
			result = import_OBJ<VEC3>(fp);
			break;
		case SurfaceFileType_PLY :
			fp.close();
			result = import_ply<VEC3>(filename);
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
		map.clear_and_remove_attributes();

		mbuild.template create_embedding<Vertex::ORBIT>();
		mbuild.template swap_chunk_array_container<Vertex::ORBIT>(this->vertex_attributes_);

		VertexAttributeHandler<std::vector<Dart>> darts_per_vertex =
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

				std::vector<Dart>& next_vertex_darts = darts_per_vertex[map.phi1(d)];
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

		map.remove_attribute(darts_per_vertex);
		this->clear();
	}

protected:

	template <typename VEC3>
	bool import_OFF(std::ifstream& fp)
	{
		using Scalar = typename VEC3::Scalar;

		std::string line;
		line.reserve(512);

		// local function for reading double with comment ignoring
		auto read_double = [&fp,&line]() -> double
		{
			fp >> line;
			while (line[0]=='#')
			{
				fp.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
				fp >> line;
			}
			return std::stod(line);
		};

		// local function for reading int with comment ignoring
		auto read_uint = [&fp,&line]() -> unsigned int
		{
			fp >> line;
			while (line[0]=='#')
			{
				fp.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
				fp >> line;
			}
			return (unsigned int)(std::stoul(line));
		};



		// read OFF header
		std::getline(fp, line);
		if (line.rfind("OFF") == std::string::npos)
		{
			std::cout << "Problem reading off file: not an off file" << std::endl;
			std::cout << line << std::endl;
			return false;
		}

		// check if binary file
		if (line.rfind("BINARY") != std::string::npos)
		{
			return import_OFF_BIN<VEC3>(fp);
		}


		// read number of vertices, edges, faces
		nb_vertices_ = read_uint();
		nb_faces_ = read_uint();
		nb_edges_ = read_uint();

		ChunkArray<VEC3>* position = vertex_attributes_.template add_attribute<VEC3>("position");

		// read vertices position
		std::vector<unsigned int> vertices_id;
		vertices_id.reserve(nb_vertices_);

		for (unsigned int i = 0; i < nb_vertices_; ++i)
		{

			double x = read_double();
			double y = read_double();
			double z = read_double();

			VEC3 pos{Scalar(x), Scalar(y), Scalar(z)};

			unsigned int vertex_id = vertex_attributes_.template insert_lines<1>();
			(*position)[vertex_id] = pos;

			vertices_id.push_back(vertex_id);
		}

		// read faces (vertex indices)
		faces_nb_edges_.reserve(nb_faces_);
		faces_vertex_indices_.reserve(nb_vertices_ * 8);
		for (unsigned int i = 0; i < nb_faces_; ++i)
		{
			unsigned int n = read_uint();
			faces_nb_edges_.push_back(n);
			for (unsigned int j = 0; j < n; ++j)
			{
				unsigned int index = read_uint();
				faces_vertex_indices_.push_back(vertices_id[index]);
			}

		}

		return true;
	}

	template <typename VEC3>
	bool import_OFF_BIN(std::ifstream& fp)
	{
		char buffer1[12];
		fp.read(buffer1,12);

		nb_vertices_= swap_endianness_system_big(*(reinterpret_cast<unsigned int*>(buffer1)));
		nb_faces_= swap_endianness_system_big(*(reinterpret_cast<unsigned int*>(buffer1+4)));
		nb_edges_= swap_endianness_system_big(*(reinterpret_cast<unsigned int*>(buffer1+8)));


		ChunkArray<VEC3>* position = vertex_attributes_.template add_attribute<VEC3>("position");


		static const unsigned int BUFFER_SZ = 1024*1024;
		float* buff_pos = new float[3*BUFFER_SZ];
		std::vector<unsigned int> vertices_id;
		vertices_id.reserve(nb_vertices_);

		{// limit scope of j
			unsigned j = BUFFER_SZ;
			for (unsigned int i = 0; i < nb_vertices_; ++i, ++j)
			{
				if (j == BUFFER_SZ)
				{
					j = 0;
					// read from file into buffer
					if (i + BUFFER_SZ < nb_vertices_)
						fp.read(reinterpret_cast<char*>(buff_pos), 3 * sizeof(float)*BUFFER_SZ);
					else
						fp.read(reinterpret_cast<char*>(buff_pos), 3 * sizeof(float)*(nb_vertices_ - i));

					//endian
					unsigned int* ptr = reinterpret_cast<unsigned int*>(buff_pos);
					for (unsigned int k = 0; k < 3 * BUFFER_SZ; ++k)
					{
						*ptr = swap_endianness_system_big(*ptr);
						++ptr;
					}
				}

				VEC3 pos{ buff_pos[3 * j], buff_pos[3 * j + 1], buff_pos[3 * j + 2] };

				unsigned int vertex_id = vertex_attributes_.template insert_lines<1>();
				(*position)[vertex_id] = pos;

				vertices_id.push_back(vertex_id);
			}
		}// j scope
		delete[] buff_pos;

		// read faces (vertex indices)

		unsigned int* buff_ind = new unsigned int[BUFFER_SZ];
		faces_nb_edges_.reserve(nb_faces_);
		faces_vertex_indices_.reserve(nb_vertices_ * 8);

		unsigned int* ptr = buff_ind;
		unsigned int nb_read = BUFFER_SZ;
		for (unsigned int i = 0; i < nb_faces_; ++i)
		{
			if (nb_read == BUFFER_SZ)
			{
				fp.read(reinterpret_cast<char*>(buff_ind),BUFFER_SZ*sizeof(unsigned int));
				ptr = buff_ind;
				for (unsigned int k=0; k< BUFFER_SZ;++k)
				{
					*ptr = swap_endianness_system_big(*ptr);
					++ptr;
				}
				ptr = buff_ind;
				nb_read =0;
			}

			unsigned int n = *ptr++;
			nb_read++;

			faces_nb_edges_.push_back(n);
			for (unsigned int j = 0; j < n; ++j)
			{
				if (nb_read == BUFFER_SZ)
				{
					fp.read(reinterpret_cast<char*>(buff_ind),BUFFER_SZ*sizeof(unsigned int));
					ptr = buff_ind;
					for (unsigned int k=0; k< BUFFER_SZ;++k)
					{
						*ptr = swap_endianness_system_big(*ptr);
						++ptr;
					}
					ptr = buff_ind;
					nb_read=0;
				}
				unsigned int index = *ptr++;
				nb_read++;
				faces_vertex_indices_.push_back(vertices_id[index]);
			}
		}

		delete[] buff_ind;

		return true;
	}


	template <typename VEC3>
	bool import_OBJ(std::ifstream& fp)
	{
		using Scalar = typename VEC3::Scalar;

		ChunkArray<VEC3>* position =
			vertex_attributes_.template add_attribute<VEC3>("position");

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

				VEC3 pos{Scalar(x), Scalar(y), Scalar(z)};

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



	template <typename VEC3>
	bool import_ply(const std::string& filename)
	{

		PlyImportData pid;

		if (! pid.read_file(filename) )
		{
			std::cerr << "Unable to open file " << filename << std::endl;
			return false;
		}

		ChunkArray<VEC3>* position = vertex_attributes_.template add_attribute<VEC3>("position");
		ChunkArray<VEC3>* color = nullptr;
		if (pid.has_colors())
		{
			 color = vertex_attributes_.template add_attribute<VEC3>("color");
		}

		nb_vertices_ = pid.nb_vertices();
		nb_faces_ = pid.nb_faces();

		
		// read vertices position
		std::vector<unsigned int> vertices_id;
		vertices_id.reserve(nb_vertices_);

		for (unsigned int i = 0; i < nb_vertices_; ++i)
		{
			VEC3 pos;
			pid.vertex_position(i, pos);

			unsigned int vertex_id = vertex_attributes_.template insert_lines<1>();
			(*position)[vertex_id] = pos;

			vertices_id.push_back(vertex_id);

			if (pid.has_colors())
			{
				VEC3 rgb;
				pid.vertex_color_float32(i, rgb);

				(*color)[vertex_id] = pos;
			}
		}

		// read faces (vertex indices)
		faces_nb_edges_.reserve(nb_faces_);
		faces_vertex_indices_.reserve(nb_vertices_ * 8);
		for (unsigned int i = 0; i < nb_faces_; ++i)
		{
			unsigned int n = pid.get_face_valence(i);
			faces_nb_edges_.push_back(n);
			int* indices = pid.get_face_indices(i);
			for (unsigned int j = 0; j < n; ++j)
			{
				unsigned int index = (unsigned int)(indices[j]);
				faces_vertex_indices_.push_back(vertices_id[index]);
			}
		}

		return true;
	}


};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_SURFACE_IMPORT_CPP_))
extern template class CGOGN_IO_API SurfaceImport<DefaultMapTraits>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_SURFACE_IMPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // IO_SURFACE_IMPORT_H_

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
#include <memory>
#include <cstdint>

#include <core/utils/endian.h>
#include <core/utils/name_types.h>
#include <core/container/chunk_array_container.h>
#include <core/cmap/cmap2.h>
#include <core/cmap/cmap2_builder.h>
#include <core/utils/string.h>

#include <geometry/types/geometry_traits.h>

#include <io/import_ply_data.h>
#include <io/vtk_cell_types.h>

#include <io/dll.h>

namespace cgogn
{

namespace io
{

inline std::string vtk_data_type_to_cgogn_name_of_type(const std::string& vtk_type_str)
{
	const std::string& data_type = to_lower(vtk_type_str);
	if (data_type == "char" || data_type == "int8")
		return name_of_type(std::int8_t());
	if (data_type == "unsigned_char" || data_type == "uint8")
		return name_of_type(std::uint8_t());
	if (data_type == "short" || data_type == "int16")
		return name_of_type(std::int16_t());
	if (data_type == "unsigned_short" || data_type == "uint16")
		return name_of_type(std::uint16_t());
	if (data_type == "int" || data_type == "int32")
		return name_of_type(std::int32_t());
	if (data_type == "unsigned_int" || data_type == "uint32")
		return name_of_type(std::uint32_t());
	if (data_type == "long" || data_type == "int64")
		return name_of_type(std::int64_t());
	if (data_type == "unsigned_long" || data_type == "uint64")
		return name_of_type(std::uint64_t());
	if (data_type == "float"  || data_type == "float32")
		return name_of_type(float());
	if (data_type == "double" || data_type == "float64")
		return name_of_type(double());

	std::cerr << "vtk_data_type_to_cgogn_name_of_type : unknown vtk type : " << vtk_type_str << std::endl;
	return std::string();
}


template<typename T = double, typename U = T>
inline std::unique_ptr<std::vector<T>> read_n_scalars(std::ifstream& fp, std::size_t n, bool binary, bool big_endian)
{
	using VecT = std::vector<T>;
	using VecU = std::vector<U>;

	std::cerr << "read_n_scalars called with T = " << name_of_type(T()) << " and U = " << name_of_type(U()) << std::endl;
	std::unique_ptr<VecT> res = make_unique<VecT>();
	res->reserve(n);

	std::unique_ptr<VecU>  buffer = make_unique<VecU>(n);

	if (binary)
	{
		fp.read(reinterpret_cast<char*>(std::addressof(buffer->operator[](0))), n * sizeof(U));

		if (big_endian != internal::cgogn_is_little_endian)
		{
			for (auto & x : *buffer)
				x = swap_endianness(x);
		}
		if (fp.eof() || fp.bad())
			buffer->clear();

	} else {
		std::string line;
		line.reserve(256);
		std::size_t i = 0ul;
		for (; i < n && (!fp.eof()) && (!fp.bad()); )
		{
			std::getline(fp,line);
			std::istringstream line_stream(line);
			while (i < n && (line_stream >> buffer->operator[](i)))
				++i;
		}

		if (i < n)
			buffer->clear();
	}

	if (std::is_same<T,U>::value)
		res.reset(reinterpret_cast<VecT*>(buffer.release()));
	else
	{
		for (auto buffer_it = buffer->begin(), end = buffer->end(); buffer_it != end; ++buffer_it)
			res->push_back(*buffer_it);
	}

	return res;
}

template<typename T = double>
inline std::unique_ptr<std::vector<T>> read_n_scalars(std::ifstream& fp, const std::string& type_name, std::size_t n, bool binary, bool big_endian)
{
	using VecT = std::vector<T>;
	std::unique_ptr<std::vector<T>> res;
	std::cerr << "read_n_scalars called with type " << type_name << std::endl;
	const std::string& type = vtk_data_type_to_cgogn_name_of_type(type_name);

	if (type == name_of_type(float()))
	{
		std::unique_ptr<std::vector<T>> scalars(std::move(read_n_scalars<T,float>(fp,n,binary,big_endian)));
		res.reset(reinterpret_cast<VecT*>(scalars.release()));
	}
	if (type == name_of_type(double()))
	{
		std::unique_ptr<std::vector<T>> scalars(std::move(read_n_scalars<T,double>(fp,n,binary,big_endian)));
		res.reset(reinterpret_cast<VecT*>(scalars.release()));
	}

	if (type == name_of_type(std::uint32_t()))
	{
		std::unique_ptr<std::vector<T>> scalars(std::move(read_n_scalars<T, std::uint32_t>(fp,n,binary,big_endian)));
		res.reset(reinterpret_cast<VecT*>(scalars.release()));
	}

	return res;
}

template<typename Vec_T>
inline std::unique_ptr<std::vector<Vec_T>> read_n_vec(std::ifstream& fp, const std::string& type_name, std::size_t n, bool binary, bool big_endian)
{
	using Scalar = typename geometry::vector_traits<Vec_T>::Scalar;
	const std::size_t size = geometry::vector_traits<Vec_T>::SIZE;
	std::unique_ptr<std::vector<Scalar>> scalars(std::move(read_n_scalars(fp, type_name,n*size,binary,big_endian)));
	std::unique_ptr<std::vector<Vec_T>> res = make_unique<std::vector<Vec_T>>();
	res->reserve(n);
	for (auto it = scalars->begin(), end = scalars->end() ; it != end;)
	{
		res->emplace_back(Scalar(*it++), Scalar(*it++), Scalar(*it++));
	}

	return res;
}


enum SurfaceFileType
{
	SurfaceFileType_UNKNOWN = 0,
	SurfaceFileType_OFF,
	SurfaceFileType_OBJ,
	SurfaceFileType_PLY,
	SurfaceFileType_VTK_LEGACY
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
	if (extension == "vtk")
		return SurfaceFileType_VTK_LEGACY;
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

	template <typename VEC3>
	bool import_file(const std::string& filename)
	{
		return import_file<VEC3>(filename, get_file_type(filename));
	}

	template <typename VEC3>
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
				result = import_OFF<VEC3>(fp);
				break;
			case SurfaceFileType_OBJ :
				result = import_OBJ<VEC3>(fp);
				break;
			case SurfaceFileType_VTK_LEGACY :
				result = import_vtk_legacy<VEC3>(fp);
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

		unsigned j = BUFFER_SZ;
		for (unsigned int i = 0; i < nb_vertices_; ++i,++j)
		{
			if (j == BUFFER_SZ)
			{
				j = 0;
				// read from file into buffer
				if (i+BUFFER_SZ < nb_vertices_)
					fp.read(reinterpret_cast<char*>(buff_pos),3*sizeof(float)*BUFFER_SZ);
				else
					fp.read(reinterpret_cast<char*>(buff_pos),3*sizeof(float)*(nb_vertices_-i));

				//endian
				unsigned int* ptr = reinterpret_cast<unsigned int*>(buff_pos);
				for (unsigned int i=0; i< 3*BUFFER_SZ;++i)
				{
					*ptr = swap_endianness_system_big(*ptr);
					++ptr;
				}
			}

			VEC3 pos{buff_pos[3*j], buff_pos[3*j+1], buff_pos[3*j+2]};

			unsigned int vertex_id = vertex_attributes_.template insert_lines<1>();
			(*position)[vertex_id] = pos;

			vertices_id.push_back(vertex_id);
		}

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
				for (unsigned int i=0; i< BUFFER_SZ;++i)
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
					for (unsigned int i=0; i< BUFFER_SZ;++i)
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
		ChunkArray<VEC3>* color;
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

	template <typename VEC3>
	bool import_vtk_legacy(std::ifstream& fp)
	{
		enum VTK_TYPE
		{
			UNKNOWN = 0,
			UNSTRUCTURED_GRID,
			POLYDATA
		};

		VTK_TYPE vtk_type(VTK_TYPE::UNKNOWN);

		std::cerr << "Opening a vtk file" << std::endl;
		using Scalar = typename VEC3::Scalar;

		std::string line;
		std::string word;
		line.reserve(512);
		word.reserve(128);

		// printing the 2 first lines
		std::getline(fp, line);
		std::cout << line << std::endl;
		std::getline(fp, line);
		std::cout << line << std::endl;

		fp >> word;
		bool ascii_file = to_upper(word) == "ASCII";
		cgogn_assert(ascii_file || to_upper(word) == "BINARY");

		fp >> word;
		cgogn_assert(to_upper(word) == "DATASET");
		fp >> word;
		const std::string& dataset = to_upper(word);
		if (dataset == "UNSTRUCTURED_GRID")
			vtk_type = VTK_TYPE::UNSTRUCTURED_GRID;
		else {
			if (dataset == "POLYDATA")
				vtk_type = VTK_TYPE::POLYDATA;
		}

		std::unique_ptr<std::vector<unsigned int>> cells;
		std::unique_ptr<std::vector<unsigned int>> cell_types;
		ChunkArray<VEC3>* position = vertex_attributes_.template add_attribute<VEC3>("position");
		std::vector<unsigned int> verticesID;

		if (vtk_type == VTK_TYPE::UNSTRUCTURED_GRID)
		{
			while(!fp.eof())
			{
				std::getline(fp,line);
				word.clear();
				std::istringstream sstream(line);
				sstream >> word;
				if (to_upper(word) == "POINTS")
				{
					std::string type_str;
					sstream >> this->nb_vertices_ >> type_str;
					type_str = to_lower(type_str);
					std::cout << nb_vertices_ << " points" << " of type " << type_str << std::endl;
					verticesID.reserve(nb_vertices_);
					std::unique_ptr<std::vector<Scalar>> pos(std::move(read_n_scalars<Scalar>(fp, type_str, 3*nb_vertices_, !ascii_file, false /*don't deal with endianness yet*/)));
					cgogn_assert(pos);
					for (std::size_t i = 0ul ; i < 3ul*nb_vertices_ ; i+=3ul)
					{
						VEC3 P(Scalar((*pos)[i]), Scalar((*pos)[i+1ul]), Scalar((*pos)[i+2ul]));
						std::cout << P[0] << " " << P[1] << " " << P[2] << std::endl;
						unsigned int id = vertex_attributes_.template insert_lines<1>();
						position->operator [](id) = P;
						verticesID.push_back(id);
					}
				}

				if (to_upper(word) == "CELLS")
				{
					std::cerr << line << std::endl;
					unsigned int size;
					sstream >> this->nb_faces_ >> size;
					std::cerr << "nb cells " << nb_faces_ << " and size " << size << std::endl;
					cells = std::move(read_n_scalars<unsigned int>(fp, "unsigned_int", size, !ascii_file, false));
					std::size_t i = 0ul;
				}

				if (to_upper(word) == "CELL_TYPES")
				{
					std::cerr << line << std::endl;
					unsigned int nbc;
					sstream >> nbc;
					std::cerr << "nb cells " << nbc << std::endl;
					cell_types = std::move(read_n_scalars<unsigned int>(fp, "unsigned_int", nbc, !ascii_file, false));
				}
			}
		}

		auto cell_type_it = cell_types->begin();
		for (auto cell_it  = cells->begin(), end = cells->end(); cell_it != end ; ++cell_type_it)
		{
			const std::size_t nb_vert = *cell_it++;

			if ((*cell_type_it) != VTK_CELL_TYPES::VTK_TRIANGLE_STRIP)
			{
				faces_nb_edges_.push_back(nb_vert);
				for (std::size_t i = 0ul ; i < nb_vert;++i)
				{
					faces_vertex_indices_.push_back(*cell_it++);
				}
			} else {
				std::vector<unsigned int> vertexIDS;
				vertexIDS.reserve(nb_vert);
				for (std::size_t i = 0ul ; i < nb_vert;++i)
				{
					vertexIDS.push_back(*cell_it++);
				}

				for (unsigned int i = 0u ; i < nb_vert -2u; ++i)
				{
					if (i != 0u)
						++nb_faces_;
					faces_nb_edges_.push_back(3);
					faces_vertex_indices_.push_back(vertexIDS[i]);
					faces_vertex_indices_.push_back(vertexIDS[i+1]);
					faces_vertex_indices_.push_back(vertexIDS[i+2]);
					if (i % 2u == 0u)
						std::swap(faces_vertex_indices_.back(), *(faces_vertex_indices_.end()-2));
				}
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

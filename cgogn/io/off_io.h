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

#ifndef IO_OFF_IO_H_
#define IO_OFF_IO_H_

#include <io/surface_import.h>

namespace cgogn
{

namespace io
{

template<typename MAP_TRAITS, typename VEC3>
class OffSurfaceImport : public SurfaceImport<MAP_TRAITS> {
public:
	using Self = OffSurfaceImport<MAP_TRAITS, VEC3>;
	using Inherit = SurfaceImport<MAP_TRAITS>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	virtual ~OffSurfaceImport() override {}
protected:
	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename.c_str(), std::ios::in);

		std::string line;
		line.reserve(512);

		// read OFF header
		std::getline(fp, line);
		if (line.rfind("OFF") == std::string::npos)
		{
			std::cerr << "Problem reading off file: not an off file" << std::endl;
			std::cerr << line << std::endl;
			return false;
		}

		// check if binary file
		if (line.rfind("BINARY") != std::string::npos)
		{
			return this->import_off_bin(fp);
		}


		// read number of vertices, edges, faces
		this->nb_vertices_ = this->read_uint(fp,line);
		this->nb_faces_ = this->read_uint(fp,line);
		this->nb_edges_ = this->read_uint(fp,line);

		ChunkArray<VEC3>* position = this->vertex_attributes_.template add_attribute<VEC3>("position");

		// read vertices position
		std::vector<unsigned int> vertices_id;
		vertices_id.reserve(this->nb_vertices_);

		for (unsigned int i = 0; i < this->nb_vertices_; ++i)
		{

			double x = this->read_double(fp,line);
			double y = this->read_double(fp,line);
			double z = this->read_double(fp,line);

			VEC3 pos{Scalar(x), Scalar(y), Scalar(z)};

			unsigned int vertex_id = this->vertex_attributes_.template insert_lines<1>();
			(*position)[vertex_id] = pos;

			vertices_id.push_back(vertex_id);
		}

		// read faces (vertex indices)
		this->faces_nb_edges_.reserve(this->nb_faces_);
		this->faces_vertex_indices_.reserve(this->nb_vertices_ * 8);
		for (unsigned int i = 0; i < this->nb_faces_; ++i)
		{
			unsigned int n = this->read_uint(fp,line);
			this->faces_nb_edges_.push_back(n);
			for (unsigned int j = 0; j < n; ++j)
			{
				unsigned int index = this->read_uint(fp,line);
				this->faces_vertex_indices_.push_back(vertices_id[index]);
			}

		}

		return true;
	}

	inline bool import_off_bin(std::istream& fp)
	{
		char buffer1[12];
		fp.read(buffer1,12);

		this->nb_vertices_= swap_endianness_native_big(*(reinterpret_cast<unsigned int*>(buffer1)));
		this->nb_faces_= swap_endianness_native_big(*(reinterpret_cast<unsigned int*>(buffer1+4)));
		this->nb_edges_= swap_endianness_native_big(*(reinterpret_cast<unsigned int*>(buffer1+8)));


		ChunkArray<VEC3>* position = this->vertex_attributes_.template add_attribute<VEC3>("position");


		static const unsigned int BUFFER_SZ = 1024*1024;
		float* buff_pos = new float[3*BUFFER_SZ];
		std::vector<unsigned int> vertices_id;
		vertices_id.reserve(this->nb_vertices_);

		unsigned j = BUFFER_SZ;
		for (unsigned int i = 0; i < this->nb_vertices_; ++i,++j)
		{
			if (j == BUFFER_SZ)
			{
				j = 0;
				// read from file into buffer
				if (i+BUFFER_SZ < this->nb_vertices_)
					fp.read(reinterpret_cast<char*>(buff_pos),3*sizeof(float)*BUFFER_SZ);
				else
					fp.read(reinterpret_cast<char*>(buff_pos),3*sizeof(float)*(this->nb_vertices_-i));

				//endian
				unsigned int* ptr = reinterpret_cast<unsigned int*>(buff_pos);
				for (unsigned int k=0; k< 3*BUFFER_SZ;++k)
				{
					*ptr = swap_endianness_native_big(*ptr);
					++ptr;
				}
			}

			VEC3 pos{buff_pos[3*j], buff_pos[3*j+1], buff_pos[3*j+2]};

			unsigned int vertex_id = this->vertex_attributes_.template insert_lines<1>();
			(*position)[vertex_id] = pos;

			vertices_id.push_back(vertex_id);
		}

		delete[] buff_pos;

		// read faces (vertex indices)

		unsigned int* buff_ind = new unsigned int[BUFFER_SZ];
		this->faces_nb_edges_.reserve(this->nb_faces_);
		this->faces_vertex_indices_.reserve(this->nb_vertices_ * 8);

		unsigned int* ptr = buff_ind;
		unsigned int nb_read = BUFFER_SZ;
		for (unsigned int i = 0; i < this->nb_faces_; ++i)
		{
			if (nb_read == BUFFER_SZ)
			{
				fp.read(reinterpret_cast<char*>(buff_ind),BUFFER_SZ*sizeof(unsigned int));
				ptr = buff_ind;
				for (unsigned int i=0; i< BUFFER_SZ;++i)
				{
					*ptr = swap_endianness_native_big(*ptr);
					++ptr;
				}
				ptr = buff_ind;
				nb_read =0;
			}

			unsigned int n = *ptr++;
			nb_read++;

			this->faces_nb_edges_.push_back(n);
			for (unsigned int j = 0; j < n; ++j)
			{
				if (nb_read == BUFFER_SZ)
				{
					fp.read(reinterpret_cast<char*>(buff_ind),BUFFER_SZ*sizeof(unsigned int));
					ptr = buff_ind;
					for (unsigned int k=0; k< BUFFER_SZ;++k)
					{
						*ptr = swap_endianness_native_big(*ptr);
						++ptr;
					}
					ptr = buff_ind;
					nb_read=0;
				}
				unsigned int index = *ptr++;
				nb_read++;
				this->faces_vertex_indices_.push_back(vertices_id[index]);
			}
		}

		delete[] buff_ind;

		return true;
	}
private:
	static inline double read_double(std::istream& fp, std::string& line)
	{
		fp >> line;
		while (line[0]=='#')
		{
			fp.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			fp >> line;
		}
		return std::stod(line);
	}

	static inline unsigned int read_uint(std::istream& fp, std::string& line)
	{
		fp >> line;
		while (line[0]=='#')
		{
			fp.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			fp >> line;
		}
		return static_cast<unsigned int>((std::stoul(line)));
	}
};

} // namespace io
} // namespace cgogn

#endif // IO_OFF_IO_H_

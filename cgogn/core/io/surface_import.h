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

CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_ON
#include <Eigen/Dense>
CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_OFF

namespace cgogn
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

template <typename DATA_TRAITS>
class SurfaceImport
{
public:

	unsigned int nb_vertices_;
	unsigned int nb_edges_;
	unsigned int nb_faces_;

	std::vector<unsigned short> faces_nb_edges_;
	std::vector<unsigned int> faces_vertex_indices_;

	ChunkArrayContainer<DATA_TRAITS::CHUNK_SIZE, unsigned int> vertex_attributes_;

	SurfaceImport()
	{}

	~SurfaceImport()
	{}

	void clear()
	{
		nb_vertices_ = 0;
		nb_edges_ = 0;
		nb_faces_ = 0;
		faces_nb_edges_.clear();
		faces_vertex_indices_.clear();
	}

	bool import_file(const std::string& filename)
	{
		return import_file(filename, get_file_type(filename));
	}

	bool import_file(const std::string& filename, SurfaceFileType type)
	{
		std::ifstream fp(filename.c_str(), std::ios::in);
		if (!fp.good())
		{
			std::cout << "Unable to open file " << filename << std::endl;
			return false;
		}

		clear();

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

		fp.close();

		return result;
	}

protected:

	bool import_OFF(std::ifstream& fp)
	{
		typedef Eigen::Vector3d VEC3;

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
		ChunkArray<DATA_TRAITS::CHUNK_SIZE, VEC3>* position =
			vertex_attributes_.template add_attribute<VEC3>("position");

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

			float x, y, z;
			oss >> x;
			oss >> y;
			oss >> z;

			VEC3 pos(x, y, z);

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
		typedef Eigen::Vector3d VEC3;

		ChunkArray<DATA_TRAITS::CHUNK_SIZE, VEC3>* position =
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

				float x, y, z;
				oss >> x;
				oss >> y;
				oss >> z;

				VEC3 pos(x, y, z);

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

} // namespace cgogn

#endif // CORE_IO_SURFACE_IMPORT_H_

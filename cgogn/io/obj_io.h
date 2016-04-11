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

#ifndef CGOGN_IO_OBJ_IO_H_
#define CGOGN_IO_OBJ_IO_H_

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/surface_import.h>

namespace cgogn
{

namespace io
{

template<typename MAP_TRAITS, typename VEC3>
class ObjSurfaceImport : public SurfaceImport<MAP_TRAITS> {
public:
	using Self = ObjSurfaceImport<MAP_TRAITS, VEC3>;
	using Inherit = SurfaceImport<MAP_TRAITS>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline ObjSurfaceImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ObjSurfaceImport);
	virtual ~ObjSurfaceImport() override {}
protected:
	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename.c_str(), std::ios::in);
		ChunkArray<VEC3>* position =
				this->vertex_attributes_.template add_attribute<VEC3>("position");

		std::string line, tag;

		do
		{
			fp >> tag;
			std::getline(fp, line);
		} while (tag != std::string("v"));

		// lecture des sommets
		std::vector<uint32> vertices_id;
		vertices_id.reserve(102400);

		uint32 i = 0;
		do
		{
			if (tag == std::string("v"))
			{
				std::stringstream oss(line);

				float64 x, y, z;
				oss >> x;
				oss >> y;
				oss >> z;

				VEC3 pos{Scalar(x), Scalar(y), Scalar(z)};

				uint32 vertex_id = this->vertex_attributes_.template insert_lines<1>();
				(*position)[vertex_id] = pos;

				vertices_id.push_back(vertex_id);
				i++;
			}

			fp >> tag;
			std::getline(fp, line);
		} while (!fp.eof());

		this->nb_vertices_ = uint32(vertices_id.size());

		fp.clear();
		fp.seekg(0, std::ios::beg);

		do
		{
			fp >> tag;
			std::getline(fp, line);
		} while (tag != std::string("f"));

		this->faces_nb_edges_.reserve(vertices_id.size() * 2);
		this->faces_vertex_indices_.reserve(vertices_id.size() * 8);

		std::vector<uint32> table;
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

					uint32 ind = 0;

					while ((ind < str.length()) && (str[ind] != '/'))
						ind++;

					if (ind > 0)
					{
						uint32 index;
						std::stringstream iss(str.substr(0, ind));
						iss >> index;
						table.push_back(index);
					}
				}

				uint32 n = uint32(table.size());
				this->faces_nb_edges_.push_back(n);
				for (uint32 j = 0; j < n; ++j)
				{
					uint32 index = table[j] - 1; // indices start at 1
					this->faces_vertex_indices_.push_back(vertices_id[index]);
				}
				this->nb_faces_++;
			}
			fp >> tag;
			std::getline(fp, line);
		} while (!fp.eof());

		return true;
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_OBJ_IO_CPP_))
extern template class CGOGN_IO_API ObjSurfaceImport<DefaultMapTraits, Eigen::Vector3d>;
extern template class CGOGN_IO_API ObjSurfaceImport<DefaultMapTraits, Eigen::Vector3f>;
extern template class CGOGN_IO_API ObjSurfaceImport<DefaultMapTraits, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API ObjSurfaceImport<DefaultMapTraits, geometry::Vec_T<std::array<float32,3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_OBJ_IO_H_))

} // namespace io
} // namespace cgogn

#endif // CGOGN_IO_OBJ_IO_H_

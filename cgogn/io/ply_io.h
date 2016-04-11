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

#ifndef CGOGN_IO_PLY_IO_H_
#define CGOGN_IO_PLY_IO_H_

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/surface_import.h>
#include <cgogn/io/import_ply_data.h>

namespace cgogn
{

namespace io
{

template<typename MAP_TRAITS, typename VEC3>
class PlySurfaceImport : public SurfaceImport<MAP_TRAITS> {
public:
	using Self = PlySurfaceImport<MAP_TRAITS, VEC3>;
	using Inherit = SurfaceImport<MAP_TRAITS>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline PlySurfaceImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(PlySurfaceImport);
	virtual ~PlySurfaceImport() override {}

protected:
	virtual bool import_file_impl(const std::string& filename) override
	{

		PlyImportData pid;

		if (! pid.read_file(filename) )
		{
			cgogn_log_error("PlySurfaceImport::import_file_impl") << "Unable to open the file \"" << filename << "\".";
			return false;
		}

		ChunkArray<VEC3>* position = this->vertex_attributes_.template add_attribute<VEC3>("position");
		ChunkArray<VEC3>* color = nullptr;
		if (pid.has_colors())
		{
			color = this->vertex_attributes_.template add_attribute<VEC3>("color");
		}

		this->nb_vertices_ = pid.nb_vertices();
		this->nb_faces_ = pid.nb_faces();


		// read vertices position
		std::vector<uint32> vertices_id;
		vertices_id.reserve(this->nb_vertices_);

		for (uint32 i = 0; i < this->nb_vertices_; ++i)
		{
			VEC3 pos;
			pid.vertex_position(i, pos);

			uint32 vertex_id = this->vertex_attributes_.template insert_lines<1>();
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
		this->faces_nb_edges_.reserve(this->nb_faces_);
		this->faces_vertex_indices_.reserve(this->nb_vertices_ * 8);
		for (uint32 i = 0; i < this->nb_faces_; ++i)
		{
			uint32 n = pid.get_face_valence(i);
			this->faces_nb_edges_.push_back(n);
			int* indices = pid.get_face_indices(i);
			for (uint32 j = 0; j < n; ++j)
			{
				uint32 index = (uint32)(indices[j]);
				this->faces_vertex_indices_.push_back(vertices_id[index]);
			}
		}

		return true;
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_PLY_IO_CPP_))
extern template class CGOGN_IO_API PlySurfaceImport<DefaultMapTraits, Eigen::Vector3d>;
extern template class CGOGN_IO_API PlySurfaceImport<DefaultMapTraits, Eigen::Vector3f>;
extern template class CGOGN_IO_API PlySurfaceImport<DefaultMapTraits, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API PlySurfaceImport<DefaultMapTraits, geometry::Vec_T<std::array<float32,3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_PLY_IO_CPP_))

}// namespace io
} // namespace cgogn
#endif // CGOGN_IO_PLY_IO_H_

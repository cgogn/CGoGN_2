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

#ifndef CGOGN_IO_FORMATS_PLO_H_
#define CGOGN_IO_FORMATS_PLO_H_

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/point_set_import.h>

namespace cgogn
{

namespace io
{

template <typename VEC3>
class PlotPointSetImport : public PointSetFileImport
{
public:
	using Self = PlotPointSetImport<VEC3>;
	using Inherit = PointSetFileImport;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline PlotPointSetImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(PlotPointSetImport);
	virtual ~PlotPointSetImport() override {}

protected:
	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename.c_str(), std::ios::in);

		ChunkArray<VEC3>* position = this->template add_vertex_attribute<VEC3>("position");

		std::string line;
		line.reserve(512);

		// reading number of points
		uint32 nb_vertices = 0u;
		{
			getline_safe(fp, line);
			std::istringstream iss(line);
			iss >> nb_vertices;
		}
		this->nb_vertices_ = nb_vertices;


		for (uint32 i = 0; i < nb_vertices; ++i)
		{
			getline_safe(fp, line);
			std::stringstream oss(line);

			float64 x, y, z;
			oss >> x;
			oss >> y;
			oss >> z;

			VEC3 pos{Scalar(x), Scalar(y), Scalar(z)};

			uint32 vertex_id = this->insert_line_vertex_container();
			(*position)[vertex_id] = pos;
		}

		return true;
	}
};


#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_EXTERNAL_TEMPLATES_CPP_))
extern template class CGOGN_IO_API PlotPointSetImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API PlotPointSetImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API PlotPointSetImport<geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API PlotPointSetImport<geometry::Vec_T<std::array<float32,3>>>;

#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_EXTERNAL_TEMPLATES_CPP_))

} //end namespace io

} //end namespace cgogn

#endif // CGOGN_IO_FORMATS_PLO_H_

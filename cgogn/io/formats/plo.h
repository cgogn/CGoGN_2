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
#include <cgogn/io/point_set_export.h>

namespace cgogn
{

namespace io
{

template <typename MAP, typename VEC3>
class PloPointSetImport : public PointSetFileImport<MAP>
{
public:

	using Self = PloPointSetImport<MAP, VEC3>;
	using Inherit = PointSetFileImport<MAP>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline PloPointSetImport(MAP& map) : Inherit(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(PloPointSetImport);
	virtual ~PloPointSetImport() override {}

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


template <typename MAP>
class PloPointSetExport : public PointSetExport<MAP>
{
public:
	using Inherit = PointSetExport<MAP>;
	using Self = PloPointSetExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;

	using ChunkArrayGen = typename Inherit::ChunkArrayGen;
	template <typename T>
	using VertexAttribute = typename Inherit::template VertexAttribute<T>;
	using ChunkArrayContainer = typename Inherit::ChunkArrayContainer;

protected:
	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& ) override
	{
		// Header
		output << map.template nb_cells<Vertex::ORBIT>() << std::endl;

		// Vertices
		map.foreach_cell([&] (Vertex v)
		{
			this->position_attribute(Vertex::ORBIT)->export_element(map.embedding(v), output, false, false);
			output << std::endl;
		}, *(this->cell_cache_));
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_EXTERNAL_TEMPLATES_CPP_))
extern template class CGOGN_IO_API PloPointSetImport<CMap0, Eigen::Vector3d>;
extern template class CGOGN_IO_API PloPointSetImport<CMap0, Eigen::Vector3f>;
extern template class CGOGN_IO_API PloPointSetImport<CMap0, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API PloPointSetImport<CMap0, geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API PlotPointSetExport<CMap0>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_EXTERNAL_TEMPLATES_CPP_))

} //end namespace io

} //end namespace cgogn

#endif // CGOGN_IO_FORMATS_PLO_H_

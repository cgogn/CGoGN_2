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

#ifndef CGOGN_IO_FORMATS_DOT_H_
#define CGOGN_IO_FORMATS_DOT_H_

#include <set>

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/graph_import.h>
#include <cgogn/io/graph_export.h>

#include <regex>
#include <iomanip>

namespace cgogn
{

namespace io
{

template <typename VEC3>
class DotGraphImport : public GraphFileImport
{
public:
	using Self = DotGraphImport<VEC3>;
	using Inherit = GraphFileImport;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline DotGraphImport() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(DotGraphImport);
	virtual ~DotGraphImport() override {}

protected:
	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename.c_str(), std::ios::in);

		ChunkArray<VEC3>* position = this->template add_vertex_attribute<VEC3>("position");
		ChunkArray<Scalar>* radius = this->template add_vertex_attribute<Scalar>("radius");


		std::regex re("[->;:\\s\\t\"\\n()]+"); // label\=\"("); //"[[:digit:]]+--[[:digit:]]+\\s+\\[label=

		//"label\\[(=)\\];:> \\t\"\\n\\rGrph}{g-"

		std::string line;
		line.reserve(512);

		while(getline_safe(fp, line))
		{
			if (line.find("--",0)!=std::string::npos)
			{
				std::sregex_token_iterator it(line.begin(), line.end(), re, -1);
				std::sregex_token_iterator reg_end;

				uint32 nb1 = std::atoi(it->str().c_str()); ++it;
				uint32 nb2 = std::atoi(it->str().c_str()); ++it;
				++it;

				this->edges_nb_vertices_.push_back(2);
				this->edges_vertex_indices_.push_back(nb1);
				this->edges_vertex_indices_.push_back(nb2);

				Scalar Cx = Scalar(std::atof(it->str().c_str())); ++it;
				Scalar Cy = Scalar(std::atof(it->str().c_str())); ++it;
				Scalar Cz = Scalar(std::atof(it->str().c_str())); ++it;
				++it; ++it;

				VEC3 pos{Cx, Cy, Cz};

				const uint32 vertex_id = this->insert_line_vertex_container();

				(*position)[vertex_id] = pos;
				(*radius)[vertex_id] = Scalar(std::atof(it->str().c_str()));

				//vertices_id.push_back(vertex_id);
			}
		}

		return true;
	}
};


#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_FORMATS_DOT_CPP_))
extern template class CGOGN_IO_API DotGraphImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API DotGraphImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API DotGraphImport<geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API DotGraphImport<geometry::Vec_T<std::array<float32,3>>>;

//extern template class CGOGN_IO_API DotGraphExport<UndirectedGraph>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_FORMATS_DOT_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_FORMATS_DOT_H_

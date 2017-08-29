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

#ifndef CGOGN_IO_2DM_IO_H_
#define CGOGN_IO_2DM_IO_H_

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/surface_import.h>

#include <iomanip>

namespace cgogn
{

namespace io
{

template <typename MAP, typename VEC3>
class SMS2DMSurfaceImport : public SurfaceFileImport<MAP, VEC3>
{
public:

	using Self = SMS2DMSurfaceImport<MAP, VEC3>;
	using Inherit = SurfaceFileImport<MAP, VEC3>;
	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template <typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	inline SMS2DMSurfaceImport(MAP& map) : Inherit(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(SMS2DMSurfaceImport);
	virtual ~SMS2DMSurfaceImport() override {}

protected:

	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream fp(filename.c_str(), std::ios::in);

		ChunkArray<VEC3>* position = this->position_attribute();

		std::string line, tag;

		// lecture des sommets

		do
		{
			fp >> tag;
			getline_safe(fp, line);
		} while (tag != std::string("ND"));

		std::vector<uint32> vertices_id;
		vertices_id.reserve(65536);

		uint32 i = 0;
		do
		{
			if (tag == std::string("ND"))
			{
				std::stringstream oss(line);

				uint32 id;
				float64 x, y, z;
				oss >> id;
				oss >> x;
				oss >> y;
				oss >> z;

				VEC3 pos{Scalar(x), Scalar(y), Scalar(z)};

				uint32 vertex_id = this->insert_line_vertex_container();
				(*position)[vertex_id] = pos;

				vertices_id.push_back(vertex_id);
				i++;
			}

			fp >> tag;
			getline_safe(fp, line);
		} while (!fp.eof());

        this->faces_nb_edges_.reserve(vertices_id.size() * 2);
        this->faces_vertex_indices_.reserve(vertices_id.size() * 8);

        // lecture des faces TRI

        fp.clear();
        fp.seekg(0, std::ios::beg);

        do
        {
            fp >> tag;
            getline_safe(fp, line);
        } while (tag != std::string("E3T") && (!fp.eof()));

        if (tag == "E3T")
        {
            do
            {
                if (tag == std::string("E3T")) // lecture d'une face
                {
                    std::stringstream oss(line);

                    uint32 id, v1, v2, v3, matid;
                    oss >> id;
                    oss >> v1;
                    oss >> v2;
                    oss >> v3;
                    oss >> matid;

                    this->faces_nb_edges_.push_back(3);
                    this->faces_vertex_indices_.push_back(vertices_id[v1-1]);
                    this->faces_vertex_indices_.push_back(vertices_id[v2-1]);
                    this->faces_vertex_indices_.push_back(vertices_id[v3-1]);
                }

                fp >> tag;
                getline_safe(fp, line);
            } while (!fp.eof());
        }

        // lecture des faces QUAD

        fp.clear();
        fp.seekg(0, std::ios::beg);

        do
        {
            fp >> tag;
            getline_safe(fp, line);
        } while (tag != std::string("E4Q") && (!fp.eof()));

        if (tag == "E4Q")
        {
            do
            {
                if (tag == std::string("E4Q")) // lecture d'une face
                {
                    std::stringstream oss(line);

                    uint32 id, v1, v2, v3, v4, matid;
                    oss >> id;
                    oss >> v1;
                    oss >> v2;
                    oss >> v3;
                    oss >> v4;
                    oss >> matid;

                    this->faces_nb_edges_.push_back(4);
                    this->faces_vertex_indices_.push_back(vertices_id[v1-1]);
                    this->faces_vertex_indices_.push_back(vertices_id[v2-1]);
                    this->faces_vertex_indices_.push_back(vertices_id[v3-1]);
                    this->faces_vertex_indices_.push_back(vertices_id[v4-1]);
                }

                fp >> tag;
                getline_safe(fp, line);
            } while (!fp.eof());
        }

		return true;
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_2DM_IO_CPP_))
extern template class CGOGN_IO_API SMS2DMSurfaceImport<CMap2, Eigen::Vector3d>;
extern template class CGOGN_IO_API SMS2DMSurfaceImport<CMap2, Eigen::Vector3f>;
extern template class CGOGN_IO_API SMS2DMSurfaceImport<CMap2, geometry::Vec_T<std::array<float64, 3>>>;
extern template class CGOGN_IO_API SMS2DMSurfaceImport<CMap2, geometry::Vec_T<std::array<float32, 3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_2DM_IO_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_2DM_IO_H_

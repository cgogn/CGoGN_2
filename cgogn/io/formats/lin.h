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

#ifndef CGOGN_IO_FORMATS_LIN_H_
#define CGOGN_IO_FORMATS_LIN_H_

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/polyline_import.h>
//#include <cgogn/io/point_set_export.h>

namespace cgogn
{

namespace io
{

template <typename MAP, typename VEC3>
class LinPolylineImport : public PolylineFileImport<MAP>
{
public:

    using Self = LinPolylineImport<MAP, VEC3>;
	using Inherit = PolylineFileImport<MAP>;
    using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
    template <typename T>
    using ChunkArray = typename Inherit::template ChunkArray<T>;

    inline LinPolylineImport(MAP& map) : Inherit(map) {}
    CGOGN_NOT_COPYABLE_NOR_MOVABLE(LinPolylineImport);
    virtual ~LinPolylineImport() override {}

protected:
    virtual bool import_file_impl(const std::string& filename) override
    {
        std::ifstream fp(filename.c_str(), std::ios::in);

        ChunkArray<VEC3>* position = this->template add_vertex_attribute<VEC3>("position");

        std::string line, tag;

		//todo hypthese sur l'ordre de lecture
        while(!fp.eof())
        {
            getline_safe(fp, line);
			fp >> tag ;

            if(tag == "v")
            {
                std::stringstream oss(line);

                float64 x, y, z;
                oss >> x;
                oss >> y;
                oss >> z;
                VEC3 pos{Scalar(x), Scalar(y), Scalar(z)};

                uint32 vertex_id = this->insert_line_vertex_container();
                (*position)[vertex_id] = pos;
            }
			else if(tag == "s")
            {
                std::stringstream oss(line);

                uint32 p1, p2;
                oss >> p1 >> p2 ;
                p1--;
                p2--;

                bool ok = true ;
//				if(p1 < 0 || p1 >= nb_vertices)
//                {
//                    cgogn_log_warning("Lin import") << p1 << " index out of bounds";
//                    ok = false ;
//                }
//				if(p2 < 0 || p2 >= nb_vertices)
//                {
//                    cgogn_log_warning("Lin import") << p2 << " index out of bounds";
//                    ok = false ;
//                }

                if(ok)
                {
                    this->edges_vertex_indices_.push_back(p1);
                    this->edges_vertex_indices_.push_back(p2);
                }
            }
        }
    }
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_EXTERNAL_TEMPLATES_CPP_))
extern template class CGOGN_IO_API LinPolylineImport<CMap1, Eigen::Vector3d>;
extern template class CGOGN_IO_API LinPolylineImport<CMap1, Eigen::Vector3f>;
extern template class CGOGN_IO_API LinPolylineImport<CMap1, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API LinPolylineImport<CMap1, geometry::Vec_T<std::array<float32,3>>>;

//extern template class CGOGN_IO_API PlotPointSetExport<CMap0>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_EXTERNAL_TEMPLATES_CPP_))

} //end namespace io

} //end namespace cgogn

#endif // CGOGN_IO_FORMATS_LIN_H_

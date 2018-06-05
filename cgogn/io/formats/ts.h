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

#ifndef CGOGN_IO_FORMATS_TS_H_
#define CGOGN_IO_FORMATS_TS_H_

#include <cgogn/core/cmap/cmap2.h>

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <cgogn/io/surface_import.h>
#include <cgogn/io/surface_export.h>

#include <iomanip>

namespace cgogn
{

namespace io
{

template <typename MAP, typename VEC3>
class TsSurfaceImport : public SurfaceFileImport<MAP>
{
public:

    using Self = TsSurfaceImport<MAP, VEC3>;
    using Inherit = SurfaceFileImport<MAP>;
    using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
    template <typename T>
    using ChunkArray = typename Inherit::template ChunkArray<T>;

    inline TsSurfaceImport(MAP& map) : Inherit(map) {}
    CGOGN_NOT_COPYABLE_NOR_MOVABLE(TsSurfaceImport);
    virtual ~TsSurfaceImport() override {}

protected:

    virtual bool import_file_impl(const std::string& filename) override
    {
        std::ifstream fp(filename.c_str(), std::ios::in);

        std::string line, tag;
        line.reserve(512);

        getline_safe(fp, line);

        if(line.compare(0, 11, "GOCAD TSurf") != 0)
            return false;

        //read the header
        {
            getline_safe(fp, line);

            //header block
            if (line == "HEADER{")
            {
                getline_safe(fp,line);
                while(line.compare(0, 1, "}") != 0)
                {
                    //todo handle properties
                    getline_safe(fp,line);
                }

            }

            //header lines
            fp >> tag;
            while(tag.compare("HDR") == 0)
            {
                //todo handle properties
                getline_safe(fp,line);
                fp >> tag;
            }
        }

        //read the coordinate system
        getline_safe(fp, line);
        if(line == "GOCAD_ORIGINAL_COORDINATE_SYSTEM")
        {
            getline_safe(fp, line);
            while(line != "END_ORIGINAL_COORDINATE_SYSTEM")
            {
                fp >> tag;

                if(tag == "NAME")
                {
                    std::cout <<"NAME: " << line << std::endl;
                }
                else if(tag == "AXIS_NAME")
                {
                    std::cout <<"AXIS_NAME: " << line << std::endl;
                }
                else if(tag == "AXIS_UNIT")
                {
                    std::cout <<"AXIS_UNIT: " << line << std::endl;
                }
                else if(tag == "ZPOSITIVE")
                {
                    std::cout <<"ZPOSITIVE: " << line << std::endl;
                }

                getline_safe(fp, line);
            }
        }

        //read the points
        std::vector<uint32> vertices_id;
        vertices_id.reserve(102400);
        uint32 max_id = 0;

        uint32 i = 0;
        do
        {
            if (tag == std::string("VRTX") || tag == std::string("PVRTX"))
            {
                std::stringstream oss(line);

                float64 x, y, z;
                oss >> x;
                oss >> y;
                oss >> z;

                VEC3 pos{Scalar(x), Scalar(y), Scalar(z)};

                uint32 vertex_id = this->insert_line_vertex_container();
                (*position)[vertex_id] = pos;

                vertices_id.push_back(vertex_id);

                if (vertex_id > max_id)
                    max_id = vertex_id;
                i++;
            }

            fp >> tag;
            getline_safe(fp, line);
        } while (!fp.eof());

        //read the triangles
        do
        {

            if (tag == std::string("TRGL"))
            {
                std::istringstream iss(line);

                std::array<uint32,3> ids;
                iss >> ids[0] >> ids[1] >> ids[2];

                this->faces_nb_edges_.push_back(3);
                for(uint32 j = 0; j < 3; j++)
                {
                    this->faces_vertex_indices_.push_back(ids[j]);
                }
            }
            fp >> tag;
            getline_safe(fp, line);
        } while (!fp.eof());

        return true;
    }
};

template <typename MAP>
class TsSurfaceExport : public SurfaceExport<MAP>
{
public:
    using Inherit = SurfaceExport<MAP>;
    using Self = TsSurfaceExport<MAP>;
    using Map = typename Inherit::Map;
    using Vertex = typename Inherit::Vertex;
    using Face = typename Inherit::Face;
    using ChunkArrayGen = typename Inherit::ChunkArrayGen;

protected:

    virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& option) override
    {

        ChunkArrayGen const* position = this->position_attribute(Vertex::ORBIT);

        //Header
        output << "GOCAD TSurf 1.0" << std::endl;


        //Coordinate system
        output << "GOCAD_ORIGINAL_COORDINATE_SYSTEM" << std::endl;
        output << "END_ORIGINAL_COORDINATE_SYSTEM" << std::endl;

        //vertices
        uint32 vertices_counter = 1u;
        map.foreach_cell([&](Vertex v)
        {
            output << "VRTX " << vertices_counter++ << " ";
            position->export_element(map.embedding(v), output, false, false);
            output << std::endl;
        }, *(this->cell_cache_));

        //faces
        std::vector<uint32> vertices;
        vertices.reserve(3u);
        map.foreach_cell([&](Face f)
        {
            vertices.clear();
            map.foreach_incident_vertex(f, [&] (Vertex v)
            { vertices.push_back(this->vindices_[v] + 1u); });

            for(uint32 i: prim)
                output << "TRGL " << i;
            output << std::end;

        }, *(this->cell_cache_));

        output << "END" << std::endl;
    }
};

} // end namespace io

} // end namespace cgogn

#endif // CGOGN_IO_FORMATS_TS_H_

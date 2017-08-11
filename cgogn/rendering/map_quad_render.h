
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

#ifndef CGOGN_RENDERING_MAP_RENDER_QUAD_H_
#define CGOGN_RENDERING_MAP_RENDER_QUAD_H_

#include <cgogn/rendering/dll.h>

#include <cgogn/core/cmap/map_base.h> // impossible to include directly attribute.h !

#include <cgogn/geometry/algos/ear_triangulation.h>

#include <cgogn/rendering/drawer.h>
#include <cgogn/rendering/shaders/vbo.h>

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>

namespace cgogn
{

namespace rendering
{


class CGOGN_RENDERING_API MapQuadRender
{
protected:

    std::unique_ptr<QOpenGLBuffer>    indices_buffers_;
    uint32     nb_indices_;

public:

    using Self = MapQuadRender;

    MapQuadRender();
    ~MapQuadRender();
    CGOGN_NOT_COPYABLE_NOR_MOVABLE(MapQuadRender);

    template <typename VEC3, typename MAP>
      inline void init_primitives(const MAP& m)
      {
          using Vertex = typename MAP::Vertex;
          using Face = typename MAP::Face;

          std::vector<uint32> table_indices;

          m.foreach_cell([&] (Face f)
          {
              Dart d0 = f.dart;
              Dart d1 = m.phi1(d0);
              Dart d2 = m.phi1(d1);
              Dart d3 = m.phi1(d2);
              table_indices.push_back(m.embedding(Vertex(d0)));
              table_indices.push_back(m.embedding(Vertex(d1)));
              table_indices.push_back(m.embedding(Vertex(d2)));
              table_indices.push_back(m.embedding(Vertex(d3)));
          });

          if (!indices_buffers_->isCreated())
              indices_buffers_->create();

          nb_indices_ = uint32(table_indices.size());
          indices_buffers_->bind();
          indices_buffers_->allocate(table_indices.data(), nb_indices_ * sizeof(uint32));
          indices_buffers_->release();
      }

    void draw();
};

} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_MAP_RENDER_QUAD_H_

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


#include <cgogn/rendering/map_quad_render.h>

namespace cgogn
{

namespace rendering
{

MapQuadRender::MapQuadRender()
{
    indices_buffers_ = make_unique<QOpenGLBuffer>(QOpenGLBuffer::IndexBuffer);
    indices_buffers_->setUsagePattern(QOpenGLBuffer::StaticDraw);
}

MapQuadRender::~MapQuadRender()
{}

void
MapQuadRender::draw()
{
    QOpenGLFunctions* ogl = QOpenGLContext::currentContext()->functions();
    indices_buffers_->bind();
    ogl->glDrawElements(GL_LINES_ADJACENCY, nb_indices_, GL_UNSIGNED_INT, 0);
    indices_buffers_->release();
}

} // namespace rendering

} // namespace cgogn

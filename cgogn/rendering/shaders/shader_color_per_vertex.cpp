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

#define CGOGN_RENDERING_DLL_EXPORT

#include <cgogn/rendering/shaders/shader_color_per_vertex.h>

#include <QOpenGLFunctions>
#include <iostream>

namespace cgogn
{

namespace rendering
{

const char* ShaderColorPerVertex::vertex_shader_source_ =
"#version 150\n"
"in vec3 vertex_pos;\n"
"in vec3 vertex_color;\n"
"uniform mat4 projection_matrix;\n"
"uniform mat4 model_view_matrix;\n"
"out vec3 color_v;\n"
"void main() {\n"
"   color_v = vertex_color;"
"   gl_Position = projection_matrix * model_view_matrix * vec4(vertex_pos,1.0);\n"
"}\n";

const char* ShaderColorPerVertex::fragment_shader_source_ =
"#version 150\n"
"in vec3 color_v;\n"
"out vec3 fragColor;\n"
"void main() {\n"
"   fragColor = color_v;\n"
"}\n";


ShaderColorPerVertex::ShaderColorPerVertex()
{
	prg_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_source_);
	prg_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_source_);
	prg_.bindAttributeLocation("vertex_pos", ATTRIB_POS);
	prg_.bindAttributeLocation("vertex_color", ATTRIB_COLOR);
    prg_.link();

	get_matrices_uniforms();
}

ShaderParamColorPerVertex::ShaderParamColorPerVertex(ShaderColorPerVertex* prg):
	ShaderParam(prg)
{}

void ShaderParamColorPerVertex::set_vbo(VBO* vbo_pos, VBO* vbo_color)
{

    QOpenGLFunctions *ogl = QOpenGLContext::currentContext()->functions();

	shader_->bind();
	vao_->bind();

	// position vbo
	vbo_pos->bind();
	ogl->glEnableVertexAttribArray(ShaderColorPerVertex::ATTRIB_POS);
	ogl->glVertexAttribPointer(ShaderColorPerVertex::ATTRIB_POS, vbo_pos->vector_dimension(), GL_FLOAT, GL_FALSE, 0, 0);
	vbo_pos->release();

	// color vbo
	vbo_color->bind();
	ogl->glEnableVertexAttribArray(ShaderColorPerVertex::ATTRIB_COLOR);
	ogl->glVertexAttribPointer(ShaderColorPerVertex::ATTRIB_COLOR, vbo_color->vector_dimension(), GL_FLOAT, GL_FALSE, 0, 0);
    vbo_color->release();

	vao_->release();
	shader_->release();

}

} // namespace rendering

} // namespace cgogn

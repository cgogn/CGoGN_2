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

#include <iostream>

#include <cgogn/rendering/shaders/shader_simple_color.h>

#include <QOpenGLFunctions>
#include <QColor>

namespace cgogn
{

namespace rendering
{

ShaderSimpleColor* ShaderSimpleColor::instance_ = nullptr;

const char* ShaderSimpleColor::vertex_shader_source_ =
"#version 150\n"
"in vec3 vertex_pos;\n"
"uniform mat4 projection_matrix;\n"
"uniform mat4 model_view_matrix;\n"
"void main() {\n"
"   gl_Position = projection_matrix * model_view_matrix * vec4(vertex_pos,1.0);\n"
"}\n";

const char* ShaderSimpleColor::fragment_shader_source_ =
"#version 150\n"
"out vec4 fragColor;\n"
"uniform vec4 color;\n"
"void main() {\n"
"   fragColor = color;\n"
"}\n";



ShaderSimpleColor::ShaderSimpleColor()
{
	prg_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_source_);
	prg_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_source_);
	prg_.bindAttributeLocation("vertex_pos", ATTRIB_POS);
	prg_.link();

	get_matrices_uniforms();

	unif_color_ = prg_.uniformLocation("color");

	//default param
	set_color(QColor(255, 255, 255));
}

void ShaderSimpleColor::set_color(const QColor& rgb)
{
	prg_.setUniformValue(unif_color_, rgb);
}



ShaderParamSimpleColor::ShaderParamSimpleColor(ShaderSimpleColor* sh):
	ShaderParam(sh),
	color_(255, 255, 255)
{}

void ShaderParamSimpleColor::set_uniforms()
{
	ShaderSimpleColor* sh = static_cast<ShaderSimpleColor*>(this->shader_);
	sh->set_color(color_);
}

void ShaderParamSimpleColor::set_vbo(VBO* vbo_pos, uint32 stride, uint32 first)
{
	QOpenGLFunctions *ogl = QOpenGLContext::currentContext()->functions();

	shader_->bind();
	vao_->bind();

	// position vbo
	vbo_pos->bind();
	ogl->glEnableVertexAttribArray(ShaderSimpleColor::ATTRIB_POS);

	ogl->glVertexAttribPointer(ShaderSimpleColor::ATTRIB_POS, vbo_pos->vector_dimension(), GL_FLOAT, GL_FALSE, stride * vbo_pos->vector_dimension() * 4, void_ptr(first * vbo_pos->vector_dimension() * 4));
	vbo_pos->release();

	vao_->release();
	shader_->release();
}

} // namespace rendering

} // namespace cgogn

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

#include <rendering/shaders/shader_simple_color.h>

#include <QOpenGLFunctions>
#include <QColor>
#include <iostream>

namespace cgogn
{

namespace rendering
{

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
	set_color(QColor(255,255,255));
}

void ShaderSimpleColor::set_color(const QColor& rgb)
{
	prg_.setUniformValue(unif_color_, rgb);
}


bool ShaderSimpleColor::set_vao(uint32 i, VBO* vbo_pos, uint32 stride, unsigned first)
{
	if (i >= vaos_.size())
	{
		std::cerr << "VAO number " << i << " does not exist" << std::endl;
		return false;
	}

	QOpenGLFunctions *ogl = QOpenGLContext::currentContext()->functions();

	prg_.bind();
	vaos_[i]->bind();

	// position vbo
	vbo_pos->bind();
	ogl->glEnableVertexAttribArray(ATTRIB_POS);

	ogl->glVertexAttribPointer(ATTRIB_POS, vbo_pos->vector_dimension(), GL_FLOAT, GL_FALSE, stride*vbo_pos->vector_dimension() * 4,
		void_ptr(first*vbo_pos->vector_dimension() * 4));
	vbo_pos->release();

    vaos_[i]->release();
	prg_.release();

    return true;
}

} // namespace rendering

} // namespace cgogn

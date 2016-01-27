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

#include "rendering/shaders/shader_simple_color.h"
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

	// do not forget to get matrices uniforms !
	get_matrices_uniforms();

	// and shader specific uniforms
	unif_color_ = prg_.uniformLocation("color");
}


void ShaderSimpleColor::set_color(const QColor& rgb)
{
	prg_.setUniformValue(unif_color_,rgb);
}


bool ShaderSimpleColor::set_vao(unsigned int i, VBO* vbo_pos)
{
	if (i>=vaos_.size())
	{
		std::cerr << "VAO number "<< i << "does not exist"<< std::endl;
		return false;
	}

	// here we need gl function specific to current context
	QOpenGLFunctions *ogl_f = QOpenGLContext::currentContext()->functions();

	prg_.bind();

	// bind the choosen vao
	vaos_[i]->bind();

	// use vbo
	vbo_pos->bind();
	//associate with attribute "vertex_pos"
	ogl_f->glEnableVertexAttribArray(ATTRIB_POS);
	// parameters
	ogl_f->glVertexAttribPointer(ATTRIB_POS, vbo_pos->nb_comp(), GL_FLOAT, GL_FALSE, 0, 0);
	// finish with this vbo
	vbo_pos->release();

	// finish vao filling
    vaos_[i]->release();

	prg_.release();

    return true;
}

} // namespace rendering
} // namespace cgogn




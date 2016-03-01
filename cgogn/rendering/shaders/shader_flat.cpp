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

#include <rendering/shaders/shader_flat.h>

#include <QOpenGLFunctions>
#include <QColor>
#include <iostream>

namespace cgogn
{

namespace rendering
{

const char* ShaderFlat::vertex_shader_source_ =
	"#version 150\n"
	"in vec3 vertex_pos;\n"
	"uniform mat4 projection_matrix;\n"
	"uniform mat4 model_view_matrix;\n"
	"out vec3 pos;\n"
	"void main() {\n"
	"	vec4 pos4 = model_view_matrix * vec4(vertex_pos,1.0);\n"
	"	pos = pos4.xyz;"
	"   gl_Position = projection_matrix * pos4;\n"
	"}\n";

const char* ShaderFlat::fragment_shader_source_ =
	"#version 150\n"
	"out vec4 fragColor;\n"
	"uniform vec4 front_color;\n"
	"uniform vec4 back_color;\n"
	"uniform vec4 ambiant_color;\n"
	"vec3 light_pos = vec3(100,100,1000);\n"
	"in vec3 pos;\n"
	"void main() {\n"
	"	vec3 N = normalize(cross(dFdx(pos),dFdy(pos)));\n"
	"	vec3 L = normalize(light_pos-pos);\n"
	"	float lambert = abs(dot(N,L));\n"
	"	if (gl_FrontFacing)\n"
	"		fragColor = ambiant_color+lambert*front_color;\n"
	"	else\n"
	"		fragColor = ambiant_color+lambert*back_color;\n"
	"}\n";


const char* ShaderFlat::vertex_shader_source2_ =
	"#version 150\n"
	"in vec3 vertex_pos;\n"
	"in vec3 vertex_col;\n"
	"uniform mat4 projection_matrix;\n"
	"uniform mat4 model_view_matrix;\n"
	"out vec3 pos;\n"
	"out vec3 col;\n"
	"void main() {\n"
	"	vec4 pos4 = model_view_matrix * vec4(vertex_pos,1.0);\n"
	"	pos = pos4.xyz;\n"
	"	col = vertex_col;\n"
	"   gl_Position = projection_matrix * pos4;\n"
	"}\n";

const char* ShaderFlat::fragment_shader_source2_ =
	"#version 150\n"
	"out vec4 fragColor;\n"
	"uniform vec4 front_color;\n"
	"uniform vec4 back_color;\n"
	"uniform vec4 ambiant_color;\n"
	"vec3 light_pos = vec3(100,100,1000);\n"
	"in vec3 pos;\n"
	"in vec3 col;\n"
	"void main() {\n"
	"	vec3 N = normalize(cross(dFdx(pos),dFdy(pos)));\n"
	"	vec3 L = normalize(light_pos-pos);\n"
	"	float lambert = abs(dot(N,L));\n"
	"	fragColor = ambiant_color+vec4(lambert*col,1.0);\n"
	"}\n";


ShaderFlat::ShaderFlat(bool color_per_vertex)
{
	if (color_per_vertex)
	{
		prg_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_source2_);
		prg_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_source2_);
		prg_.bindAttributeLocation("vertex_pos", ATTRIB_POS);
		prg_.bindAttributeLocation("vertex_col", ATTRIB_COLOR);
		prg_.link();
		get_matrices_uniforms();
		unif_ambiant_color_ = prg_.uniformLocation("ambiant_color");
	}
	else
	{
		prg_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_source_);
		prg_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_source_);
		prg_.bindAttributeLocation("vertex_pos", ATTRIB_POS);
		prg_.link();
		get_matrices_uniforms();
		unif_front_color_ = prg_.uniformLocation("front_color");
		unif_back_color_ = prg_.uniformLocation("back_color");
		unif_ambiant_color_ = prg_.uniformLocation("ambiant_color");


	}
}

void ShaderFlat::set_front_color(const QColor& rgb)
{
	prg_.setUniformValue(unif_front_color_,rgb);
}

void ShaderFlat::set_back_color(const QColor& rgb)
{
	prg_.setUniformValue(unif_back_color_,rgb);
}

void ShaderFlat::set_ambiant_color(const QColor& rgb)
{
	prg_.setUniformValue(unif_ambiant_color_,rgb);
}


bool ShaderFlat::set_vao(unsigned int i, VBO* vbo_pos, VBO* vbo_color)
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
	ogl->glVertexAttribPointer(ATTRIB_POS, vbo_pos->vector_dimension(), GL_FLOAT, GL_FALSE, 0, 0);
	vbo_pos->release();

	if (vbo_color)
	{
		// color  vbo
		vbo_color->bind();
		ogl->glEnableVertexAttribArray(ATTRIB_COLOR);
		ogl->glVertexAttribPointer(ATTRIB_COLOR, vbo_color->vector_dimension(), GL_FLOAT, GL_FALSE, 0, 0);
		vbo_color->release();
	}

	vaos_[i]->release();
	prg_.release();

	return true;
}

} // namespace rendering

} // namespace cgogn

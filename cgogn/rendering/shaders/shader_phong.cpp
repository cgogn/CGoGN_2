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

#include <rendering/shaders/shader_phong.h>

#include <QOpenGLFunctions>
#include <QColor>
#include <iostream>

namespace cgogn
{

namespace rendering
{

const char* ShaderPhong::vertex_shader_source_ =
	"#version 150\n"
	"in vec3 vertex_pos;\n"
	"in vec3 vertex_normal;\n"
	"uniform mat4 projection_matrix;\n"
	"uniform mat4 model_view_matrix;\n"
	"uniform mat3 normal_matrix;\n"
	"uniform vec3 lightPosition;\n"
	"out vec3 EyeVector;\n"
	"out vec3 Normal;\n"
	"out vec3 LightDir;\n"
	"void main ()\n"
	"{\n"
	"	Normal = normal_matrix * vertex_normal;\n"
	"	vec3 Position = vec3 (model_view_matrix * vec4 (vertex_pos, 1.0));\n"
	"	LightDir = lightPosition - Position;\n"
	"	EyeVector = -Position;"
	"	gl_Position = projection_matrix * model_view_matrix * vec4 (vertex_pos, 1.0);\n"
	"}\n";

const char* ShaderPhong::fragment_shader_source_ =
	"#version 150\n"
	"in vec3 EyeVector;\n"
	"in vec3 Normal;\n"
	"in vec3 LightDir;\n"
	"uniform vec4 front_color;\n"
	"uniform vec4 spec_color;\n"
	"uniform vec4 ambiant_color;\n"
	"uniform vec4 back_color;\n"
	"uniform float spec_coef;\n"
	"uniform bool double_side;\n"
	"out vec4 frag_color;\n"
	"void main()\n"
	"{\n"
	"	vec3 N = normalize (Normal);\n"
	"	vec3 L = normalize (LightDir);\n"
	"	vec4 finalColor = ambiant_color;\n"
	"	vec4 currentColor = front_color;\n"
	"	if (!gl_FrontFacing)\n"
	"	{\n"
	"		if (!double_side)\n"
	"			discard;\n"
	"		N *= -1.0;\n"
	"		currentColor = back_color;\n"
	"	}\n"
	"	float lambertTerm = clamp(dot(N,L),0.0,1.0);\n"
	"	finalColor += currentColor*lambertTerm ;\n"
	"	vec3 E = normalize(EyeVector);\n"
	"	vec3 R = reflect(-L, N);\n"
	"	float specular = pow( max(dot(R, E), 0.0), spec_coef );\n"
	"	finalColor += spec_color * specular;\n"
	"	frag_color=finalColor;\n"
	"}\n";


const char* ShaderPhong::vertex_shader_source_2_ =
	"#version 150\n"
	"in vec3 vertex_pos;\n"
	"in vec3 vertex_normal;\n"
	"in vec3 vertex_color;\n"
	"uniform mat4 projection_matrix;\n"
	"uniform mat4 model_view_matrix;\n"
	"uniform mat3 normal_matrix;\n"
	"uniform vec3 lightPosition;\n"
	"out vec3 EyeVector;\n"
	"out vec3 Normal;\n"
	"out vec3 LightDir;\n"
	"out vec3 front_color;\n"
	"void main ()\n"
	"{\n"
	"	Normal = normal_matrix * vertex_normal;\n"
	"	vec3 Position = vec3 (model_view_matrix * vec4 (vertex_pos, 1.0));\n"
	"	LightDir = lightPosition - Position;\n"
	"	EyeVector = -Position;"
	"	front_color = vertex_color;"
	"	gl_Position = projection_matrix * model_view_matrix * vec4 (vertex_pos, 1.0);\n"
	"}\n";

const char* ShaderPhong::fragment_shader_source_2_ =
	"#version 150\n"
	"in vec3 EyeVector;\n"
	"in vec3 Normal;\n"
	"in vec3 LightDir;\n"
	"in vec3 front_color;\n"
	"uniform vec4 spec_color;\n"
	"uniform vec4 ambiant_color;\n"
	"uniform float spec_coef;\n"
	"uniform bool double_side;\n"
	"out vec4 frag_color;\n"
	"void main()\n"
	"{\n"
	"	vec3 N = normalize (Normal);\n"
	"	vec3 L = normalize (LightDir);\n"
	"	vec4 finalColor = ambiant_color;\n"
	"	if (!gl_FrontFacing)\n"
	"	{\n"
	"		if (!double_side)\n"
	"			discard;\n"
	"		N *= -1.0;\n"
	"	}\n"
	"	float lambertTerm = clamp(dot(N,L),0.0,1.0);\n"
	"	finalColor += vec4(front_color*lambertTerm,0.0);\n"
	"	vec3 E = normalize(EyeVector);\n"
	"	vec3 R = reflect(-L, N);\n"
	"	float specular = pow( max(dot(R, E), 0.0), spec_coef );\n"
	"	finalColor += spec_color * specular;\n"
	"	frag_color=finalColor;\n"
	"}\n";



ShaderPhong::ShaderPhong(bool color_per_vertex)
{
	if (color_per_vertex)
	{
		prg_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_source_2_);
		prg_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_source_2_);
		prg_.bindAttributeLocation("vertex_pos", ATTRIB_POS);
		prg_.bindAttributeLocation("vertex_normal", ATTRIB_NORM);
		prg_.bindAttributeLocation("vertex_color", ATTRIB_COLOR);
		prg_.link();

		get_matrices_uniforms();

		unif_ambiant_color_ = prg_.uniformLocation("ambiant_color");
		unif_spec_color_ = prg_.uniformLocation("spec_color");
		unif_spec_coef_ = prg_.uniformLocation("spec_coef");
		unif_double_side_ = prg_.uniformLocation("double_side");
	}
	else
	{
		prg_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_source_);
		prg_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_source_);
		prg_.bindAttributeLocation("vertex_pos", ATTRIB_POS);
		prg_.bindAttributeLocation("vertex_normal", ATTRIB_NORM);
		prg_.link();

		get_matrices_uniforms();

		unif_front_color_ = prg_.uniformLocation("front_color");
		unif_back_color_ = prg_.uniformLocation("back_color");
		unif_ambiant_color_ = prg_.uniformLocation("ambiant_color");
		unif_spec_color_ = prg_.uniformLocation("spec_color");
		unif_spec_coef_ = prg_.uniformLocation("spec_coef");
		unif_double_side_ = prg_.uniformLocation("double_side");
	}
}


void ShaderPhong::set_front_color(const QColor& rgb)
{
	if (unif_front_color_>=0)
		prg_.setUniformValue(unif_front_color_,rgb);
}

void ShaderPhong::set_back_color(const QColor& rgb)
{
	if (unif_back_color_>=0)
		prg_.setUniformValue(unif_back_color_,rgb);
}

void ShaderPhong::set_ambiant_color(const QColor& rgb)
{
	prg_.setUniformValue(unif_ambiant_color_,rgb);
}

void ShaderPhong::set_specular_color(const QColor& rgb)
{
	prg_.setUniformValue(unif_spec_color_,rgb);
}

void ShaderPhong::set_specular_coef(float coef)
{
	prg_.setUniformValue(unif_spec_coef_,coef);
}

void ShaderPhong::set_double_side(bool ts)
{
	prg_.setUniformValue(unif_double_side_,ts);
}

bool ShaderPhong::set_vao(unsigned int i, VBO* vbo_pos, VBO* vbo_norm, VBO* vbo_color)
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

	// position vbo
	vbo_norm->bind();
	ogl->glEnableVertexAttribArray(ATTRIB_NORM);
	ogl->glVertexAttribPointer(ATTRIB_NORM, vbo_norm->vector_dimension(), GL_FLOAT, GL_FALSE, 0, 0);
	vbo_norm->release();

	if (vbo_color)
	{
		// color  vbo
		vbo_color->bind();
		ogl->glEnableVertexAttribArray(ATTRIB_COLOR);
		ogl->glVertexAttribPointer(ATTRIB_COLOR, vbo_color->vector_dimension(), GL_FLOAT, GL_FALSE, 0, 0);
		vbo_color->release();
	}
	else
	{
		if (unif_front_color_<0)
			std::cerr << "ShaderPhong: no color attribute, no color uniform"<< std::endl;
	}

	vaos_[i]->release();
	prg_.release();

	return true;
}

} // namespace rendering

} // namespace cgogn

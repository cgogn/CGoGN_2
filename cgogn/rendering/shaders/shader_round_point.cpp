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

#include <rendering/shaders/shader_round_point.h>

#include <QOpenGLFunctions>
#include <QColor>
#include <iostream>

namespace cgogn
{

namespace rendering
{

const char* ShaderRoundPoint::vertex_shader_source_ =
"#version 150\n"
"in vec3 vertex_pos;\n"
"void main() {\n"
"   gl_Position =  vec4(vertex_pos,1.0);\n"
"}\n";


const char* ShaderRoundPoint::geometry_shader_source_ =
"#version 150\n"
"layout (points) in;\n"
"layout (triangle_strip, max_vertices=4) out;\n"
"uniform mat4 projection_matrix;\n"
"uniform mat4 model_view_matrix;\n"
"uniform vec2 pointSizes;\n"
"out vec2 local;\n"
"void main()\n"
"{\n"
"	vec4 A = projection_matrix*model_view_matrix * gl_in[0].gl_Position;\n"
"	A = A/A.w;\n"
"	local = vec2(-1.0,-1.0);\n"
"	gl_Position = vec4(A.xyz-vec3(-pointSizes[0],-pointSizes[1],0.0), 1.0);\n"
"	EmitVertex();\n"
"	local = vec2(1.0,-1.0);\n"
"	gl_Position = vec4(A.xyz-vec3(pointSizes[0],-pointSizes[1],0.0), 1.0);\n"
"	EmitVertex();\n"
"	local = vec2(-1.0,1.0);\n"
"	gl_Position = vec4(A.xyz-vec3(-pointSizes[0],pointSizes[1],0.0), 1.0);\n"
"	EmitVertex();\n"
"	local = vec2(1.0,1.0);\n"
"	gl_Position = vec4(A.xyz-vec3(pointSizes[0],pointSizes[1],0.0), 1.0);\n"
"	EmitVertex();\n"
"	EndPrimitive();\n"
"}\n";


const char* ShaderRoundPoint::fragment_shader_source_ =
"#version 150\n"
"uniform vec4 color;\n"
"in vec2 local;\n"
"out vec4 fragColor;\n"
"void main() {\n"

"	float r2 = dot(local,local);\n"
"   if (r2 > 1.0) discard;\n"
"   fragColor = vec4(color.rgb,(1.0-r2*r2));\n"
"}\n";




const char* ShaderRoundPoint::vertex_shader_source2_ =
"#version 150\n"
"in vec3 vertex_pos;\n"
"in vec3 vertex_color;\n"
"out vec3 color_v;\n"
"void main() {\n"
"   color_v = vertex_color;\n"
"   gl_Position = vec4(vertex_pos,1.0);\n"
"}\n";


const char* ShaderRoundPoint::geometry_shader_source2_ =
"#version 150\n"
"layout (points) in;\n"
"layout (triangle_strip, max_vertices=4) out;\n"
"uniform mat4 projection_matrix;\n"
"uniform mat4 model_view_matrix;\n"
"uniform vec2 pointSizes;\n"
"in vec3 color_v[];\n"
"out vec2 local;\n"
"out vec3 color_f;\n"
"void main()\n"
"{\n"
"	vec4 A = projection_matrix*model_view_matrix * gl_in[0].gl_Position;\n"
"	A = A/A.w;\n"
"	color_f = color_v[0];\n"
"	local = vec2(-1.0,-1.0);\n"
"	gl_Position = vec4(A.xyz-vec3(-pointSizes[0],-pointSizes[1],0.0), 1.0);\n"
"	EmitVertex();\n"
"	local = vec2(1.0,-1.0);\n"
"	gl_Position = vec4(A.xyz-vec3(pointSizes[0],-pointSizes[1],0.0), 1.0);\n"
"	EmitVertex();\n"
"	local = vec2(-1.0,1.0);\n"
"	gl_Position = vec4(A.xyz-vec3(-pointSizes[0],pointSizes[1],0.0), 1.0);\n"
"	EmitVertex();\n"
"	local = vec2(1.0,1.0);\n"
"	gl_Position = vec4(A.xyz-vec3(pointSizes[0],pointSizes[1],0.0), 1.0);\n"
"	EmitVertex();\n"
"	EndPrimitive();\n"
"}\n";

const char* ShaderRoundPoint::fragment_shader_source2_ =
"#version 150\n"
"in vec2 local;\n"
"in vec3 color_f;\n"
"out vec4 fragColor;\n"
"void main() {\n"
"	float r2 = dot(local,local);\n"
"   if (r2 > 1.0) discard;\n"
"   fragColor = vec4(color_f,(1.0-r2*r2));\n"
"}\n";


ShaderRoundPoint::ShaderRoundPoint(bool color_per_vertex)
{
	if (color_per_vertex)
	{
		prg_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_source2_);
		prg_.addShaderFromSourceCode(QOpenGLShader::Geometry, geometry_shader_source2_);
		prg_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_source2_);
		prg_.bindAttributeLocation("vertex_pos", ATTRIB_POS);
		prg_.bindAttributeLocation("vertex_color", ATTRIB_COLOR);
		prg_.link();
		get_matrices_uniforms();
	}
	else
	{
		prg_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_source_);
		prg_.addShaderFromSourceCode(QOpenGLShader::Geometry, geometry_shader_source_);
		prg_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_source_);
		prg_.bindAttributeLocation("vertex_pos", ATTRIB_POS);
		prg_.link();

		get_matrices_uniforms();
	}
	unif_color_ = prg_.uniformLocation("color");
	unif_width_ = prg_.uniformLocation("pointSizes");

	set_width(3.0f);
	set_color(QColor(255,255,255));
}



void ShaderRoundPoint::set_color(const QColor& rgb)
{
	if (unif_color_)
		prg_.setUniformValue(unif_color_, rgb);
}

void ShaderRoundPoint::set_width(float wpix)
{
	QOpenGLFunctions *ogl = QOpenGLContext::currentContext()->functions();
	int viewport[4];
	ogl->glGetIntegerv(GL_VIEWPORT, viewport);
	QSizeF wd(wpix / float(viewport[2]), wpix / float(viewport[3]));
	prg_.setUniformValue(unif_width_, wd);
}

bool ShaderRoundPoint::set_vao(unsigned int i, VBO* vbo_pos, VBO* vbo_color)
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
		// color vbo
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

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

#include <rendering/shaders/shader_bold_line.h>

#include <QOpenGLFunctions>
#include <iostream>

namespace cgogn
{

namespace rendering
{

const char* ShaderBoldLine::vertex_shader_source_ =
"#version 150\n"
"in vec3 vertex_pos;\n"
"void main() {\n"
"   gl_Position =  vec4(vertex_pos,1.0);\n"
"}\n";


const char* ShaderBoldLine::geometry_shader_source_ =
"#version 150\n"
"layout (lines) in;\n"
"layout (triangle_strip, max_vertices=6) out;\n"
"out vec4 color_f;\n"
"uniform mat4 projection_matrix;\n"
"uniform mat4 model_view_matrix;\n"
"uniform vec2 lineWidths;\n"
"uniform vec4 lineColor;\n"
"void main()\n"
"{\n"
"	vec4 A = model_view_matrix * gl_in[0].gl_Position;\n"
"	vec4 B = model_view_matrix * gl_in[1].gl_Position;\n"
"	float nearZ = 1.0;\n"
"	if (projection_matrix[2][2] !=  1.0)\n"
"		nearZ = - projection_matrix[3][2] / (projection_matrix[2][2] - 1.0); \n"
"	if ((A.z < nearZ) || (B.z < nearZ))\n"
"	{\n"
"		if (A.z >= nearZ)\n"
"			A = B + (A-B)*(nearZ-B.z)/(A.z-B.z);\n"
"		if (B.z >= nearZ)\n"
"			B = A + (B-A)*(nearZ-A.z)/(B.z-A.z);\n"
"		A = projection_matrix*A;\n"
"		B = projection_matrix*B;\n"
"		A = A/A.w;\n"
"		B = B/B.w;\n"
"		vec2 U2 = normalize(vec2(lineWidths[1],lineWidths[0])*(B.xy - A.xy));\n"
"		vec2 LWCorr =lineWidths * max(abs(U2.x),abs(U2.y));\n"
"		vec3 U = vec3(0.5*LWCorr*U2,0.0);\n"
"		vec3 V = vec3(LWCorr*vec2(U2[1], -U2[0]), 0.0);	\n"
"		vec3 color3 = lineColor.rgb;\n"
"		color_f = vec4(color3,0.0);\n"
"		gl_Position = vec4(A.xyz-V, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color3,0.0);\n"
"		gl_Position = vec4(B.xyz-V, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color3,1.0);\n"
"		gl_Position = vec4(A.xyz-U, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color3,1.0);\n"
"		gl_Position = vec4(B.xyz+U, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color3,0.0);\n"
"		gl_Position = vec4(A.xyz+V, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color3,0.0);\n"
"		gl_Position = vec4(B.xyz+V, 1.0);\n"
"		EmitVertex();\n"
"		EndPrimitive();\n"
"	}\n"
"}\n";


const char* ShaderBoldLine::fragment_shader_source_ =
"#version 150\n"
"in vec4 color_f;\n"
"out vec4 fragColor;\n"
"void main() {\n"
"   fragColor = color_f;\n"
"}\n";




const char* ShaderBoldLine::vertex_shader_source2_ =
"#version 150\n"
"in vec3 vertex_pos;\n"
"in vec3 vertex_color;\n"
"out vec3 color_v;\n"
"void main() {\n"
"   color_v = vertex_color;\n"
"   gl_Position = vec4(vertex_pos,1.0);\n"
"}\n";


const char* ShaderBoldLine::geometry_shader_source2_ =
"#version 150\n"
"layout (lines) in;\n"
"layout (triangle_strip, max_vertices=6) out;\n"
"in vec3 color_v[];\n"
"out vec4 color_f;\n"
"uniform mat4 projection_matrix;\n"
"uniform mat4 model_view_matrix;\n"
"uniform vec2 lineWidths;\n"
"void main()\n"
"{\n"
"	vec4 A = model_view_matrix * gl_in[0].gl_Position;\n"
"	vec4 B = model_view_matrix * gl_in[1].gl_Position;\n"
"	float nearZ = 1.0;\n"
"	if (projection_matrix[2][2] !=  1.0)\n"
"		nearZ = - projection_matrix[3][2] / (projection_matrix[2][2] - 1.0); \n"
"	if ((A.z < nearZ) || (B.z < nearZ))\n"
"	{\n"
"		if (A.z >= nearZ)\n"
"			A = B + (A-B)*(nearZ-B.z)/(A.z-B.z);\n"
"		if (B.z >= nearZ)\n"
"			B = A + (B-A)*(nearZ-A.z)/(B.z-A.z);\n"
"		A = projection_matrix*A;\n"
"		B = projection_matrix*B;\n"
"		A = A/A.w;\n"
"		B = B/B.w;\n"
"		vec2 U2 = normalize(vec2(lineWidths[1],lineWidths[0])*(B.xy - A.xy));\n"
"		vec2 LWCorr =lineWidths * max(abs(U2.x),abs(U2.y));\n"
"		vec3 U = vec3(LWCorr*U2,0.0);\n"
"		vec3 V = vec3(LWCorr*vec2(U2[1], -U2[0]), 0.0);	\n"
"		color_f = vec4(color_v[0],0.0);\n"
"		gl_Position = vec4(A.xyz-V, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color_v[1],0.0);\n"
"		gl_Position = vec4(B.xyz-V, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color_v[0],1.0);\n"
"		gl_Position = vec4(A.xyz-U, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color_v[1],1.0);\n"
"		gl_Position = vec4(B.xyz+U, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color_v[0],0.0);\n"
"		gl_Position = vec4(A.xyz+V, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color_v[1],0.0);\n"
"		gl_Position = vec4(B.xyz+V, 1.0);\n"
"		EmitVertex();\n"
"		EndPrimitive();\n"
"	}\n"
"}\n";


const char* ShaderBoldLine::fragment_shader_source2_ =
"#version 150\n"
"in vec4 color_f;\n"
"out vec4 fragColor;\n"
"void main() {\n"
"   fragColor = color_f;\n"
"}\n";



ShaderBoldLine::ShaderBoldLine(bool color_per_vertex)
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
	unif_color_ = prg_.uniformLocation("lineColor");
	unif_width_ = prg_.uniformLocation("lineWidths");

}



void ShaderBoldLine::set_color(const QColor& rgb)
{
	if (unif_color_ >= 0)
		prg_.setUniformValue(unif_color_, rgb);
}

void ShaderBoldLine::set_width(float32 wpix)
{
	QOpenGLFunctions *ogl = QOpenGLContext::currentContext()->functions();
	GLint viewport[4];
	ogl->glGetIntegerv(GL_VIEWPORT, viewport);
	QSizeF wd(wpix / float32(viewport[2]), wpix / float32(viewport[3]));
	prg_.setUniformValue(unif_width_, wd);
}

bool ShaderBoldLine::set_vao(uint32 i, VBO* vbo_pos, VBO* vbo_color)
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

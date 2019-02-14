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

#define CGOGN_RENDER_SHADERS_BOLD_LINE_CPP_

#include <iostream>

#include <cgogn/core/utils/logger.h>
#include <cgogn/rendering/shaders/shader_bold_line.h>

namespace cgogn
{

namespace rendering
{

const char* ShaderBoldLineGen::vertex_shader_source_ =
"#version 150\n"
"in vec3 vertex_pos;\n"
"void main()\n"
"{\n"
"   gl_Position =  vec4(vertex_pos,1.0);\n"
"}\n";

const char* ShaderBoldLineGen::geometry_shader_source_ =
"#version 150\n"
"layout (lines) in;\n"
"layout (triangle_strip, max_vertices=6) out;\n"
"out vec4 color_f;\n"
"out vec4 posi_clip;\n"
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
"		posi_clip = gl_in[0].gl_Position;\n"
"		gl_Position = vec4(A.xyz-V, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color3,0.0);\n"
"		posi_clip = gl_in[1].gl_Position;\n"
"		gl_Position = vec4(B.xyz-V, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color3,1.0);\n"
"		posi_clip = gl_in[0].gl_Position;\n"
"		gl_Position = vec4(A.xyz-U, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color3,1.0);\n"
"		posi_clip = gl_in[1].gl_Position;\n"
"		gl_Position = vec4(B.xyz+U, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color3,0.0);\n"
"		posi_clip = gl_in[0].gl_Position;\n"
"		gl_Position = vec4(A.xyz+V, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color3,0.0);\n"
"		posi_clip = gl_in[1].gl_Position;\n"
"		gl_Position = vec4(B.xyz+V, 1.0);\n"
"		EmitVertex();\n"
"		EndPrimitive();\n"
"	}\n"
"}\n";

const char* ShaderBoldLineGen::fragment_shader_source_ =
"#version 150\n"
"uniform vec4 plane_clip;\n"
"uniform vec4 plane_clip2;\n"
"in vec4 color_f;\n"
"in vec4 posi_clip;\n"
"out vec4 fragColor;\n"
"void main()\n"
"{\n"
"	float d = dot(plane_clip,posi_clip);\n"
"	float d2 = dot(plane_clip2,posi_clip);\n"
"	if ((d>0.0)||(d2>0.0))  discard;\n"
"   fragColor = color_f;\n"
"}\n";

const char* ShaderBoldLineGen::vertex_shader_source2_ =
"#version 150\n"
"in vec3 vertex_pos;\n"
"in vec3 vertex_color;\n"
"out vec3 color_v;\n"
"void main()\n"
"{\n"
"   color_v = vertex_color;\n"
"   gl_Position = vec4(vertex_pos,1.0);\n"
"}\n";

const char* ShaderBoldLineGen::geometry_shader_source2_ =
"#version 150\n"
"layout (lines) in;\n"
"layout (triangle_strip, max_vertices=6) out;\n"
"in vec3 color_v[];\n"
"out vec4 color_f;\n"
"out vec4 posi_clip;\n"
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
"		posi_clip = gl_in[0].gl_Position;\n"
"		gl_Position = vec4(A.xyz-V, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color_v[1],0.0);\n"
"		posi_clip = gl_in[1].gl_Position;\n"
"		gl_Position = vec4(B.xyz-V, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color_v[0],1.0);\n"
"		posi_clip = gl_in[0].gl_Position;\n"
"		gl_Position = vec4(A.xyz-U, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color_v[1],1.0);\n"
"		posi_clip = gl_in[1].gl_Position;\n"
"		gl_Position = vec4(B.xyz+U, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color_v[0],0.0);\n"
"		posi_clip = gl_in[0].gl_Position;\n"
"		gl_Position = vec4(A.xyz+V, 1.0);\n"
"		EmitVertex();\n"
"		color_f = vec4(color_v[1],0.0);\n"
"		posi_clip = gl_in[1].gl_Position;\n"
"		gl_Position = vec4(B.xyz+V, 1.0);\n"
"		EmitVertex();\n"
"		EndPrimitive();\n"
"	}\n"
"}\n";

const char* ShaderBoldLineGen::fragment_shader_source2_ =
"#version 150\n"
"uniform vec4 plane_clip;\n"
"uniform vec4 plane_clip2;\n"
"in vec4 color_f;\n"
"in vec4 posi_clip;\n"
"out vec4 fragColor;\n"
"void main()\n"
"{\n"
"	float d = dot(plane_clip,posi_clip);\n"
"	float d2 = dot(plane_clip2,posi_clip);\n"
"	if ((d>0.0)||(d2>0.0))  discard;\n"
"   fragColor = color_f;\n"
"}\n";

ShaderBoldLineGen::ShaderBoldLineGen(bool color_per_vertex)
{
	if (color_per_vertex)
	{
		prg_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_source2_);
		prg_.addShaderFromSourceCode(QOpenGLShader::Geometry, geometry_shader_source2_);
		prg_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_source2_);
		prg_.bindAttributeLocation("vertex_pos", ATTRIB_POS);
		prg_.bindAttributeLocation("vertex_color", ATTRIB_COLOR);
	}
	else
	{
		prg_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_source_);
		prg_.addShaderFromSourceCode(QOpenGLShader::Geometry, geometry_shader_source_);
		prg_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_source_);
		prg_.bindAttributeLocation("vertex_pos", ATTRIB_POS);
	}
	prg_.link();
	get_matrices_uniforms();
	unif_color_ = prg_.uniformLocation("lineColor");
	unif_width_ = prg_.uniformLocation("lineWidths");
	unif_plane_clip_ = prg_.uniformLocation("plane_clip");
	unif_plane_clip2_ = prg_.uniformLocation("plane_clip2");
}

void ShaderBoldLineGen::set_color(const QColor& rgb)
{
	if (unif_color_ >= 0)
		prg_.setUniformValue(unif_color_, rgb);
}

void ShaderBoldLineGen::set_width(float32 w)
{
	QOpenGLFunctions* ogl = QOpenGLContext::currentContext()->functions();
	GLint viewport[4];
	ogl->glGetIntegerv(GL_VIEWPORT, viewport);
	QSizeF wd(w / float32(viewport[2]), w / float32(viewport[3]));
	prg_.setUniformValue(unif_width_, wd);
}

void ShaderBoldLineGen::set_plane_clip(const QVector4D& plane)
{
	prg_.setUniformValue(unif_plane_clip_, plane);
}

void ShaderBoldLineGen::set_plane_clip2(const QVector4D& plane)
{
	prg_.setUniformValue(unif_plane_clip2_, plane);
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES)
template class CGOGN_RENDERING_EXPORT ShaderBoldLineTpl<false>;
template class CGOGN_RENDERING_EXPORT ShaderBoldLineTpl<true>;
#endif

ShaderParamBoldLine<false>::~ShaderParamBoldLine()
{}

ShaderParamBoldLine<true>::~ShaderParamBoldLine()
{}



} // namespace rendering

} // namespace cgogn

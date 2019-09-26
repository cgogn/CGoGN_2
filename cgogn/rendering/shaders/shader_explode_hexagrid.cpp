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

#define CGOGN_RENDER_SHADERS_EXPLODE_HEXAGRID_CPP_

#include <iostream>

#include <cgogn/rendering/shaders/shader_explode_hexagrid.h>



namespace cgogn
{

namespace rendering
{

const char* ShaderExplodeHexaGrid::vertex_shader_source2_ =
"#version 150\n"
"in vec3 vertex_pos;\n"
"in vec3 vertex_color;\n"
"out vec3 color_v;\n"
"void main()\n"
"{\n"
"   color_v = vertex_color;\n"
"   gl_Position = vec4(vertex_pos,1.0);\n"
"}\n";

const char* ShaderExplodeHexaGrid::geometry_shader_source2_ =
"#version 150\n"
"layout (lines_adjacency) in;\n"
"layout (triangle_strip, max_vertices=3) out;\n"
"in vec3 color_v[];\n"
"out vec3 color_f;\n"
"uniform mat4 projection_matrix;\n"
"uniform mat4 model_view_matrix;\n"
"uniform mat3 normal_matrix;\n"
"uniform float explode_vol;\n"
"uniform vec3 light_position;\n"
"uniform vec4 plane_clip;\n"
"uniform vec3 planes_clip_topo;\n"
"uniform vec3 planes_clip_topo2;\n"
"uniform vec4 ucolor;\n"
"void main()\n"
"{\n"
"	float d = dot(plane_clip,gl_in[0].gl_Position);\n"
"   bool topo_clip_front = all(greaterThanEqual(color_v[0].xyz,planes_clip_topo.xyz));\n"
"   bool topo_clip_back = all(lessThanEqual(color_v[0].xyz,planes_clip_topo2.xyz));\n"
"	if ((d<=0.0) && topo_clip_front && topo_clip_back)\n"
"	{\n"
"		vec3 v1 = gl_in[2].gl_Position.xyz - gl_in[1].gl_Position.xyz;\n"
"		vec3 v2 = gl_in[3].gl_Position.xyz - gl_in[1].gl_Position.xyz;\n"
"		vec3 N  = normalize(normal_matrix*cross(v1,v2));\n"
"		vec4 face_center =  model_view_matrix * gl_in[1].gl_Position;\n"
"		vec3 L =  normalize (light_position - face_center.xyz);\n"
"		float lambertTerm = abs(dot(N,L));\n"
"		for (int i=1; i<=3; i++)\n"
"		{\n"
"			vec4 Q = explode_vol *  gl_in[i].gl_Position  + (1.0-explode_vol) * gl_in[0].gl_Position;\n"
"			gl_Position = projection_matrix * model_view_matrix *  Q;\n"
"			color_f = (ucolor.rgb+0.0001*color_v[i])*lambertTerm; \n"
"			EmitVertex();\n"
"		}\n"
"		EndPrimitive();\n"
"	}\n"
"}\n";

const char* ShaderExplodeHexaGrid::fragment_shader_source2_ =
"#version 150\n"
"in vec3 color_f;\n"
"out vec3 fragColor;\n"
"void main()\n"
"{\n"
"   fragColor = color_f;\n"
"}\n";

ShaderExplodeHexaGrid::ShaderExplodeHexaGrid()
{
	prg_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_source2_);
	prg_.addShaderFromSourceCode(QOpenGLShader::Geometry, geometry_shader_source2_);
	prg_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_source2_);
	prg_.bindAttributeLocation("vertex_pos", ATTRIB_POS);
	prg_.bindAttributeLocation("vertex_color", ATTRIB_COLOR);
	prg_.link();
	get_matrices_uniforms();
	unif_expl_v_ = prg_.uniformLocation("explode_vol");
	unif_plane_clip_ = prg_.uniformLocation("plane_clip");
	unif_planes_clip_topo_ = prg_.uniformLocation("planes_clip_topo");
	unif_planes_clip_topo2_ = prg_.uniformLocation("planes_clip_topo2");
	unif_light_position_ = prg_.uniformLocation("light_position");
	unif_color_ = prg_.uniformLocation("ucolor");

	// default param
	bind();
	set_light_position(QVector3D(10.0f,100.0f,1000.0f));
	set_explode_volume(0.8f);
	set_plane_clip(QVector4D(0,0,0,0));
	set_planes_clip_topo(QVector3D(0,0,0));
	set_planes_clip_topo2(QVector3D(9999,9999,9999));
	set_color(QColor(0,180,0));
	release();
}

void ShaderExplodeHexaGrid::set_light_position(const QVector3D& l)
{
	prg_.setUniformValue(unif_light_position_, l);
}

void ShaderExplodeHexaGrid::set_explode_volume(float32 x)
{
	prg_.setUniformValue(unif_expl_v_, x);
}

void ShaderExplodeHexaGrid::set_plane_clip(const QVector4D& plane)
{
	prg_.setUniformValue(unif_plane_clip_, plane);
}


void ShaderExplodeHexaGrid::set_planes_clip_topo(const QVector3D& planes)
{
	prg_.setUniformValue(unif_planes_clip_topo_, planes) ;
}

void ShaderExplodeHexaGrid::set_planes_clip_topo2(const QVector3D& planes)
{
	prg_.setUniformValue(unif_planes_clip_topo2_, planes) ;
}

void ShaderExplodeHexaGrid::set_color(const QColor& col)
{
	prg_.setUniformValue(unif_color_, col);
}


std::unique_ptr<typename ShaderExplodeHexaGrid::Param> ShaderExplodeHexaGrid::generate_param()
{
	static ShaderExplodeHexaGrid* instance_ = nullptr;
	if (!instance_)
	{
		instance_ = new ShaderExplodeHexaGrid();
		ShaderProgram::register_instance(instance_);
	}
	return cgogn::make_unique<Param>(instance_);
}


} // namespace rendering

} // namespace cgogn

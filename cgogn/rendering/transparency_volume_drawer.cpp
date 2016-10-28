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

#define CGOGN_RENDERING_TRANSP_VOLUME_RENDER_CPP_

#include <cgogn/rendering/transparency_volume_drawer.h>

#include <QOpenGLFunctions>
#include <iostream>
#include<QColor>
#include<QImage>

namespace cgogn
{

namespace rendering
{


const char* ShaderTransparentVolumes::vertex_shader_source_ =
"#version 330\n"
"in vec3 vertex_pos;\n"
"void main()\n"
"{\n"
"   gl_Position = vec4(vertex_pos,1.0);\n"
"}\n";

const char* ShaderTransparentVolumes::geometry_shader_source_ =
"#version 330\n"
"layout (lines_adjacency) in;\n"
"layout (triangle_strip, max_vertices=3) out;\n"
"out vec3 color_f;\n"
"out float alpha;\n"
"out vec4 projCoord;\n"
"uniform mat4 projection_matrix;\n"
"uniform mat4 model_view_matrix;\n"
"uniform mat3 normal_matrix;\n"
"uniform float explode_vol;\n"
"uniform vec3 light_position;\n"
"uniform vec4 color;\n"
"uniform vec4 plane_clip;\n"
"uniform vec4 plane_clip2;\n"
"uniform bool lighted;\n"
"void main()\n"
"{\n"
"	float d = dot(plane_clip,gl_in[0].gl_Position);\n"
"	float d2 = dot(plane_clip2,gl_in[0].gl_Position);\n"
"	if ((d<=0.0)&&(d2<=0.0))\n"
"	{\n"
"		vec4 face_center =  model_view_matrix * gl_in[1].gl_Position;\n"
"		float lambertTerm = 1.0;\n"
"		if (lighted)\n"
"		{\n"
"			vec3 v1 = gl_in[2].gl_Position.xyz - gl_in[1].gl_Position.xyz;\n"
"			vec3 v2 = gl_in[3].gl_Position.xyz - gl_in[1].gl_Position.xyz;\n"
"			vec3 N  = normalize(normal_matrix*cross(v1,v2));\n"
"			vec3 L =  normalize (light_position - face_center.xyz);\n"
"			lambertTerm = abs(dot(N,L));\n"
"		}\n"
"		for (int i=1; i<=3; i++)\n"
"		{\n"
"			vec4 Q = explode_vol *  gl_in[i].gl_Position  + (1.0-explode_vol) * gl_in[0].gl_Position;\n"
"			gl_Position = projection_matrix * model_view_matrix *  Q;\n"
"			color_f = color.rgb * lambertTerm;\n"
"			alpha = color.a;\n"
"			projCoord = gl_Position;\n"
"			EmitVertex();\n"
"		}\n"
"		EndPrimitive();\n"
"	}\n"
"}\n";

const char* ShaderTransparentVolumes::fragment_shader_source_ =
"#version 330\n"
"layout(location = 0) out vec4 color_out;\n"
"layout(location = 1) out float depth_out;\n"
"in vec4 projCoord;\n"
"in vec3 color_f;\n"
"in float alpha;\n"
"uniform sampler2D rgba_texture ;\n"
"uniform sampler2D depth_texture ;\n"
"uniform int layer;\n"
"uniform bool cull_back_face;\n"


"void main()\n"
"{\n"
"	if (!gl_FrontFacing && cull_back_face) discard;\n"
"	vec3 tc = 0.5*projCoord.xyz/projCoord.w + vec3(0.5,0.5,0.5);\n"
"	if ((layer>0) && (tc.z <= texture(depth_texture, tc.xy).r)) discard;\n"
"	vec4 dst = texture(rgba_texture, tc.xy);\n"
"	color_out = vec4( (dst.a * (alpha * color_f) + dst.rgb), (1.0-alpha)*dst.a);"
"	depth_out = tc.z;\n"
"}\n";



std::unique_ptr<ShaderTransparentVolumes> ShaderTransparentVolumes::instance_ = nullptr;


ShaderTransparentVolumes::ShaderTransparentVolumes()
{
	prg_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_source_);
	prg_.addShaderFromSourceCode(QOpenGLShader::Geometry, geometry_shader_source_);
	prg_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_source_);
	prg_.bindAttributeLocation("vertex_pos", ATTRIB_POS);
	prg_.link();
	get_matrices_uniforms();

	unif_expl_v_ = prg_.uniformLocation("explode_vol");
	unif_plane_clip_ = prg_.uniformLocation("plane_clip");
	unif_plane_clip2_ = prg_.uniformLocation("plane_clip2");
	unif_light_position_ = prg_.uniformLocation("light_position");
	unif_color_ = prg_.uniformLocation("color");

	unif_bf_culling_ = prg_.uniformLocation("cull_back_face");
	unif_lighted_ = prg_.uniformLocation("lighted");
	unif_layer_ = prg_.uniformLocation("layer");
	unif_depth_texture_sampler_ = prg_.uniformLocation("depth_texture");
	unif_rgba_texture_sampler_ = prg_.uniformLocation("rgba_texture");

	// default param
	bind();
	set_light_position(QVector3D(10.0f,100.0f,1000.0f));
	set_explode_volume(0.8f);
	set_color(QColor(255,0,0));
	set_plane_clip(QVector4D(0,0,0,0));
	set_plane_clip2(QVector4D(0,0,0,0));
	release();
}


void ShaderTransparentVolumes::set_color(const QColor& rgb)
{
		prg_.setUniformValue(unif_color_, rgb);
}

void ShaderTransparentVolumes::set_light_position(const QVector3D& l)
{
	prg_.setUniformValue(unif_light_position_, l);
}

void ShaderTransparentVolumes::set_explode_volume(float32 x)
{
	prg_.setUniformValue(unif_expl_v_, x);
}

void ShaderTransparentVolumes::set_plane_clip(const QVector4D& plane)
{
	prg_.setUniformValue(unif_plane_clip_, plane);
}

void ShaderTransparentVolumes::set_plane_clip2(const QVector4D& plane)
{
	prg_.setUniformValue(unif_plane_clip2_, plane);
}

void ShaderTransparentVolumes::set_bf_culling(bool cull)
{
	prg_.setUniformValue(unif_bf_culling_, cull);
}

void ShaderTransparentVolumes::set_lighted(bool lighted)
{
	prg_.setUniformValue(unif_lighted_, lighted);
}

void ShaderTransparentVolumes::set_layer(int layer)
{
	prg_.setUniformValue(unif_layer_, layer);
}

void ShaderTransparentVolumes::set_rgba_sampler(GLuint rgba_samp)
{
	prg_.setUniformValue(unif_rgba_texture_sampler_, rgba_samp);
}

void ShaderTransparentVolumes::set_depth_sampler(GLuint depth_samp)
{
	prg_.setUniformValue(unif_depth_texture_sampler_, depth_samp);
}


std::unique_ptr< ShaderTransparentVolumes::Param> ShaderTransparentVolumes::generate_param()
{
	if (!instance_)
		instance_ = std::unique_ptr<ShaderTransparentVolumes>(new ShaderTransparentVolumes());
	return cgogn::make_unique<ShaderTransparentVolumes::Param>(instance_.get());
}





ShaderParamTransparentVolumes::ShaderParamTransparentVolumes(ShaderTransparentVolumes* sh) :
	ShaderParam(sh),
	color_(255, 0, 0),
	plane_clip_(0, 0, 0, 0),
	plane_clip2_(0, 0, 0, 0),
	light_position_(10.0f, 100.0f, 1000.0f),
	explode_factor_(0.8f),
	layer_(0),
	rgba_texture_sampler_(0),
	depth_texture_sampler_(1),
	bf_culling_(false),
	lighted_(true)
{}

void ShaderParamTransparentVolumes::set_position_vbo(VBO* vbo_pos)
{
	QOpenGLFunctions* ogl = QOpenGLContext::currentContext()->functions();
	shader_->bind();
	vao_->bind();
	vbo_pos->bind();
	ogl->glEnableVertexAttribArray(ShaderTransparentVolumes::ATTRIB_POS);
	ogl->glVertexAttribPointer(ShaderTransparentVolumes::ATTRIB_POS, vbo_pos->vector_dimension(), GL_FLOAT, GL_FALSE, 0, 0);
	vbo_pos->release();
	vao_->release();
	shader_->release();

}

void ShaderParamTransparentVolumes::set_uniforms()
{
	ShaderTransparentVolumes* sh = static_cast<ShaderTransparentVolumes*>(this->shader_);
	sh->set_color(color_);
	sh->set_explode_volume(explode_factor_);
	sh->set_light_position(light_position_);
	sh->set_plane_clip(plane_clip_);
	sh->set_plane_clip2(plane_clip2_);

	sh->set_layer(layer_);
	sh->set_bf_culling(bf_culling_);
	sh->set_lighted(lighted_);
	sh->set_rgba_sampler(rgba_texture_sampler_);
	sh->set_depth_sampler(depth_texture_sampler_);
}









const char* ShaderTranspQuad2::vertex_shader_source_ =
"#version 330\n"
"out vec2 tc;\n"
"void main(void)\n"
"{\n"
"	vec4 vertices[4] = vec4[4](vec4(-1.0, -1.0, 0.0, 1.0), vec4(1.0, -1.0, 0.0, 1.0), vec4(-1.0, 1.0, 0.0, 1.0), vec4(1.0, 1.0, 0.0, 1.0));\n"
"	gl_Position = vertices[gl_VertexID];\n"
"	tc = (gl_Position.xy + vec2(1,1))/2;\n"
"}\n";

const char* ShaderTranspQuad2::fragment_shader_source_ =
"#version 330\n"
"uniform sampler2D rgba_texture;\n"
"uniform sampler2D depth_texture;\n"
"in vec2 tc;\n"
"out vec4 fragColor;\n"
"void main(void)\n"
"{\n"
"	vec4 color = texture(rgba_texture, tc);"
"	if (color.a <= 0.0) discard;"
"	gl_FragDepth = texture(depth_texture, tc).r;\n"
"	fragColor = color;\n"
"}\n";

std::unique_ptr<ShaderTranspQuad2> ShaderTranspQuad2::instance_ = nullptr;

ShaderTranspQuad2::ShaderTranspQuad2()
{
	prg_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_source_);
	prg_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_source_);
	prg_.link();
	unif_rgba_texture_sampler_ =  prg_.uniformLocation("rgba_texture");
	unif_depth_texture_sampler_ = prg_.uniformLocation("depth_texture");
}

void ShaderTranspQuad2::set_rgba_sampler(GLuint rgba_samp)
{
	prg_.setUniformValue(unif_rgba_texture_sampler_, rgba_samp);
}

void ShaderTranspQuad2::set_depth_sampler(GLuint depth_samp)
{
	prg_.setUniformValue(unif_depth_texture_sampler_, depth_samp);
}

std::unique_ptr< ShaderTranspQuad2::Param> ShaderTranspQuad2::generate_param()
{
	if (!instance_)
		instance_ = std::unique_ptr<ShaderTranspQuad2>(new ShaderTranspQuad2());
	return cgogn::make_unique<ShaderTranspQuad2::Param>(instance_.get());
}



ShaderParamTranspQuad2::ShaderParamTranspQuad2(ShaderTranspQuad2* sh) :
	ShaderParam(sh)
{}


void ShaderParamTranspQuad2::set_uniforms()
{
	ShaderTranspQuad2* sh = static_cast<ShaderTranspQuad2*>(this->shader_);
	sh->set_rgba_sampler(rgba_texture_sampler_);
	sh->set_depth_sampler(depth_texture_sampler_);
}



















VolumeTransparencyDrawer::VolumeTransparencyDrawer() :
	vbo_pos_(nullptr),
	face_color_(0,150,0),
	shrink_v_(0.6f)
{
	vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
}



VolumeTransparencyDrawer::Renderer::~Renderer()
{
	param_transp_vol_.reset();
	param_trq_.reset();
	fbo_layer_.reset();
	if (ogl33_)
		ogl33_->glDeleteQueries(1, &oq_transp);


}

VolumeTransparencyDrawer::Renderer::Renderer(VolumeTransparencyDrawer* vr, int w, int h, QOpenGLFunctions_3_3_Core* ogl33) :
	param_transp_vol_(nullptr),
	volume_drawer_data_(vr),
	param_trq_(nullptr),
	max_nb_layers_(8),
	fbo_layer_(nullptr),
	oq_transp(0u),
	ogl33_(ogl33),
	width_(w),
	height_(h)
{
	param_transp_vol_ = ShaderTransparentVolumes::generate_param();
	param_transp_vol_->set_position_vbo(vr->vbo_pos_.get());
	param_transp_vol_->explode_factor_ = vr->shrink_v_;
	param_transp_vol_->color_ = vr->face_color_;
	param_trq_ = cgogn::rendering::ShaderTranspQuad2::generate_param();

	fbo_layer_= cgogn::make_unique<QOpenGLFramebufferObject>(width_,height_,QOpenGLFramebufferObject::Depth,GL_TEXTURE_2D,/*GL_RGBA8*/GL_RGBA32F);
	fbo_layer_->addColorAttachment(width_,height_,GL_R32F);
	fbo_layer_->addColorAttachment(width_,height_,GL_R32F);
	fbo_layer_->addColorAttachment(width_,height_);
	fbo_layer_->addColorAttachment(width_,height_,GL_R32F); // first depth

	ogl33_->glGenQueries(1, &oq_transp);
}

void VolumeTransparencyDrawer::Renderer::resize(int w, int h)
{
	width_ = w;
	height_ = h;

	fbo_layer_= cgogn::make_unique<QOpenGLFramebufferObject>(width_,height_,QOpenGLFramebufferObject::Depth,GL_TEXTURE_2D,/*GL_RGBA8*/GL_RGBA32F);
	fbo_layer_->addColorAttachment(width_,height_,GL_R32F);
	fbo_layer_->addColorAttachment(width_,height_,GL_R32F);
	fbo_layer_->addColorAttachment(width_,height_);
	fbo_layer_->addColorAttachment(width_,height_,GL_R32F); // first depth
}




void VolumeTransparencyDrawer::Renderer::draw_faces(const QMatrix4x4& projection, const QMatrix4x4& modelview)
{
	ogl33_->glEnable(GL_TEXTURE_2D);
	QVector<GLuint> textures = fbo_layer_->textures();
	GLenum buffs[2] = {GL_COLOR_ATTACHMENT0,GL_COLOR_ATTACHMENT2};

	param_transp_vol_->rgba_texture_sampler_ = 0;
	param_transp_vol_->depth_texture_sampler_ = 1;
	fbo_layer_->bind();

	GLenum clear_buff[1] = {GL_COLOR_ATTACHMENT3};
	ogl33_->glDrawBuffers(1,clear_buff);
	ogl33_->glClearColor(0.0f,0.0f,0.0f,1.0f);
	ogl33_->glClear(GL_COLOR_BUFFER_BIT);

	ogl33_->glDrawBuffers(1,buffs);
	ogl33_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	for (int p=0;p<max_nb_layers_;++p)
	{
		ogl33_->glDrawBuffers(1,buffs);
		ogl33_->glClear(GL_DEPTH_BUFFER_BIT);

		ogl33_->glDrawBuffers(2,buffs);
		param_transp_vol_->layer_ = p;
		ogl33_->glActiveTexture(GL_TEXTURE0);
		ogl33_->glBindTexture(GL_TEXTURE_2D,textures[3]);
		ogl33_->glActiveTexture(GL_TEXTURE1);
		ogl33_->glBindTexture(GL_TEXTURE_2D,textures[1]);

		ogl33_->glBeginQuery(GL_SAMPLES_PASSED, oq_transp);

		param_transp_vol_->bind(projection, modelview);
		ogl33_->glDrawArrays(GL_LINES_ADJACENCY, 0, volume_drawer_data_->vbo_pos_->size());
		param_transp_vol_->release();

		ogl33_->glEndQuery(GL_SAMPLES_PASSED);

		GLuint nb_samples;
		ogl33_->glGetQueryObjectuiv(oq_transp, GL_QUERY_RESULT, &nb_samples);

		if (nb_samples==0) // finished ?
		{
			std::cout << "passes "<< p << std::endl;
			p = max_nb_layers_;
		}
		else
		{
			ogl33_->glReadBuffer(GL_COLOR_ATTACHMENT2);
			ogl33_->glBindTexture(GL_TEXTURE_2D,textures[1]);
			ogl33_->glCopyTexImage2D(GL_TEXTURE_2D,0,GL_R32F,0,0,width_,height_,0);

			if (p==0)
			{
				ogl33_->glBindTexture(GL_TEXTURE_2D,textures[4]);
				ogl33_->glCopyTexImage2D(GL_TEXTURE_2D,0,GL_R32F,0,0,width_,height_,0);
			}

			ogl33_->glReadBuffer(GL_COLOR_ATTACHMENT0);
			ogl33_->glBindTexture(GL_TEXTURE_2D,textures[3]);
			ogl33_->glCopyTexImage2D(GL_TEXTURE_2D,0,/*GL_RGBA8*/GL_RGBA32F,0,0,width_,height_,0);
		}
	}

	fbo_layer_->release();

	// real draw with blending with opaque object

	param_trq_->rgba_texture_sampler_ = 0;
	param_trq_->depth_texture_sampler_ = 1;

	ogl33_->glActiveTexture(GL_TEXTURE0);
	ogl33_->glBindTexture(GL_TEXTURE_2D,textures[3]);

	ogl33_->glActiveTexture(GL_TEXTURE1);
	ogl33_->glBindTexture(GL_TEXTURE_2D,textures[4]);

	ogl33_->glEnable(GL_BLEND);
	ogl33_->glBlendFunc(GL_ONE,GL_SRC_ALPHA);
	param_trq_->bind(projection,modelview);
	ogl33_->glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	param_trq_->release();
	ogl33_->glDisable(GL_BLEND);

}




void VolumeTransparencyDrawer::Renderer::set_explode_volume(float32 x)
{
	if (param_transp_vol_)
		param_transp_vol_->explode_factor_ = x;
}

void VolumeTransparencyDrawer::Renderer::set_color(const QColor& rgb)
{
	if (param_transp_vol_)
		param_transp_vol_->color_ = rgb;
}

void VolumeTransparencyDrawer::Renderer::set_clipping_plane(const QVector4D& pl)
{
	if (param_transp_vol_)
		param_transp_vol_->plane_clip_ = pl;
}

void VolumeTransparencyDrawer::Renderer::set_clipping_plane2(const QVector4D& pl)
{
	if (param_transp_vol_)
		param_transp_vol_->plane_clip2_ = pl;
}

void VolumeTransparencyDrawer::Renderer::set_thick_clipping_plane(const QVector4D& p, float32 th)
{
	QVector4D p1 = p;
	p1[3] -= th/2.0f;
	set_clipping_plane(p1);

	QVector4D p2 = -p;
	p2[3] -= th/2.0f;
	set_clipping_plane2(p2);
}

void VolumeTransparencyDrawer::Renderer::set_back_face_culling(bool cull)
{
	param_transp_vol_->bf_culling_ = cull;
}

void VolumeTransparencyDrawer::Renderer::set_lighted(bool lighted)
{
	param_transp_vol_->lighted_=lighted;
}

void VolumeTransparencyDrawer::Renderer::set_max_nb_layers(int nbl)
{
	max_nb_layers_ = nbl;
}



} // namespace rendering

} // namespace cgogn

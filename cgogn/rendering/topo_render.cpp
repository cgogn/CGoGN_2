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

#include <cgogn/rendering/topo_render.h>

#include <QOpenGLFunctions>
#include <iostream>
#include<QColor>

namespace cgogn
{

namespace rendering
{

// static members init
ShaderBoldLine* TopoRender::shader_bl_ = nullptr;
ShaderRoundPoint* TopoRender::shader_rp_ = nullptr;
int32 TopoRender::nb_instances_ = 0;

TopoRender::TopoRender(QOpenGLFunctions_3_3_Core* ogl33):
	ogl33_(ogl33),
	dart_color_(255,255,255),
	phi2_color_(255,0,0),
	phi3_color_(255,255,0),
	shrink_v_(0.6f),
	shrink_f_(0.85f),
	shrink_e_(0.95f)
{
	nb_instances_++;

	vbo_darts_ = new cgogn::rendering::VBO(3);
	vbo_relations_ = new cgogn::rendering::VBO(3);

	if (!shader_bl_)
		shader_bl_ = new ShaderBoldLine();

	param_bl_ = shader_bl_->generate_param();
	param_bl_->set_vbo(vbo_darts_);
	param_bl_->color_= dart_color_;


	param_bl2_ = shader_bl_->generate_param();
	param_bl2_->set_vbo(vbo_relations_);
	param_bl2_->color_= phi2_color_;


	if (!shader_rp_)
		shader_rp_ = new ShaderRoundPoint();

	param_rp_ = shader_rp_->generate_param();
	param_rp_->set_vbo(vbo_darts_,nullptr,2,0);
	param_rp_->color_ = dart_color_;
}

TopoRender::~TopoRender()
{
	delete vbo_darts_;
	delete vbo_relations_;

	nb_instances_--;
	if (nb_instances_ == 0)
	{
		// delete shaders when last TopoRender is deleted
		// ensure context still enable when delete shaders
		delete shader_rp_;
		delete shader_bl_;
	}

	delete param_rp_;
	delete param_bl_;
	delete param_bl2_;

}

void TopoRender::draw(const QMatrix4x4& projection, const QMatrix4x4& modelview, bool with_blending)
{
	float32 lw = 2.0;
	if(with_blending)
	{
		ogl33_->glEnable(GL_BLEND);
		ogl33_->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		lw = 3.0;
	}
//	param_bl_->width_= lw;
//	param_bl2_->width_= lw;
	param_rp_->size_ = 2*lw;

	param_rp_->bind(projection,modelview);
	ogl33_->glDrawArrays(GL_POINTS,0,vbo_darts_->size()/2);
	param_rp_->release();

	shader_bl_->bind();
	shader_bl_->set_matrices(projection,modelview);
	shader_bl_->set_width(lw);
	shader_bl_->set_color(dart_color_);
	param_bl_->bind_vao_only(false);
	ogl33_->glDrawArrays(GL_LINES,0,vbo_darts_->size());
	param_bl_->release_vao_only();

	param_bl2_->bind_vao_only(false);
	shader_bl_->set_color(phi2_color_);
	ogl33_->glDrawArrays(GL_LINES,0,vbo_darts_->size());

	if (vbo_relations_->size() > vbo_darts_->size())
	{
		shader_bl_->set_color(phi3_color_);
		ogl33_->glDrawArrays(GL_LINES,vbo_darts_->size(),vbo_darts_->size());
	}
	param_bl2_->release_vao_only();
	shader_bl_->release();

	ogl33_->glDisable(GL_BLEND);

}




} // namespace rendering

} // namespace cgogn

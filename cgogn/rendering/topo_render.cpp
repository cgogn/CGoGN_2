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

TopoRender::TopoRender():
	dart_color_(255,255,255),
	phi2_color_(255,0,0),
	phi3_color_(255,255,0),
	shrink_v_(0.6f),
	shrink_f_(0.85f),
	shrink_e_(0.95f)
{

	vbo_darts_ = new cgogn::rendering::VBO(3);
	vbo_relations_ = new cgogn::rendering::VBO(3);

	param_bl_ = ShaderBoldLine::generate_param();
	param_bl_->set_vbo(vbo_darts_);
	param_bl_->color_= dart_color_;

	param_bl2_ = ShaderBoldLine::generate_param();
	param_bl2_->set_vbo(vbo_relations_);
	param_bl2_->color_= phi2_color_;

	param_rp_ = ShaderRoundPoint::generate_param();
	param_rp_->set_vbo(vbo_darts_,2,0);
	param_rp_->color_ = dart_color_;
}

TopoRender::~TopoRender()
{
	delete vbo_darts_;
	delete vbo_relations_;

	delete param_rp_;
	delete param_bl_;
	delete param_bl2_;

}

void TopoRender::reinit_vao()
{
	param_bl_->reinit_vao();
	param_bl2_->reinit_vao();
	param_rp_->reinit_vao();

	param_bl_->set_vbo(vbo_darts_);
	param_bl2_->set_vbo(vbo_relations_);
	param_rp_->set_vbo(vbo_darts_,2,0);
}

void TopoRender::draw(const QMatrix4x4& projection, const QMatrix4x4& modelview, QOpenGLFunctions_3_3_Core* ogl33, bool with_blending)
{
	float32 lw = 2.0;
	if(with_blending)
	{
		ogl33->glEnable(GL_BLEND);
		ogl33->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		lw = 3.0;
	}

	param_bl_->width_ = lw;
	param_bl2_->width_ = lw;
	param_rp_->size_ = 2*lw;

	param_rp_->bind(projection,modelview);
	ogl33->glDrawArrays(GL_POINTS,0,vbo_darts_->size()/2);
	param_rp_->release();	

	param_bl_->bind(projection,modelview);
	ogl33->glDrawArrays(GL_LINES,0,vbo_darts_->size());
	param_bl_->release();

	param_bl2_->color_ = phi2_color_;
	param_bl2_->bind(projection,modelview);
	ogl33->glDrawArrays(GL_LINES,0,vbo_darts_->size());
	param_bl2_->release();

	if (vbo_relations_->size() > vbo_darts_->size())
	{
		param_bl2_->color_ = phi3_color_;
		param_bl2_->bind(projection,modelview);
		ogl33->glDrawArrays(GL_LINES,vbo_darts_->size(),vbo_darts_->size());
		param_bl2_->release();
	}

//	shader_bl_->bind();
//	shader_bl_->set_matrices(projection,modelview);
//	shader_bl_->set_width(lw);
//	shader_bl_->set_color(dart_color_);
//	param_bl_->bind_vao_only(false);
//	ogl33->glDrawArrays(GL_LINES,0,vbo_darts_->size());
//	param_bl_->release_vao_only();

//	param_bl2_->bind_vao_only(false);
//	shader_bl_->set_color(phi2_color_);
//	ogl33->glDrawArrays(GL_LINES,0,vbo_darts_->size());

//	if (vbo_relations_->size() > vbo_darts_->size())
//	{
//		shader_bl_->set_color(phi3_color_);
//		ogl33->glDrawArrays(GL_LINES,vbo_darts_->size(),vbo_darts_->size());
//	}
//	param_bl2_->release_vao_only();
//	shader_bl_->release();

	ogl33->glDisable(GL_BLEND);

}


} // namespace rendering

} // namespace cgogn

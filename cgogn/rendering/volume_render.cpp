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

#include <cgogn/rendering/volume_render.h>

#include <QOpenGLFunctions>
#include <iostream>
#include<QColor>

namespace cgogn
{

namespace rendering
{



VolumeRender::VolumeRender():
//	shader_expl_vol_(nullptr),
//	shader_expl_vol_line_(nullptr),
	param_expl_vol_(nullptr),
	param_expl_vol_col_(nullptr),
	param_expl_vol_line_(nullptr),
	vbo_pos_(nullptr),
	vbo_col_(nullptr),
	face_color_(0,150,0),
	vbo_pos2_(nullptr),
	edge_color_(0,0,0),
	shrink_v_(0.6f),
	shrink_f_(0.85f)
{
	vbo_pos_ = new cgogn::rendering::VBO(3);
}

void VolumeRender::reinit_vao()
{

	if (param_expl_vol_)
	{
		param_expl_vol_->reinit_vao();
		param_expl_vol_->set_vbo(vbo_pos_);
	}

	if (param_expl_vol_col_ && vbo_col_)
	{
		param_expl_vol_col_->reinit_vao();
		param_expl_vol_col_->set_vbo(vbo_pos_, vbo_col_);
	}

	if(param_expl_vol_line_)
	{
		param_expl_vol_line_->reinit_vao();
		param_expl_vol_line_->set_vbo(vbo_pos2_);
	}
}


void VolumeRender::init_with_color()
{
	// check if all is already well initialized
	if (vbo_col_!= nullptr)
		return;

	vbo_col_ = new cgogn::rendering::VBO(3);

	delete param_expl_vol_;
	param_expl_vol_col_ = ShaderExplodeVolumesColor::generate_param();
	param_expl_vol_col_->explode_factor_ = shrink_v_;
	param_expl_vol_col_->set_vbo(vbo_pos_,vbo_col_);
}

void VolumeRender::init_without_color()
{
	delete vbo_col_;
	vbo_col_ = nullptr;

	delete param_expl_vol_;
	param_expl_vol_ = ShaderExplodeVolumes::generate_param();
	param_expl_vol_->set_vbo(vbo_pos_);
	param_expl_vol_->explode_factor_ = shrink_v_;
	param_expl_vol_->color_ = face_color_;
}

void VolumeRender::init_edge()
{
	if (vbo_pos2_ != nullptr)
		return;

	vbo_pos2_ = new cgogn::rendering::VBO(3);

	param_expl_vol_line_ = ShaderExplodeVolumesLine::generate_param();
	param_expl_vol_line_->set_vbo(vbo_pos2_);
	param_expl_vol_line_->explode_factor_ = shrink_v_;
	param_expl_vol_line_->color_ = edge_color_;
}


VolumeRender::~VolumeRender()
{
	delete vbo_pos_;
	delete vbo_col_;
	delete param_expl_vol_;
	delete param_expl_vol_line_;

}


void VolumeRender::draw_faces(const QMatrix4x4& projection, const QMatrix4x4& modelview, QOpenGLFunctions_3_3_Core* ogl33)
{
	if (vbo_col_)
	{
		param_expl_vol_col_->bind(projection, modelview);
		ogl33->glDrawArrays(GL_LINES_ADJACENCY, 0, vbo_pos_->size());
		param_expl_vol_col_->release();
	}
	else
	{ 
		param_expl_vol_->bind(projection, modelview);
		ogl33->glDrawArrays(GL_LINES_ADJACENCY, 0, vbo_pos_->size());
		param_expl_vol_->release();
	}
}

void VolumeRender::draw_edges(const QMatrix4x4& projection, const QMatrix4x4& modelview, QOpenGLFunctions_3_3_Core* ogl33)
{

	param_expl_vol_line_->bind(projection,modelview);

	ogl33->glDrawArrays(GL_TRIANGLES,0,vbo_pos2_->size());

	param_expl_vol_line_->release();
}



} // namespace rendering

} // namespace cgogn

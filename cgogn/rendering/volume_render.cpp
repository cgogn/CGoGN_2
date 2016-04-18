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



VolumeRender::VolumeRender(QOpenGLFunctions_3_3_Core* ogl33):
	shader_expl_vol_(nullptr),
	shader_expl_vol_line_(nullptr),
	vbo_col_(nullptr),
	ogl33_(ogl33),
	face_color_(0,150,0),
	edge_color_(0,0,0),
	shrink_v_(0.6f),
	shrink_f_(0.85f)
{
	vbo_pos_ = new cgogn::rendering::VBO(3);
}

void VolumeRender::init_with_color()
{
	if ((vbo_col_!= nullptr) && (shader_expl_vol_!= nullptr))
		return;

	vbo_col_ = new cgogn::rendering::VBO(3);

	delete shader_expl_vol_;
	shader_expl_vol_ = new ShaderExplodeVolumes(true);
	vao1_ = shader_expl_vol_->add_vao();
	shader_expl_vol_->set_vao(vao1_,vbo_pos_,vbo_col_);
}

void VolumeRender::init_without_color()
{
	if ((vbo_col_== nullptr) && (shader_expl_vol_!= nullptr))
		return;

	delete vbo_col_;
	vbo_col_ = nullptr;

	delete shader_expl_vol_;
	shader_expl_vol_ = new ShaderExplodeVolumes(false);
	vao1_ = shader_expl_vol_->add_vao();
	shader_expl_vol_->set_vao(vao1_,vbo_pos_,vbo_col_);
}

void VolumeRender::init_edge()
{
	if (shader_expl_vol_line_!= nullptr)
		return;

	vbo_pos2_ = new cgogn::rendering::VBO(3);

	shader_expl_vol_line_ = new ShaderExplodeVolumesLine();
	vao2_ = shader_expl_vol_line_->add_vao();
	shader_expl_vol_line_->set_vao(vao2_,vbo_pos2_);
}


VolumeRender::~VolumeRender()
{
	delete vbo_pos_;
	delete vbo_col_;
	delete shader_expl_vol_;
}

void VolumeRender::draw_faces(const QMatrix4x4& projection, const QMatrix4x4& modelview)
{
	shader_expl_vol_->bind();
	shader_expl_vol_->set_matrices(projection,modelview);
	shader_expl_vol_->set_explode_volume(shrink_v_);
	shader_expl_vol_->set_color(face_color_);
	shader_expl_vol_->bind_vao(vao1_);
	ogl33_->glDrawArrays(GL_LINES_ADJACENCY,0,vbo_pos_->size());
	shader_expl_vol_->release_vao(vao1_);
	shader_expl_vol_->release();
}

void VolumeRender::draw_edges(const QMatrix4x4& projection, const QMatrix4x4& modelview)
{
	shader_expl_vol_line_->bind();
	shader_expl_vol_line_->set_matrices(projection,modelview);
	shader_expl_vol_line_->set_explode_volume(shrink_v_);
	shader_expl_vol_line_->set_color(edge_color_);
	shader_expl_vol_line_->bind_vao(vao2_);
	ogl33_->glDrawArrays(GL_TRIANGLES,0,vbo_pos2_->size());
	shader_expl_vol_line_->release_vao(vao2_);
	shader_expl_vol_line_->release();
}



} // namespace rendering

} // namespace cgogn

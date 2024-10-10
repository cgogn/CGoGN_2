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

#define CGOGN_RENDERING_VOLUME_RENDER_CPP_

#include <cgogn/rendering/hexagrid_drawer.h>

#include <QOpenGLFunctions>
#include <iostream>
#include<QColor>

namespace cgogn
{

namespace rendering
{

HexaGridDrawer::HexaGridDrawer() :
	vbo_pos_(nullptr),
	vbo_col_(nullptr),
	face_color_(0,150,100),
	vbo_pos2_(nullptr),
	edge_color_(0,0,0),
	shrink_v_(0.6f)
{
	vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
	vbo_pos2_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
	vbo_col_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
}

HexaGridDrawer::~HexaGridDrawer()
{}

HexaGridDrawer::Renderer::Renderer(HexaGridDrawer* vr) :
	param_expl_hg_(nullptr),
	param_expl_hg_line_(nullptr),
	hg_drawer_data_(vr)
{
	param_expl_hg_ = ShaderExplodeHexaGrid::generate_param();
	param_expl_hg_->set_position_vbo(vr->vbo_pos_.get());
	param_expl_hg_->set_color_vbo(vr->vbo_col_.get());
	param_expl_hg_->explode_factor_ = vr->shrink_v_;
	param_expl_hg_->color_ = vr->face_color_;

	if (vr->vbo_pos2_)
	{
		param_expl_hg_line_ = ShaderExplodeHexaGridLine::generate_param();
		param_expl_hg_line_->set_position_vbo(vr->vbo_pos2_.get());
		param_expl_hg_line_->explode_factor_ = vr->shrink_v_;
		param_expl_hg_line_->color_ = vr->edge_color_;
	}
}

HexaGridDrawer::Renderer::~Renderer()
{}

void HexaGridDrawer::Renderer::draw_faces(const QMatrix4x4& projection, const QMatrix4x4& modelview)
{
	QOpenGLFunctions_3_3_Core * ogl33 = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_3_3_Core>();

	param_expl_hg_->bind(projection, modelview);
	ogl33->glDrawArrays(GL_LINES_ADJACENCY, 0, hg_drawer_data_->vbo_pos_->size());
	param_expl_hg_->release();
}

void HexaGridDrawer::Renderer::draw_edges(const QMatrix4x4& projection, const QMatrix4x4& modelview)
{
	QOpenGLFunctions_3_3_Core * ogl33 = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_3_3_Core>();

	param_expl_hg_line_->bind(projection,modelview);
	ogl33->glDrawArrays(GL_LINES_ADJACENCY, 0, hg_drawer_data_->vbo_pos2_->size());
	param_expl_hg_line_->release();
}

void HexaGridDrawer::Renderer::set_explode_volume(float32 x)
{
	if (param_expl_hg_)
		param_expl_hg_->explode_factor_ = x;
	if (param_expl_hg_line_)
		param_expl_hg_line_->explode_factor_ = x;
}

void HexaGridDrawer::Renderer::set_edge_color(const QColor& rgb)
{
	if (param_expl_hg_line_)
		param_expl_hg_line_->color_=rgb;
}

void HexaGridDrawer::Renderer::set_face_color(const QColor& rgb)
{
	if (param_expl_hg_)
		param_expl_hg_->color_=rgb;
}

void HexaGridDrawer::Renderer::set_clipping_plane(const QVector4D& pl)
{
	if (param_expl_hg_)
		param_expl_hg_->plane_clip_ = pl;
	if (param_expl_hg_line_)
		param_expl_hg_line_->plane_clip_ = pl;
}

void HexaGridDrawer::Renderer::set_clipping_plane_topo(const QVector3D& pl)
{
	if (param_expl_hg_)
		param_expl_hg_->planes_clip_topo_ = pl;
	if (param_expl_hg_line_)
		param_expl_hg_line_->planes_clip_topo_ = pl;
}

void HexaGridDrawer::Renderer::set_clipping_plane_topo2(const QVector3D& pl)
{
	if (param_expl_hg_)
		param_expl_hg_->planes_clip_topo2_ = pl;
	if (param_expl_hg_line_)
		param_expl_hg_line_->planes_clip_topo2_ = pl;
}

} // namespace rendering

} // namespace cgogn

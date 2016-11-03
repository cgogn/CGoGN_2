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


#include <cgogn/rendering/transparency_drawer.h>

namespace cgogn
{

namespace rendering
{

FlatTransparencyDrawer::~FlatTransparencyDrawer()
{
	param_flat_.reset();
	param_trq_.reset();
	fbo_layer_.reset();
	if (ogl33_)
		ogl33_->glDeleteQueries(1, &oq_transp);
}

FlatTransparencyDrawer::FlatTransparencyDrawer(int w, int h, QOpenGLFunctions_3_3_Core* ogl33):
	max_nb_layers_(8),
	param_flat_(nullptr),
	param_trq_(nullptr),
	fbo_layer_(nullptr),
	oq_transp(0u),
	ogl33_(ogl33),
	width_(w),
	height_(h)
{
	param_flat_ = cgogn::rendering::ShaderFlatTransp::generate_param();
	param_flat_->front_color_ = QColor(0,250,0,120);
	param_flat_->back_color_ = QColor(0,0,250,120);
	param_flat_->ambiant_color_ = QColor(0,0,0,0);

	param_trq_ = cgogn::rendering::ShaderTranspQuad::generate_param();

	fbo_layer_= cgogn::make_unique<QOpenGLFramebufferObject>(width_,height_,QOpenGLFramebufferObject::Depth,GL_TEXTURE_2D,/*GL_RGBA8*/GL_RGBA32F);
	fbo_layer_->addColorAttachment(width_,height_,GL_R32F);
	fbo_layer_->addColorAttachment(width_,height_,GL_R32F);
	fbo_layer_->addColorAttachment(width_,height_);
	fbo_layer_->addColorAttachment(width_,height_,GL_R32F); // first depth

	ogl33_->glGenQueries(1, &oq_transp);
}

void FlatTransparencyDrawer::resize(int w, int h)
{
	width_ = w;
	height_ = h;

	fbo_layer_= cgogn::make_unique<QOpenGLFramebufferObject>(width_,height_,QOpenGLFramebufferObject::Depth,GL_TEXTURE_2D,/*GL_RGBA8*/GL_RGBA32F);
	fbo_layer_->addColorAttachment(width_,height_,GL_R32F);
	fbo_layer_->addColorAttachment(width_,height_,GL_R32F);
	fbo_layer_->addColorAttachment(width_,height_);
	fbo_layer_->addColorAttachment(width_,height_,GL_R32F); // first depth
}




} // namespace rendering

} // namespace cgogn

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


#include <cgogn/rendering/wall_paper.h>

#include <QOpenGLFunctions>
#include <QColor>
#include <iostream>


namespace cgogn
{

namespace rendering
{

void WallPaper::init(const QImage& img)
{
	if (vbo_pos_ == nullptr)
	{
		vbo_pos_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
		vbo_pos_->allocate(4, 3);
	}
	if (vbo_tc_ == nullptr)
	{
		vbo_tc_ = cgogn::make_unique<cgogn::rendering::VBO>(2);
		vbo_tc_->allocate(4, 2);
		float32* ptr_tc = vbo_tc_->lock_pointer();
		*ptr_tc++ = 0.0f;
		*ptr_tc++ = 0.0f;
		*ptr_tc++ = 1.0f;
		*ptr_tc++ = 0.0f;
		*ptr_tc++ = 1.0f;
		*ptr_tc++ = 1.0f;
		*ptr_tc++ = 0.0f;
		*ptr_tc++ = 1.0f;
		vbo_tc_->release_pointer();
	}
	texture_ = cgogn::make_unique<QOpenGLTexture>(img,QOpenGLTexture::DontGenerateMipMaps);
	set_full_screen(false);
}

WallPaper::WallPaper(const QImage& img):
	vbo_pos_(nullptr),
	vbo_tc_(nullptr),
	texture_(nullptr)
{
	init(img);
}

WallPaper::WallPaper(const QColor& col) :
	vbo_pos_(nullptr),
	vbo_tc_(nullptr),
	texture_(nullptr)
{
	QImage img(1, 1, QImage::Format_RGB32);
	img.setPixel(0, 0, col.rgba());
	init(img);
}

WallPaper::WallPaper(const QColor& col_tl, const QColor& col_tr, const QColor& col_bl, const QColor& col_br) :
	vbo_pos_(nullptr),
	vbo_tc_(nullptr),
	texture_(nullptr)
{
	QImage img(2, 2, QImage::Format_RGB32);
	img.setPixel(0, 1, col_bl.rgba());
	img.setPixel(1, 1, col_br.rgba());
	img.setPixel(1, 0, col_tr.rgba());
	img.setPixel(0, 0, col_tl.rgba());
	init(img);
	texture_->setWrapMode(QOpenGLTexture::ClampToEdge);
}


WallPaper::~WallPaper()
{
}


void WallPaper::change_color(const QColor& col)
{
	if ((texture_->width()==1)&&(texture_->height()==1))
	{
		float32 color[3] = {float32(col.red()), float32(col.green()), float32(col.blue())};
		texture_->setData(QOpenGLTexture::RGB, QOpenGLTexture::Float32, color);
	}
	else
	{
		cgogn_log_warning("change colors")<< "Attemping to change color with a wall paper initialized with more than one color";
	}
}

void WallPaper::change_colors(const QColor& col_tl, const QColor& col_tr, const QColor& col_bl, const QColor& col_br)
{
	if ((texture_->width()==2)&&(texture_->height()==2))
	{
		float32 colors[12] =
		{float32(col_tl.red()), float32(col_tl.green()), float32(col_tl.blue()),
		 float32(col_tr.red()), float32(col_tr.green()), float32(col_tr.blue()),
		 float32(col_bl.red()), float32(col_bl.green()), float32(col_bl.blue()),
		 float32(col_br.red()), float32(col_br.green()), float32(col_br.blue())
		 };

		texture_->setData(QOpenGLTexture::RGB, QOpenGLTexture::Float32, colors);
		texture_->setWrapMode(QOpenGLTexture::ClampToEdge);
	}
	else
	{
		cgogn_log_warning("change colors")<< "Attemping to change colors with a wall paper not initialized with 4 colors";
	}
}



void WallPaper::set_full_screen(bool front)
{
	float32 depth = 0.9999999f;
	if (front)
		depth = 0.0f;

	float32* ptr_pos = vbo_pos_->lock_pointer();
	*ptr_pos++ = -1.0f;
	*ptr_pos++ =  1.0f;
	*ptr_pos++ =  depth;
	*ptr_pos++ =  1.0f;
	*ptr_pos++ =  1.0f;
	*ptr_pos++ =  depth;
	*ptr_pos++ =  1.0f;
	*ptr_pos++ = -1.0f;
	*ptr_pos++ =  depth;
	*ptr_pos++ = -1.0f;
	*ptr_pos++ = -1.0f;
	*ptr_pos++ =  depth;
	vbo_pos_->release_pointer();
}

void WallPaper::set_local_position(uint32 win_w, uint32 win_h, uint32 x, uint32 y, uint32 w, uint32 h, bool front)
{
	float32 depth = 0.0f;
	if (!front)
		depth = 0.9999999f;

	float32 xmin = -1.0f + float32(2*x)/float32(win_w);
	float32 xmax = xmin + float32(2*w)/float32(win_w);

	float32 ymin = 1.0f - float32(2*y)/float32(win_h);
	float32 ymax = ymin - float32(2*h)/float32(win_h);

	float32* ptr_pos = vbo_pos_->lock_pointer();
	*ptr_pos++ = xmin;
	*ptr_pos++ = ymin;
	*ptr_pos++ = depth;
	*ptr_pos++ = xmax;
	*ptr_pos++ = ymin;
	*ptr_pos++ = depth;
	*ptr_pos++ = xmax;
	*ptr_pos++ = ymax;
	*ptr_pos++ = depth;
	*ptr_pos++ = xmin;
	*ptr_pos++ = ymax;
	*ptr_pos++ = depth;
	vbo_pos_->release_pointer();
}

void WallPaper::set_local_position(float x, float y, float w, float h, bool front)
{
	float32 depth = 0.0f;
	if (!front)
		depth = 0.9999999f;

	float32 xmin = -1.0f + 2*x;
	float32 xmax = xmin + 2*w;

	float32 ymin = 1.0f - 2*y;
	float32 ymax = ymin - 2*h;

	float32* ptr_pos = vbo_pos_->lock_pointer();
	*ptr_pos++ = xmin;
	*ptr_pos++ = ymin;
	*ptr_pos++ = depth;
	*ptr_pos++ = xmax;
	*ptr_pos++ = ymin;
	*ptr_pos++ = depth;
	*ptr_pos++ = xmax;
	*ptr_pos++ = ymax;
	*ptr_pos++ = depth;
	*ptr_pos++ = xmin;
	*ptr_pos++ = ymax;
	*ptr_pos++ = depth;
	vbo_pos_->release_pointer();
}

WallPaper::Renderer::Renderer(WallPaper* wp) :
	wall_paper_data_(wp)
{
	param_texture_ = ShaderTexture::generate_param();
	param_texture_->set_vbo(wp->vbo_pos_.get(), wp->vbo_tc_.get());
	param_texture_->texture_ = wp->texture_.get();
}

WallPaper::Renderer::~Renderer()
{}

void WallPaper::Renderer::draw(QOpenGLFunctions_3_3_Core* ogl33)
{
	QMatrix4x4 id;
	param_texture_->bind(id, id);
	ogl33->glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
	param_texture_->release();
}

} // namespace rendering

} // namespace cgogn

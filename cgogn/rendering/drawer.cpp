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

#include <cgogn/rendering/drawer.h>

#include <QOpenGLFunctions>
#include <iostream>
#include<QColor>

namespace cgogn
{

namespace rendering
{

// static members init
ShaderColorPerVertex* Drawer::shader_cpv_ = nullptr;
ShaderBoldLine* Drawer::shader_bl_ = nullptr;
ShaderRoundPoint* Drawer::shader_rp_ = nullptr;
ShaderPointSprite* Drawer::shader_ps_ = nullptr;
int32 Drawer::nb_instances_ = 0;

Drawer::Drawer():
	current_size_(1.0f),
	current_aa_(true),
	current_ball_(true)
{
	nb_instances_++;

	vbo_pos_ = new VBO(3);
	vbo_col_ = new VBO(3);
	if (!shader_cpv_)
		shader_cpv_ = new ShaderColorPerVertex();

	if (!shader_bl_)
		shader_bl_ = new ShaderBoldLine(true);

	if (!shader_rp_)
		shader_rp_ = new ShaderRoundPoint(true);

	if (!shader_ps_)
		shader_ps_ = new ShaderPointSprite(true);

	param_cpv_ = shader_cpv_->generate_param();
	param_bl_ = shader_bl_->generate_param();
	param_rp_ = shader_rp_->generate_param();
	param_ps_ = shader_ps_->generate_param();

	param_cpv_->set_vbo(vbo_pos_,vbo_col_);
	param_bl_->set_vbo(vbo_pos_,vbo_col_);
	param_rp_->set_vbo(vbo_pos_,vbo_col_);
	param_ps_->set_vbo(vbo_pos_,vbo_col_);
}

void Drawer::reinit_vao()
{
	param_cpv_->reinit_vao();
	param_bl_->reinit_vao();
	param_rp_->reinit_vao();
	param_ps_->reinit_vao();

	param_cpv_->set_vbo(vbo_pos_,vbo_col_);
	param_bl_->set_vbo(vbo_pos_,vbo_col_);
	param_rp_->set_vbo(vbo_pos_,vbo_col_);
	param_ps_->set_vbo(vbo_pos_,vbo_col_);
}

Drawer::~Drawer()
{
	delete vbo_pos_;
	delete vbo_col_;

	nb_instances_--;
	if (nb_instances_ == 0)
	{
		// delete shaders when last drawer is deleted
		// ensure context still enable when delete shaders
		delete shader_ps_;
		delete shader_rp_;
		delete shader_bl_;
		delete shader_cpv_;
	}
}

void Drawer::new_list()
{
	data_pos_.clear();
	data_col_.clear();
	begins_point_.clear();
	begins_round_point_.clear();
	begins_balls_.clear();
	begins_line_.clear();
	begins_bold_line_.clear();
	begins_face_.clear();
}

void Drawer::begin(GLenum mode)
{
	switch (mode)
	{
	case GL_POINTS:
		if (current_ball_)
		{
			begins_balls_.push_back(PrimParam(data_pos_.size(), mode, current_size_,false));
			current_begin_ = &begins_balls_;
		}
		else if (current_size_ > 2.0)
		{
			begins_round_point_.push_back(PrimParam(data_pos_.size(), mode, current_size_,current_aa_));
			current_begin_ = &begins_round_point_;
		}
		else
		{
			begins_point_.push_back(PrimParam(data_pos_.size(), mode, current_size_,false));
			current_begin_ = &begins_point_;
		}
		break;
	case GL_LINES:
	case GL_LINE_STRIP:
	case GL_LINE_LOOP:
		if (current_size_ > 1.0)
		{
			begins_bold_line_.push_back(PrimParam(data_pos_.size(), mode, current_size_,current_aa_));
			current_begin_ = &begins_bold_line_;
		}
		else
		{
			begins_line_.push_back(PrimParam(data_pos_.size(), mode, 1.0,current_aa_));
			current_begin_ = &begins_line_;
		}
		break;
	default:
		begins_face_.push_back(PrimParam(data_pos_.size(), mode, 1.0f,false));
		current_begin_ = &begins_face_;
		break;
	}
}

void Drawer::end()
{
	current_begin_->back().nb = uint32(data_pos_.size() - current_begin_->back().begin);
}

void Drawer::vertex3f(float32 x, float32 y, float32 z)
{
	if (data_pos_.size() == data_col_.size())
	{
		if (data_col_.empty())
			data_col_.push_back(Vec3f{1.0f, 1.0f, 1.0f});
		else
			data_col_.push_back( data_col_.back());
	}
	data_pos_.push_back(Vec3f{x,y,z});
}

void Drawer::color3f(float32 r, float32 g, float32 b)
{
	if (data_pos_.size() == data_col_.size())
		data_col_.push_back(Vec3f{r,g,b});
	else
		data_col_.back() = Vec3f{r,g,b};
}

void Drawer::end_list()
{
	uint32 nb_elts = uint32(data_pos_.size());

	if (nb_elts == 0)
		return;

	vbo_pos_->allocate(nb_elts,3);
	float32* ptr = vbo_pos_->lock_pointer();
	std::memcpy(ptr,data_pos_[0].data(),nb_elts*12);
	vbo_pos_->release_pointer();

	vbo_col_->allocate(nb_elts,3);
	ptr = vbo_col_->lock_pointer();
	std::memcpy(ptr,data_col_[0].data(),nb_elts*12);
	vbo_col_->release_pointer();

	// free memory
	data_pos_.clear();
	data_pos_.shrink_to_fit();
	data_col_.clear();
	data_col_.shrink_to_fit();
}

void Drawer::call_list(const QMatrix4x4& projection, const QMatrix4x4& modelview, QOpenGLFunctions_3_3_Core* ogl33)
{
	//classic rendering
	if (!begins_point_.empty() || !begins_line_.empty() || !begins_face_.empty())
	{
		param_cpv_->bind(projection,modelview);

		for (auto& pp : begins_point_)
		{
			ogl33->glPointSize(pp.width);
			ogl33->glDrawArrays(pp.mode, pp.begin, pp.nb);
		}

		for (auto& pp : begins_line_)
		{
			ogl33->glDrawArrays(pp.mode, pp.begin, pp.nb);
		}

		for (auto& pp : begins_face_)
		{
			ogl33->glDrawArrays(pp.mode, pp.begin, pp.nb);
		}

		param_cpv_->release();
	}

	// balls
	if (!begins_balls_.empty())
	{
		param_ps_->bind(projection,modelview);

		for (auto& pp : begins_balls_)
		{
			shader_ps_->set_size(pp.width);
			ogl33->glDrawArrays(pp.mode, pp.begin, pp.nb);
		}
		param_ps_->release();
	}

	// round points
	if (!begins_round_point_.empty())
	{
		param_rp_->bind(projection,modelview);

		for (auto& pp : begins_round_point_)
		{
			if (pp.aa)
			{
				ogl33->glEnable(GL_BLEND);
				ogl33->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			}

			shader_rp_->set_size(pp.width);
			ogl33->glDrawArrays(pp.mode, pp.begin, pp.nb);

			if (pp.aa)
				ogl33->glDisable(GL_BLEND);
		}
		param_rp_->release();
	}

	// bold lines
	if (!begins_bold_line_.empty())
	{
		param_bl_->bind(projection,modelview);

		for (auto& pp : begins_bold_line_)
		{
			shader_bl_->set_width(pp.width);
			shader_bl_->set_color(QColor(255,255,0));

			if (pp.aa)
			{
				ogl33->glEnable(GL_BLEND);
				ogl33->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			}

			ogl33->glDrawArrays(pp.mode, pp.begin, pp.nb);

			if (pp.aa)
				ogl33->glDisable(GL_BLEND);
		}

		param_bl_->release();
	}
}

} // namespace rendering

} // namespace cgogn

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

#include <rendering/drawer.h>

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
int Drawer::nb_instances_ = 0;

Drawer::Drawer(QOpenGLFunctions_3_3_Core* ogl33):
	current_size_(1.0f),
	current_aa_(true),
	ogl33_(ogl33)
{
	nb_instances_++;

	vbo_pos_ = new VBO(3);
	vbo_col_ = new VBO(3);
	if (!shader_cpv_)
		shader_cpv_ = new ShaderColorPerVertex();

	vao_cpv_ = shader_cpv_->add_vao();
	shader_cpv_->set_vao(vao_cpv_,vbo_pos_,vbo_col_);

	if (!shader_bl_)
		shader_bl_ = new ShaderBoldLine(true);
	vao_bl_ = shader_bl_->add_vao();
	shader_bl_->bind();
	shader_bl_->release();
	shader_bl_->set_vao(vao_bl_,vbo_pos_,vbo_col_);

	if (!shader_rp_)
		shader_rp_ = new ShaderRoundPoint(true);
	vao_rp_ = shader_rp_->add_vao();
	shader_rp_->bind();
	shader_rp_->release();
	shader_rp_->set_vao(vao_rp_,vbo_pos_,vbo_col_);


}

Drawer::~Drawer()
{
	delete vbo_pos_;
	delete vbo_col_;

	nb_instances_--;
	if (nb_instances_ ==0)
	{
		// delete shaders when last drawer is deleted
		// ensure context still enable when delete shaders
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
	begins_line_.clear();
	begins_bold_line_.clear();
	begins_face_.clear();
}

void Drawer::begin(GLenum mode)
{
	switch (mode)
	{
	case GL_POINTS:
		if (current_size_ > 2.0)
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
	current_begin_->back().nb = static_cast<unsigned int>(data_pos_.size() - current_begin_->back().begin);
}


void Drawer::vertex3f(float x, float y, float z)
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


void Drawer::color3f(float r, float g, float b)
{
	if (data_pos_.size() == data_col_.size())
		data_col_.push_back(Vec3f{r,g,b});
	else
		data_col_.back() = Vec3f{r,g,b};
}


void Drawer::end_list()
{
	unsigned int nb_elts = static_cast<unsigned int>(data_pos_.size());

	if (nb_elts == 0)
		return;

	vbo_pos_->allocate(nb_elts,3);
	float* ptr = vbo_pos_->lock_pointer();
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

void Drawer::callList(const QMatrix4x4& projection, const QMatrix4x4& modelview)
{

	//classic rendering
	shader_cpv_->bind();
	shader_cpv_->set_matrices(projection,modelview);
	shader_cpv_->bind_vao(vao_cpv_);

	for (auto& pp : begins_point_)
	{
		ogl33_->glPointSize(pp.width);
		ogl33_->glDrawArrays(pp.mode, pp.begin, pp.nb);
	}

	for (auto& pp : begins_line_)
	{
		ogl33_->glDrawArrays(pp.mode, pp.begin, pp.nb);
	}

	for (auto& pp : begins_face_)
	{
		ogl33_->glDrawArrays(pp.mode, pp.begin, pp.nb);
	}

	shader_cpv_->release_vao(vao_cpv_);
	shader_cpv_->release();


	// round points

	shader_rp_->bind();
	shader_rp_->set_matrices(projection,modelview);
	shader_rp_->bind_vao(vao_bl_);

	for (auto& pp : begins_round_point_)
	{
		if (pp.aa)
		{
			ogl33_->glEnable(GL_BLEND);
			ogl33_->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		}

		shader_rp_->set_width(pp.width);
		ogl33_->glDrawArrays(pp.mode, pp.begin, pp.nb);

		if (pp.aa)
			ogl33_->glDisable(GL_BLEND);

	}
	shader_rp_->release_vao(vao_bl_);
	shader_rp_->release();


	// bold lines

	shader_bl_->bind();
	shader_bl_->set_matrices(projection,modelview);
	shader_bl_->bind_vao(vao_bl_);

	for (auto& pp : begins_bold_line_)
	{
		shader_bl_->set_width(pp.width);
		shader_bl_->set_color(QColor(255,255,0));

		if (pp.aa)
		{
			ogl33_->glEnable(GL_BLEND);
			ogl33_->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		}

		ogl33_->glDrawArrays(pp.mode, pp.begin, pp.nb);

		if (pp.aa)
			ogl33_->glDisable(GL_BLEND);

	}

	shader_bl_->release_vao(vao_bl_);
	shader_bl_->release();


}




} // namespace rendering

} // namespace cgogn

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

namespace cgogn
{

namespace rendering
{

ShaderColorPerVertex* Drawer::shader_cpv_= NULL;

Drawer::Drawer() :
	current_size_(1.0f)
{
	vbo_pos_ = new VBO(3);
	vbo_col_ = new VBO(3);

	if (shader_cpv_ == NULL)
		shader_cpv_ = new ShaderColorPerVertex();


	vao_ = shader_cpv_->add_vao();
	shader_cpv_->set_vao(vao_,vbo_pos_,vbo_col_);

}

Drawer::~Drawer()
{

}

void Drawer::new_list()
{
	data_pos_.clear();
	data_col_.clear();
	begins_.clear();
}

void Drawer::begin(GLenum mode)
{
	if (mode == GL_POINTS)
		begins_.push_back(PrimParam(data_pos_.size(), mode, current_size_));
	else
		begins_.push_back(PrimParam(data_pos_.size(), mode, 1.0));
}

void Drawer::end()
{
	begins_.back().nb = data_pos_.size() - begins_.back().begin;
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
	unsigned int nb_elts(data_pos_.size());

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

	for (const auto& beg : begins_)
	{
		switch (beg.mode)
		{
		case GL_POINTS:
			begins_point_.push_back(beg);
			break;
		case GL_LINES:
		case GL_LINE_STRIP:
		case GL_LINE_LOOP:
			begins_line_.push_back(beg);
			break;
		default:
			begins_face_.push_back(beg);
			break;
		}
	}
}

void Drawer::callList(const QMatrix4x4& projection, const QMatrix4x4& modelview)
{
	QOpenGLFunctions *ogl = QOpenGLContext::currentContext()->functions();

	if (begins_.empty())
		return;

	shader_cpv_->bind();
	shader_cpv_->set_matrices(projection,modelview);
	shader_cpv_->bind_vao(vao_);
	for (auto& pp : begins_point_)
	{
		glPointSize(pp.width);
		ogl->glDrawArrays(pp.mode, pp.begin, pp.nb);
	}

	for (auto& pp : begins_line_)
	{
		glLineWidth(3.0);
		ogl->glDrawArrays(pp.mode, pp.begin, pp.nb);
	}

	for (auto& pp : begins_face_)
	{
		ogl->glDrawArrays(pp.mode, pp.begin, pp.nb);
	}

	shader_cpv_->release_vao(vao_);
	shader_cpv_->release();

}




} // namespace rendering

} // namespace cgogn

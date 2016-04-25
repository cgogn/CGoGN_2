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

#ifndef CGOGN_RENDERING_SHADERS_ROUND_POINT_H_
#define CGOGN_RENDERING_SHADERS_ROUND_POINT_H_

#include <cgogn/rendering/dll.h>
#include <cgogn/rendering/shaders/shader_program.h>
#include <cgogn/rendering/shaders/vbo.h>

#include <QOpenGLFunctions>
#include <QColor>


namespace cgogn
{

namespace rendering
{


class CGOGN_RENDERING_API ShaderRoundPointGen : public ShaderProgram
{
	static const char* vertex_shader_source_;
	static const char* geometry_shader_source_;
	static const char* fragment_shader_source_;

	static const char* vertex_shader_source2_;
	static const char* geometry_shader_source2_;
	static const char* fragment_shader_source2_;

	// uniform ids
	GLint unif_color_;
	GLint unif_size_;

public:
	using Self = ShaderRoundPointGen;
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ShaderRoundPointGen);

	enum
	{
		ATTRIB_POS = 0,
		ATTRIB_COLOR
	};

	ShaderRoundPointGen(bool color_per_vertex = false);

	/**
	 * @brief set current color
	 * @param rgb
	 */
	void set_color(const QColor& rgb);

	/**
	 * @brief set the width of lines (call before each draw)
	 * @param w width in pixel
	 */
	void set_size(float32 w);
};


template <bool CPV>
class ShaderParamRoundPoint: public ShaderParam
{};

template <bool CPV>
class ShaderRoundPointTpl : public ShaderRoundPointGen
{
public:
	using Param = ShaderParamRoundPoint<CPV>;
	static Param* generate_param();
private:
	ShaderRoundPointTpl():
		ShaderRoundPointGen(CPV)
	{}
	static ShaderRoundPointTpl* instance_;
};

template <bool CPV>
ShaderRoundPointTpl<CPV>* ShaderRoundPointTpl<CPV>::instance_ = nullptr;


// COLOR UNIFORM PARAM
template <>
class ShaderParamRoundPoint<false> : public ShaderParam
{
protected:

	void set_uniforms() override
	{
		ShaderRoundPointGen* sh = static_cast<ShaderRoundPointGen*>(this->shader_);
		sh->set_color(color_);
		sh->set_size(size_);
	}

public:
	QColor color_;
	float32 size_;

	ShaderParamRoundPoint(ShaderRoundPointTpl<false>* sh):
		ShaderParam(sh), // cast because type of sh in only a forward
		color_(0, 0, 255),
		size_(1.0)
	{}

	void set_vbo(VBO* vbo_pos, uint32 stride = 0, uint32 first = 0)
	{
		QOpenGLFunctions *ogl = QOpenGLContext::currentContext()->functions();
		shader_->bind();
		vao_->bind();
		// position vbo
		vbo_pos->bind();
		ogl->glEnableVertexAttribArray(ShaderRoundPointGen::ATTRIB_POS);
		ogl->glVertexAttribPointer(ShaderRoundPointGen::ATTRIB_POS, vbo_pos->vector_dimension(), GL_FLOAT, GL_FALSE, stride * vbo_pos->vector_dimension() * 4, void_ptr(first * vbo_pos->vector_dimension() * 4));
		vbo_pos->release();
		vao_->release();
		shader_->release();
	}
};


// COLOR PER VERTEX PARAM
template <>
class ShaderParamRoundPoint<true> : public ShaderParam
{
protected:

	void set_uniforms() override
	{
		ShaderRoundPointGen* sh = static_cast<ShaderRoundPointGen*>(this->shader_);
		sh->set_size(size_);
	}

public:
	float32 size_;

	ShaderParamRoundPoint(ShaderRoundPointTpl<true>* sh):
		ShaderParam(sh), // cast because type of sh in only a forward
		size_(1.0)
	{}

	void set_vbo(VBO* vbo_pos, VBO* vbo_color, uint32 stride = 0, uint32 first = 0)
	{
		QOpenGLFunctions *ogl = QOpenGLContext::currentContext()->functions();
		shader_->bind();
		vao_->bind();
			// position vbo
		vbo_pos->bind();
		ogl->glEnableVertexAttribArray(ShaderRoundPointGen::ATTRIB_POS);
		ogl->glVertexAttribPointer(ShaderRoundPointGen::ATTRIB_POS, vbo_pos->vector_dimension(), GL_FLOAT, GL_FALSE, stride * vbo_pos->vector_dimension() * 4, void_ptr(first * vbo_pos->vector_dimension() * 4));
		vbo_pos->release();
		// color vbo
		vbo_color->bind();
		ogl->glEnableVertexAttribArray(ShaderRoundPointGen::ATTRIB_COLOR);
		ogl->glVertexAttribPointer(ShaderRoundPointGen::ATTRIB_COLOR, vbo_color->vector_dimension(), GL_FLOAT, GL_FALSE, stride * vbo_pos->vector_dimension() * 4, void_ptr(first * vbo_pos->vector_dimension() * 4));
		vbo_color->release();
		vao_->release();
		shader_->release();
	}
};


template <bool CPV>
typename ShaderRoundPointTpl<CPV>::Param* ShaderRoundPointTpl<CPV>::generate_param()
{
	if (instance_==nullptr)
		instance_ = new ShaderRoundPointTpl<CPV>;
	return (new Param(instance_));
}


using ShaderRoundPoint = ShaderRoundPointTpl<false>;
using ShaderRoundPointColor = ShaderRoundPointTpl<true>;


#if !defined(CGOGN_RENDER_SHADERS_ROUND_POINT_CPP_)
extern template class CGOGN_RENDERING_API ShaderRoundPointTpl<false>;
extern template class CGOGN_RENDERING_API ShaderRoundPointTpl<true>;
extern template class CGOGN_RENDERING_API ShaderParamRoundPoint<false>;
extern template class CGOGN_RENDERING_API ShaderParamRoundPoint<true>;
#endif



} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_SHADERS_ROUND_POINT_H_

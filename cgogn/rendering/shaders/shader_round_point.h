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

#include <QColor>
#include <cgogn/rendering/shaders/shader_program.h>
#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/dll.h>

namespace cgogn
{

namespace rendering
{

class ShaderRoundPoint;

class CGOGN_RENDERING_API ShaderParamRoundPoint : public ShaderParam
{
protected:
	void set_uniforms();

public:
	QColor color_;
	float32 width_;

	ShaderParamRoundPoint(ShaderRoundPoint* sh);

	void set_vbo(VBO* vbo_pos, VBO* vbo_color=nullptr, uint32 stride=0, unsigned first=0);
};


class CGOGN_RENDERING_API ShaderRoundPoint : public ShaderProgram
{
	static const char* vertex_shader_source_;
	static const char* geometry_shader_source_;
	static const char* fragment_shader_source_;

	static const char* vertex_shader_source2_;
	static const char* geometry_shader_source2_;
	static const char* fragment_shader_source2_;

	// uniform ids
	GLint unif_color_;
	GLint unif_width_;

public:

	enum
	{
		ATTRIB_POS = 0,
		ATTRIB_COLOR
	};

	using Param = ShaderParamRoundPoint;

	/**
	 * @brief generate shader parameter object
	 * @return pointer
	 */
	inline Param* generate_param()
	{
		return (new Param(this));
	}

	ShaderRoundPoint(bool color_per_vertex = false);

	/**
	 * @brief set current color
	 * @param rgb
	 */
	void set_color(const QColor& rgb);

	/**
	 * @brief set the width of lines (call before each draw)
	 * @param w width in pixel
	 */
	void set_width(float32 w);

};


} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_SHADERS_ROUND_POINT_H_

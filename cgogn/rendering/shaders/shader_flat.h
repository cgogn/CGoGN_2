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

#ifndef CGOGN_RENDERING_SHADERS_FLAT_H_
#define CGOGN_RENDERING_SHADERS_FLAT_H_

#include <cgogn/rendering/dll.h>
#include <cgogn/rendering/shaders/shader_program.h>
#include <cgogn/rendering/shaders/vbo.h>

#include <QColor>

namespace cgogn
{

namespace rendering
{

class ShaderFlat;

class CGOGN_RENDERING_API ShaderParamFlat : public ShaderParam
{
protected:

	void set_uniforms() override;

public:

	QColor front_color_;
	QColor back_color_;
	QColor ambiant_color_;
	QVector3D light_pos_;

	ShaderParamFlat(ShaderFlat* sh);

	void set_vbo(VBO* vbo_pos, VBO* vbo_color = nullptr);
};

class CGOGN_RENDERING_API ShaderFlat : public ShaderProgram
{
	static const char* vertex_shader_source_;
	static const char* fragment_shader_source_;

	static const char* vertex_shader_source2_;
	static const char* fragment_shader_source2_;

	// uniform ids
	GLint unif_front_color_;
	GLint unif_back_color_;
	GLint unif_ambiant_color_;
	GLint unif_light_position_;

public:

	enum
	{
		ATTRIB_POS = 0,
		ATTRIB_COLOR
	};

	ShaderFlat(bool color_per_vertex = false);

	using Param = ShaderParamFlat;

	/**
	 * @brief generate shader parameter object
	 * @return pointer
	 */
	inline Param* generate_param()
	{
		return (new Param(this));
	}

	/**
	 * @brief set current front color
	 * @param rgb
	 */
	void set_front_color(const QColor& rgb);

	/**
	 * @brief set current front color
	 * @param rgb
	 */
	void set_back_color(const QColor& rgb);

	/**
	 * @brief set current ambiant color
	 * @param rgb
	 */
	void set_ambiant_color(const QColor& rgb);

	/**
	 * @brief set light position relative to screen
	 * @param l light position
	 */
	void set_light_position(const QVector3D& l);

	/**
	 * @brief set light position relative to world
	 * @param l light position
	 * @param view_matrix
	 */
	void set_local_light_position(const QVector3D& l, const QMatrix4x4& view_matrix);
};

} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_SHADERS_FLAT_H_

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

#ifndef CGOGN_RENDERING_SHADERS_PHONG_H_
#define CGOGN_RENDERING_SHADERS_PHONG_H_

#include <QColor>
#include <QVector3D>
#include <cgogn/rendering/shaders/shader_program.h>
#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/dll.h>


namespace cgogn
{

namespace rendering
{

class ShaderPhong;

class CGOGN_RENDERING_API ShaderParamPhong : public ShaderParam
{
protected:
	void set_uniforms();

public:
	QVector3D light_position_;
	QColor front_color_;
	QColor back_color_;
	QColor ambiant_color_;
	QColor specular_color_;
	float32 specular_coef_;
	bool double_side_;

	ShaderParamPhong(ShaderPhong* sh);

	void set_vbo(VBO* vbo_pos, VBO* vbo_norm, VBO* vbo_color=nullptr);
};


class CGOGN_RENDERING_API ShaderPhong : public ShaderProgram
{
	static const char* vertex_shader_source_;
	static const char* fragment_shader_source_;

	static const char* vertex_shader_source_2_;
	static const char* fragment_shader_source_2_;

	// uniform ids
	GLint unif_front_color_;
	GLint unif_back_color_;
	GLint unif_ambiant_color_;
	GLint unif_spec_color_;
	GLint unif_spec_coef_;
	GLint unif_double_side_;
	GLint unif_light_position_;

public:
	enum
	{
		ATTRIB_POS = 0,
		ATTRIB_NORM,
		ATTRIB_COLOR
	};

	using Param = ShaderParamPhong;

	/**
	 * @brief generate shader parameter object
	 * @return pointer
	 */
	inline Param* generate_param()
	{
		return (new Param(this));
	}

	ShaderPhong(bool color_per_vertex = false);

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
	 * @brief set current specular color
	 * @param rgb
	 */
	void set_specular_color(const QColor& rgb);

	/**
	 * @brief set current specular coefficient
	 * @param rgb
	 */
	void set_specular_coef(float32 coef);

	/**
	 * @brief set double side option
	 * @param ts
	 */
	void set_double_side(bool ts);

	/**
	 * @brief set_light_position
	 * @param l
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

#endif // CGOGN_RENDERING_SHADERS_PHONG_H_

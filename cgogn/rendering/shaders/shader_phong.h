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

#ifndef RENDERING_SHADERS_PHONG_H_
#define RENDERING_SHADERS_PHONG_H_

#include <rendering/shaders/shader_program.h>
#include <rendering/shaders/vbo.h>
#include <rendering/dll.h>

class QColor;

namespace cgogn
{

namespace rendering
{

class CGOGN_RENDERING_API ShaderPhong : public ShaderProgram
{
	static const char* vertex_shader_source_;
	static const char* fragment_shader_source_;

	static const char* vertex_shader_source_2_;
	static const char* fragment_shader_source_2_;


	enum
	{
		ATTRIB_POS = 0,
		ATTRIB_NORM,
		ATTRIB_COLOR
	};

	// uniform ids
	int unif_front_color_;
	int unif_back_color_;
	int unif_ambiant_color_;
	int unif_spec_color_;
	int unif_spec_coef_;
	int unif_double_side_;

public:

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
	void set_specular_coef(float coef);

	/**
	 * @brief set double side option
	 * @param ts
	 */
	void set_double_side(bool ts);


	/**
	 * @brief set a vao configuration
	 * @param i id of vao (0,1,....)
	 * @param vbo_pos pointer on position vbo (XYZ)
	 * @param vbo_norm pointer on normal vbo (XYZ)
	 * @param vbo_color pointer on normal vbo (RGB) only used when color per vertex rendering
	 * @return true if ok
	 */
	bool set_vao(unsigned int i, VBO* vbo_pos, VBO* vbo_norm, VBO* vbo_color=NULL);
};

} // namespace rendering

} // namespace cgogn

#endif // RENDERING_SHADERS_PHONG_H_

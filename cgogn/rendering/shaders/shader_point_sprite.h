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

#ifndef RENDERING_SHADER_POINT_SPRITE_H_
#define RENDERING_SHADER_POINT_SPRITE_H_

#include <rendering/shaders/shader_program.h>
#include <rendering/shaders/vbo.h>
#include <rendering/dll.h>

namespace cgogn
{

namespace rendering
{

class CGOGN_RENDERING_API ShaderPointSprite : public ShaderProgram
{
	static const char* vertex_shader_source_;
	static const char* geometry_shader_source_;
	static const char* fragment_shader_source_;

	static const char* vertex_shader_source2_;
	static const char* geometry_shader_source2_;
	static const char* fragment_shader_source2_;

	enum
	{
		ATTRIB_POS = 0,
		ATTRIB_COLOR,
		ATTRIB_SIZE
	};

	// uniform ids
	GLint unif_color_;
	GLint unif_size_;
	GLint unif_ambiant_;
	GLint unif_light_pos_;

public:

	ShaderPointSprite(bool color_per_vertex = false, bool size_per_vertex = false);

	/**
	 * @brief set current color
	 * @param rgb
	 */
	void set_color(const QColor& rgb);

	/**
	 * @brief set ambiant color
	 * @param rgb
	 */
	void set_ambiant(const QColor& rgb);

	/**
	 * @brief set light position relative to screen
	 * @param l
	 */
	void set_light_position(const QVector3D& l);

	/**
	 * @brief set light position relative to world
	 * @param l
	 * @param view_matrix
	 */
	void set_local_light_position(const QVector3D& l, const QMatrix4x4& view_matrix);



	/**
	 * @brief set the size of sphere (call before each draw)
	 * @param w size ofs phere
	 */
	void set_size(float w);

	/**
	 * @brief set a vao configuration
	 * @param i vao id (0,1,...)
	 * @param vbo_pos pointer on position vbo (XYZ)
	 * @param vbo_color pointer on color vbo
	 * @param vbo_size pointer on size (diameters of spheres) vbo
	 * @return true if ok
	 */
	bool set_vao(uint32 i, VBO* vbo_pos,  VBO* vbo_color=NULL, VBO* vbo_size=NULL);
};

} // namespace rendering

} // namespace cgogn

#endif // RENDERING_SHADER_POINT_SPRITE_H_

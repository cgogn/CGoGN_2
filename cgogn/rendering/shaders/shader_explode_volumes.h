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

#ifndef RENDERING_SHADERS_EXPLODE_VOLUMES_H_
#define RENDERING_SHADERS_EXPLODE_VOLUMES_H_

#include <QVector3D>
#include <QVector4D>
#include <rendering/shaders/shader_program.h>
#include <rendering/shaders/vbo.h>
#include <rendering/dll.h>

namespace cgogn
{

namespace rendering
{

class CGOGN_RENDERING_API ShaderExplodeVolumes : public ShaderProgram
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
		ATTRIB_COLOR
	};

	// uniform ids
	int unif_expl_v_;
	int unif_light_position_;
	int unif_plane_clip_;
	int unif_color_;


public:

	ShaderExplodeVolumes(bool color_per_vertex = false);

	void set_explode_volume(float x);

	void set_light_position(const QVector3D& l);

	void set_plane_clip(const QVector4D& plane);

	void set_color(const QColor& rgb);

	/**
	 * @brief set a vao configuration
	 * @param i vao id (0,1,...)
	 * @param vbo_pos pointer on position vbo (XYZ)
	 * @param vbo_color pointer on color vbo
	 * @return true if ok
	 */
	bool set_vao(unsigned int i, VBO* vbo_pos,  VBO* vbo_color = nullptr);
};

} // namespace rendering

} // namespace cgogn

#endif // RENDERING_SHADERS_EXPLODE_VOLUMES_H_

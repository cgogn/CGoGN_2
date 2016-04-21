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

#ifndef CGOGN_RENDERING_SHADERS_EXPLODE_VOLUMES_H_
#define CGOGN_RENDERING_SHADERS_EXPLODE_VOLUMES_H_

#include <QVector3D>
#include <QVector4D>
#include <QColor>
#include <cgogn/rendering/shaders/shader_program.h>
#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/dll.h>

namespace cgogn
{

namespace rendering
{

class ShaderExplodeVolumes;

class CGOGN_RENDERING_API ShaderParamExplodeVolumes : public ShaderParam
{
protected:
	void set_uniforms();

public:
	QColor color_;
	QVector4D plane_clip_;
	QVector3D light_position_;
	float32 explode_factor_;

	ShaderParamExplodeVolumes(ShaderExplodeVolumes* sh);


	void set_vbo(VBO* vbo_pos, VBO* vbo_color=nullptr);
};


class CGOGN_RENDERING_API ShaderExplodeVolumes : public ShaderProgram
{
	static const char* vertex_shader_source_;
	static const char* geometry_shader_source_;
	static const char* fragment_shader_source_;

	static const char* vertex_shader_source2_;
	static const char* geometry_shader_source2_;
	static const char* fragment_shader_source2_;


	// uniform ids
	GLint unif_expl_v_;
	GLint unif_light_position_;
	GLint unif_plane_clip_;
	GLint unif_color_;

public:

	enum
	{
		ATTRIB_POS = 0,
		ATTRIB_COLOR
	};

	using Param = ShaderParamExplodeVolumes;

	/**
	 * @brief generate shader parameter object
	 * @return pointer
	 */
	inline Param* generate_param()
	{
		return (new Param(this));
	}

	ShaderExplodeVolumes(bool color_per_vertex = false);

	void set_explode_volume(float32 x);

	void set_light_position(const QVector3D& l);

	void set_plane_clip(const QVector4D& plane);

	void set_color(const QColor& rgb);

};


} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_SHADERS_EXPLODE_VOLUMES_H_

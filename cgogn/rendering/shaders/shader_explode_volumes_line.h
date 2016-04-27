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

#ifndef CGOGN_RENDERING_SHADERS_EXPLODE_VOLUMES_LINE_H_
#define CGOGN_RENDERING_SHADERS_EXPLODE_VOLUMES_LINE_H_

#include <cgogn/rendering/dll.h>
#include <cgogn/rendering/shaders/shader_program.h>
#include <cgogn/rendering/shaders/vbo.h>

#include <QVector3D>
#include <QVector4D>
#include <QColor>

namespace cgogn
{

namespace rendering
{

class ShaderExplodeVolumesLine;

class CGOGN_RENDERING_API ShaderParamExplodeVolumesLine : public ShaderParam
{
protected:

	void set_uniforms() override;

public:

	QColor color_;
	QVector4D plane_clip_;
	float32 explode_factor_;

	ShaderParamExplodeVolumesLine(ShaderExplodeVolumesLine* sh);

	void set_vbo(VBO* vbo_pos);
};

class CGOGN_RENDERING_API ShaderExplodeVolumesLine : public ShaderProgram
{
	static const char* vertex_shader_source_;
	static const char* geometry_shader_source_;
	static const char* fragment_shader_source_;

	// uniform ids
	GLint unif_expl_v_;
	GLint unif_plane_clip_;
	GLint unif_color_;

public:

	enum
	{
		ATTRIB_POS = 0,
	};

	using Param = ShaderParamExplodeVolumesLine;

	/**
	 * @brief generate shader parameter object
	 * @return pointer
	 */
	inline static Param* generate_param()
	{
		if (instance_==nullptr)
			instance_ = new ShaderExplodeVolumesLine;
		return (new Param(instance_));
	}

	void set_explode_volume(float32 x);

	void set_plane_clip(const QVector4D& plane);

	void set_color(const QColor& rgb);

private:

	ShaderExplodeVolumesLine();
	static ShaderExplodeVolumesLine* instance_;
};

} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_SHADERS_EXPLODE_VOLUMES_LINE_H_

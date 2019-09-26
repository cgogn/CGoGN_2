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

#ifndef CGOGN_RENDERING_SHADERS_EXPLODE_HEXAGRID_H_
#define CGOGN_RENDERING_SHADERS_EXPLODE_HEXAGRID_H_

#include <cgogn/rendering/cgogn_rendering_export.h>
#include <cgogn/rendering/shaders/shader_program.h>
#include <cgogn/rendering/shaders/vbo.h>

#include <QVector3D>
#include <QVector4D>
#include <QColor>
#include <QOpenGLFunctions>

namespace cgogn
{

namespace rendering
{

// forward

class ShaderParamExplodeHexaGrid;

class CGOGN_RENDERING_EXPORT ShaderExplodeHexaGrid : public ShaderProgram
{
	friend class ShaderParamExplodeHexaGrid;

protected:


	static const char* vertex_shader_source2_;
	static const char* geometry_shader_source2_;
	static const char* fragment_shader_source2_;

	// uniform ids
	GLint unif_expl_v_;
	GLint unif_light_position_;
	GLint unif_plane_clip_;
	GLint unif_planes_clip_topo_;
	GLint unif_planes_clip_topo2_;
	GLint unif_color_;

public:

	using Self = ShaderExplodeHexaGrid;
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ShaderExplodeHexaGrid);

	enum
	{
		ATTRIB_POS = 0,
		ATTRIB_COLOR
	};

	void set_explode_volume(float32 x);
	void set_light_position(const QVector3D& l);
	void set_plane_clip(const QVector4D& plane);
	void set_planes_clip_topo(const QVector3D& planes);
	void set_planes_clip_topo2(const QVector3D& planes);
	void set_color(const QColor& col);

	using Param = ShaderParamExplodeHexaGrid;
	static std::unique_ptr<Param> generate_param();

protected:

	ShaderExplodeHexaGrid();
};





class CGOGN_RENDERING_EXPORT ShaderParamExplodeHexaGrid : public ShaderParam
{
protected:

	inline void set_uniforms() override
	{
		ShaderExplodeHexaGrid* sh = static_cast<ShaderExplodeHexaGrid*>(this->shader_);
		sh->set_explode_volume(explode_factor_);
		sh->set_light_position(light_position_);
		sh->set_plane_clip(plane_clip_);
		sh->set_planes_clip_topo(planes_clip_topo_);
		sh->set_planes_clip_topo2(planes_clip_topo2_);
	}

public:

	QVector4D plane_clip_;
	QVector3D planes_clip_topo_;
	QVector3D planes_clip_topo2_;
	QVector3D light_position_;
	float32 explode_factor_;
	QColor color_;

	ShaderParamExplodeHexaGrid(ShaderExplodeHexaGrid* sh) :
		ShaderParam(sh),
		plane_clip_(0, 0, 0, 0),
		planes_clip_topo_(0,0,0),
		planes_clip_topo2_(9999,9999,9999),
		light_position_(10.0f, 100.0f, 1000.0f),
		explode_factor_(0.8f)
	{}

	void set_all_vbos(VBO* vbo_pos, VBO* vbo_color)
	{
		QOpenGLFunctions* ogl = QOpenGLContext::currentContext()->functions();
		shader_->bind();
		vao_->bind();
		// position vbo
		vbo_pos->bind();
		ogl->glEnableVertexAttribArray(ShaderExplodeHexaGrid::ATTRIB_POS);
		ogl->glVertexAttribPointer(ShaderExplodeHexaGrid::ATTRIB_POS, vbo_pos->vector_dimension(), GL_FLOAT, GL_FALSE, 0, 0);
		vbo_pos->release();
		// color vbo
		vbo_color->bind();
		ogl->glEnableVertexAttribArray(ShaderExplodeHexaGrid::ATTRIB_COLOR);
		ogl->glVertexAttribPointer(ShaderExplodeHexaGrid::ATTRIB_COLOR, vbo_color->vector_dimension(), GL_FLOAT, GL_FALSE, 0, 0);
		vbo_color->release();
		vao_->release();
		shader_->release();
	}

	void set_position_vbo(VBO* vbo_pos)
	{
		QOpenGLFunctions* ogl = QOpenGLContext::currentContext()->functions();
		shader_->bind();
		vao_->bind();
		vbo_pos->bind();
		ogl->glEnableVertexAttribArray(ShaderExplodeHexaGrid::ATTRIB_POS);
		ogl->glVertexAttribPointer(ShaderExplodeHexaGrid::ATTRIB_POS, vbo_pos->vector_dimension(), GL_FLOAT, GL_FALSE, 0, 0);
		vbo_pos->release();
		vao_->release();
		shader_->release();
	}

	void set_color_vbo(VBO* vbo_color)
	{
		QOpenGLFunctions* ogl = QOpenGLContext::currentContext()->functions();
		shader_->bind();
		vao_->bind();
		vbo_color->bind();
		ogl->glEnableVertexAttribArray(ShaderExplodeHexaGrid::ATTRIB_COLOR);
		ogl->glVertexAttribPointer(ShaderExplodeHexaGrid::ATTRIB_COLOR, vbo_color->vector_dimension(), GL_FLOAT, GL_FALSE, 0, 0);
		vbo_color->release();
		vao_->release();
		shader_->release();
	}
};


} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_SHADERS_EXPLODE_VOLUMES_H_

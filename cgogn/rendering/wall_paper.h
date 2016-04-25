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

#ifndef CGOGN_RENDERING_WALL_PAPER_H_
#define CGOGN_RENDERING_WALL_PAPER_H_

#include <cgogn/rendering/shaders/shader_texture.h>
#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/dll.h>
#include <QOpenGLFunctions_3_3_Core>

namespace cgogn
{

namespace rendering
{


class CGOGN_RENDERING_API WallPaper
{
protected:

	ShaderTexture::Param* param_texture_;

	VBO* vbo_pos_;
	VBO* vbo_tc_;

public:

	using Self = WallPaper;
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(WallPaper);

	/**
	 * constructor, init all buffers (data and OpenGL) and shader
	 * @Warning need OpenGL context
	 */
	WallPaper(const QImage& img);

	/**
	 * release buffers and shader
	 */
	~WallPaper();

	/**
	 * @brief reinit the vaos (call if you want to use drawer in a new context)
	 */
	void reinit_vao();

	void set_full_screen(bool front = false);

	void set_local_position(uint32 win_w, uint32 win_h, uint32 x, uint32 y, uint32 w, uint32 h, bool front = true);

	void draw(QOpenGLFunctions_3_3_Core* ogl33);
};



} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_WALL_PAPER_H_

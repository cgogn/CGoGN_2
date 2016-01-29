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

#ifndef RENDERING_SHADERVECTORPERVERTEX_H
#define RENDERING_SHADERVECTORPERVERTEX_H

#include <rendering/shaders/shader_program.h>
#include <rendering/shaders/vbo.h>
#include <rendering/dll.h>

namespace cgogn
{
namespace rendering
{

class CGOGN_RENDERING_API ShaderVectorPerVertex : public ShaderProgram
{
	/// vertex shader source
	static const char* vertex_shader_source_;
	/// fragment shader source
	static const char* geometry_shader_source_;
	/// fragment shader source
	static const char* fragment_shader_source_;

	/// here two attributes
	enum {ATTRIB_POS=0, ATTRIB_NORMAL};

	/// uniform id of color
	int unif_color_;

	/// uniform id of length
	int unif_length_;


public:
	ShaderVectorPerVertex();

	/**
	 * @brief set current color
	 * @param rgb
	 */
	void set_color(const QColor& rgb);

	/**
	 * @brief set length of normal
	 * @param l length
	 */
	void set_length(float l);

	/**
	 * @brief set a vao configuration
	 * @param i vao id (0,1,...)
	 * @param vbo_pos pointer on position vbo (XYZ)
	 * @param vbo_norm pointer on normal vbo
	 * @return true if ok
	 */
	bool set_vao(unsigned int i, VBO* vbo_pos,  VBO* vbo_norm);

};

} // namespace rendering
} // namespace cgogn


#endif // RENDERING_SHADERVECTORPERVERTEX_H

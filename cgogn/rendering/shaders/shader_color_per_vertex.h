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

#ifndef CGOGN_RENDERING_SHADERS_COLORPERVERTEX_H_
#define CGOGN_RENDERING_SHADERS_COLORPERVERTEX_H_

#include <cgogn/rendering/shaders/shader_program.h>
#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/dll.h>

namespace cgogn
{

namespace rendering
{

class ShaderColorPerVertex;

class CGOGN_RENDERING_API ShaderParamColorPerVertex : public ShaderParam
{
protected:
	inline void set_uniforms() {}

public:

	ShaderParamColorPerVertex(ShaderColorPerVertex* prg);


	/**
	 * @brief set a vbo configuration
	 * @param vbo_pos pointer on position vbo (XYZ)
	 * @param vbo_col pointer on color vbo (RGB)
	 */
	void set_vbo(VBO* vbo_pos, VBO* vbo_col);

};


class CGOGN_RENDERING_API ShaderColorPerVertex : public ShaderProgram
{
	static const char* vertex_shader_source_;
	static const char* fragment_shader_source_;

public:

	enum
	{
		ATTRIB_POS = 0,
		ATTRIB_COLOR
	};

    ShaderColorPerVertex();

	using Param = ShaderParamColorPerVertex;

	/**
	 * @brief generate shader parameter object
	 * @return pointer
	 */
	inline Param* generate_param()
	{
		return (new Param(this));
	}
};




} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_SHADERS_COLORPERVERTEX_H_

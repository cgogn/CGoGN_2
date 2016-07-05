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

#ifndef CGOGN_RENDERING_TRANSPARENCY_SORTING_H_
#define CGOGN_RENDERING_TRANSPARENCY_SORTING_H_

#include <cgogn/rendering/dll.h>
#include <cgogn/rendering/shaders/shader_program.h>
#include <cgogn/rendering/shaders/vbo.h>

#include <QOpenGLFunctions>
#include <QColor>

namespace cgogn
{

namespace rendering
{

class ShaderParamTransfoOnly;
class ShaderTransfoOnly;


class CGOGN_RENDERING_API TransparencySorting
{
	std::vector< std::array<GLfloat,9> > feedback_buffer_;
	uint32 nb_indices_;
	std::unique_ptr<cgogn::rendering::VBO> vbo2_;
	std::unique_ptr<cgogn::rendering::ShaderParamTransfoOnly> param_transfo_;
	std::unique_ptr<cgogn::rendering::ShaderTransfoOnly> sh_transfo_;

public:
	TransparencySorting();

	~TransparencySorting();

	/**
	 * @brief init the sort
	 * @param input_vbo input position vbo
	 * @param nb_indices number of indices (tri*3)
	 * @param ogl33
	 */
	void init(cgogn::rendering::VBO* input_vbo, uint32 nb_indices, QOpenGLFunctions_3_3_Core* ogl33);

	/**
	 * @brief output sorted vbo (you can call once in your init)
	 * @return
	 */
	cgogn::rendering::VBO* output_vbo();

	/**
	 * @brief pre sorting: call before "render_->draw(cgogn::rendering::TRIANGLES);"
	 * @param proj
	 * @param view
	 * @param ogl33
	 */
	void pre_sorting(const QMatrix4x4& proj, const QMatrix4x4& view, QOpenGLFunctions_3_3_Core* ogl33);

	/**
	 * @brief post_sorting: call after "render_->draw(cgogn::rendering::TRIANGLES);"
	 * @param ogl33
	 */
	void post_sorting(QOpenGLFunctions_3_3_Core* ogl33);
};


class CGOGN_RENDERING_API ShaderTransfoOnly : public ShaderProgram
{
	friend class ShaderParamTransfoOnly;
protected:
	static const char* vertex_shader_source_;
public:
	enum { ATTRIB_POS = 0 };
	ShaderTransfoOnly(QOpenGLFunctions_3_3_Core* ogl33);
};



class CGOGN_RENDERING_API ShaderParamTransfoOnly : public ShaderParam
{
protected:
	inline void set_uniforms() override {}
public:
	ShaderParamTransfoOnly(ShaderTransfoOnly* sh);
	void set_position_vbo(VBO* vbo_pos);
};




} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_TRANSPARENCY_SORTING_H_

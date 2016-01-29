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
#define CGOGN_RENDERING_DLL_EXPORT
#include <rendering/shaders/shader_program.h>

namespace cgogn
{
namespace rendering
{

ShaderProgram::~ShaderProgram()
{
	for (QOpenGLVertexArrayObject* vao: vaos_)
	{
		vao->destroy();
		delete vao;
	}
}

void ShaderProgram::get_matrices_uniforms()
{
	unif_mv_matrix_ = prg_.uniformLocation("model_view_matrix");
	unif_projection_matrix_ = prg_.uniformLocation("projection_matrix");
	unif_normal_matrix_ = prg_.uniformLocation("normal_matrix");
}

void ShaderProgram::set_matrices(const QMatrix4x4& proj, const QMatrix4x4& mv)
{
	prg_.setUniformValue(unif_projection_matrix_,proj);
	prg_.setUniformValue(unif_mv_matrix_, mv);

	if (unif_normal_matrix_>=0)
	{
		QMatrix3x3 normalMatrix = mv.normalMatrix();
		prg_.setUniformValue(unif_normal_matrix_, normalMatrix);
	}
}



void ShaderProgram::set_view_matrice(const QMatrix4x4& mv)
{
	prg_.setUniformValue(unif_mv_matrix_, mv);

	if (unif_normal_matrix_>=0)
	{
		QMatrix3x3 normalMatrix = mv.normalMatrix();
		prg_.setUniformValue(unif_normal_matrix_, normalMatrix);
	}
}

} // namespace rendering
} // namespace cgogn

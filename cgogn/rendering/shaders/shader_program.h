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

#ifndef RENDERING_SHADERS_SHADERPROGRAM_H_
#define RENDERING_SHADERS_SHADERPROGRAM_H_

#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLFunctions_3_3_Core>
#include <cassert>

#include <rendering/dll.h>

namespace cgogn
{
namespace rendering
{

class CGOGN_RENDERING_API ShaderProgram : protected QOpenGLFunctions_3_3_Core
{
protected:

	QOpenGLShaderProgram prg_;

	std::vector<QOpenGLVertexArrayObject*> vaos_;

	int unif_mv_matrix_;
	int unif_projection_matrix_;
	int unif_normal_matrix_;

public:

	~ShaderProgram();

	/**
	 * @brief get the matrices uniforms (call after link)
	 */
	void get_matrices_uniforms();

	/**
	 * @brief set matrices (normal matrix computed if needed)
	 * @param proj projection matrix
	 * @param mv model view matrix
	 */
	void set_matrices(const QMatrix4x4& proj, const QMatrix4x4& mv);

	/**
	 * @brief set model-view matrice (normal matrix computed if needed)
	 * @param mv model view matrix
	 */
	void set_view_matrix(const QMatrix4x4& mv);

	/**
	 * @brief add a vao (vbo configuration)
	 * @return the id of vao
	 */
	inline unsigned int add_vao()
	{
		vaos_.emplace_back(new QOpenGLVertexArrayObject);
		vaos_.back()->create();
		return vaos_.size() - 1;
	}

	/**
	 * @brief allocate new vaos until total nb is reached
	 * @param nb number of vaos to reach
	 */
	void alloc_vao(unsigned int nb)
	{
		while (vaos_.size() < nb)
			vaos_.emplace_back(new QOpenGLVertexArrayObject);
	}

	/**
	 * @brief number of allocated vaos
	 * @return the number of allocated vaos
	 */
	inline unsigned int nb_vaos()
	{
		return (unsigned int)(vaos_.size());
	}

	/**
	 * @brief bind a vao
	 * @param i vao id (0,1,...)
	 */
	inline void bind_vao(unsigned int i)
	{
//		assert(i < vaos_.size());
//		if (!vaos_[i]->isCreated())
//			vaos_[i]->create();
		vaos_[i]->bind();
	}

	/**
	 * @brief release the vao
	 * @param i id
	 */
	inline void release_vao(unsigned int i)
	{
//		assert(i < vaos_.size());
		vaos_[i]->release();
	}

	/**
	 * @brief bind the shader
	 */
	inline void bind()
	{
		prg_.bind();
	}

	/**
	 * @brief release the shader
	 */
	inline void release()
	{
		prg_.release();
	}
};

} // namespace rendering

} // namespace cgogn

#endif // RENDERING_SHADERS_SHADERPROGRAM_H_

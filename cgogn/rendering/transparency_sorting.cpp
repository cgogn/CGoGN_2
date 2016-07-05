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

#include <cgogn/rendering/transparency_sorting.h>

namespace cgogn
{

namespace rendering
{

const char* ShaderTransfoOnly::vertex_shader_source_ =
"#version 150\n"
"in vec3 vertex_pos;\n"
"out vec3 position;\n"
"uniform mat4 model_view_matrix;\n"
"void main()\n"
"{\n"
"   position = (model_view_matrix * vec4(vertex_pos,1.0)).xyz;\n"
"}\n";


ShaderTransfoOnly::ShaderTransfoOnly(QOpenGLFunctions_3_3_Core* ogl33)
{
	const GLchar* feedbackVaryings[] = { "position" };
	QOpenGLShader* sh = new QOpenGLShader(QOpenGLShader::Vertex);
	sh->compileSourceCode(vertex_shader_source_);
	prg_.addShader(sh);

	ogl33->glTransformFeedbackVaryings(prg_.programId(), 1, feedbackVaryings, GL_INTERLEAVED_ATTRIBS);
	prg_.bindAttributeLocation("vertex_pos", ATTRIB_POS);
	prg_.link();
	get_matrices_uniforms();
}



ShaderParamTransfoOnly::ShaderParamTransfoOnly(ShaderTransfoOnly* sh) :
	ShaderParam(sh)
{}

void ShaderParamTransfoOnly::set_position_vbo(VBO* vbo_pos)
{
	QOpenGLFunctions* ogl = QOpenGLContext::currentContext()->functions();
	shader_->bind();
	vao_->bind();
	vbo_pos->bind();
	ogl->glEnableVertexAttribArray(ShaderTransfoOnly::ATTRIB_POS);
	ogl->glVertexAttribPointer(ShaderTransfoOnly::ATTRIB_POS, vbo_pos->vector_dimension(), GL_FLOAT, GL_FALSE, 0, 0);
	vbo_pos->release();
	vao_->release();
	shader_->release();

}




TransparencySorting::TransparencySorting():
	sh_transfo_(nullptr),
	param_transfo_(nullptr)
{
	vbo2_ = cgogn::make_unique<cgogn::rendering::VBO>(3);
}


TransparencySorting::~TransparencySorting()
{
	sh_transfo_.reset();
	param_transfo_.reset();
	vbo2_.reset();
}

void TransparencySorting::init(cgogn::rendering::VBO* input_vbo, uint32 nb_indices, QOpenGLFunctions_3_3_Core* ogl33)
{
	if (!sh_transfo_)
	{
		sh_transfo_ = cgogn::make_unique<cgogn::rendering::ShaderTransfoOnly>(ogl33);
		param_transfo_ = cgogn::make_unique<cgogn::rendering::ShaderParamTransfoOnly>(sh_transfo_.get());
	}
	param_transfo_->set_position_vbo(input_vbo);
	vbo2_->allocate(nb_indices,3);
	nb_indices_ = nb_indices;
}

cgogn::rendering::VBO* TransparencySorting::output_vbo()
{
	return vbo2_.get();
}


void TransparencySorting::pre_sorting(const QMatrix4x4& proj, const QMatrix4x4& view, QOpenGLFunctions_3_3_Core* ogl33)
{
	param_transfo_->bind(proj,view);
	ogl33->glEnable(GL_RASTERIZER_DISCARD);
	ogl33->glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, vbo2_->id());
	ogl33->glBeginTransformFeedback(GL_TRIANGLES);
}

void TransparencySorting::post_sorting(QOpenGLFunctions_3_3_Core* ogl33)
{
	ogl33->glEndTransformFeedback();
	ogl33->glDisable(GL_RASTERIZER_DISCARD);
	ogl33->glFlush();
	param_transfo_->release();

	feedback_buffer_.resize(nb_indices_/3);
	vbo2_->bind();
	ogl33->glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, feedback_buffer_.size()*9*sizeof(GLfloat), feedback_buffer_.data());

	std::sort(feedback_buffer_.begin(),feedback_buffer_.end(),
			  [] (const std::array<GLfloat,9>& ta, const std::array<GLfloat,9>& tb )
	{
		GLfloat av_a = ta[2]+ta[5]+ta[8];
		GLfloat av_b = tb[2]+tb[5]+tb[8];
		return av_a < av_b;
	});

	ogl33->glBufferSubData(GL_ARRAY_BUFFER, 0, feedback_buffer_.size()*9*sizeof(GLfloat), feedback_buffer_.data());
	vbo2_->release();
}


} // namespace rendering

} // namespace cgogn

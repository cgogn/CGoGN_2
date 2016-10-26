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

#ifndef CGOGN_RENDERING_FLAT_TR_DR_H_
#define CGOGN_RENDERING_FLAT_TR_DR_H_

#include <cgogn/rendering/dll.h>
#include <cgogn/rendering/shaders/shader_program.h>
#include <cgogn/rendering/shaders/vbo.h>

#include <QOpenGLFunctions>
#include <QColor>
#include <QOpenGLFramebufferObject>

namespace cgogn
{

namespace rendering
{

// forward
class ShaderFlatTransp;
class ShaderTranspQuad;




class CGOGN_RENDERING_API ShaderParamFlatTransp : public ShaderParam
{
protected:

	void set_uniforms() override;

public:

	QColor front_color_;
	QColor back_color_;
	QColor ambiant_color_;
	QVector3D light_pos_;
	GLint layer_;
	GLuint rgba_texture_sampler_;
	GLuint depth_texture_sampler_;
	bool bf_culling_;

	ShaderParamFlatTransp(ShaderFlatTransp* sh);

	void set_position_vbo(VBO* vbo_pos);
};




class CGOGN_RENDERING_API ShaderFlatTransp : public ShaderProgram
{
	friend class ShaderParamFlatTransp;

protected:

	static const char* vertex_shader_source_;
	static const char* fragment_shader_source_;

	// uniform ids
	GLint unif_front_color_;
	GLint unif_back_color_;
	GLint unif_ambiant_color_;
	GLint unif_light_position_;
	GLint unif_bf_culling_;

	GLint unif_layer_;
	GLint unif_depth_texture_sampler_;
	GLint unif_rgba_texture_sampler_;

public:

	using Self = ShaderFlatTransp;
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ShaderFlatTransp);

	enum
	{
		ATTRIB_POS = 0,
	};

	/**
	 * @brief set current front color
	 * @param rgb
	 */
	void set_front_color(const QColor& rgb);

	/**
	 * @brief set current front color
	 * @param rgb
	 */
	void set_back_color(const QColor& rgb);

	/**
	 * @brief set current ambiant color
	 * @param rgb
	 */
	void set_ambiant_color(const QColor& rgb);

	/**
	 * @brief set light position relative to screen
	 * @param l light position
	 */
	void set_light_position(const QVector3D& l);

	void set_bf_culling(bool cull);

	void set_layer(int layer);

	void set_rgba_sampler(GLuint rgba_samp);

	void set_depth_sampler(GLuint depth_samp);

	using Param = ShaderParamFlatTransp;

	static std::unique_ptr<Param> generate_param();

private:

	ShaderFlatTransp();
	static std::unique_ptr<ShaderFlatTransp> instance_;

};




class CGOGN_RENDERING_API ShaderParamTranspQuad : public ShaderParam
{
protected:
	void set_uniforms() override;
public:
	GLuint rgba_texture_sampler_;
	GLuint depth_texture_sampler_;
	ShaderParamTranspQuad(ShaderTranspQuad* sh);
};


class CGOGN_RENDERING_API ShaderTranspQuad : public ShaderProgram
{
	friend class ShaderParamTranspQuad;

protected:

	static const char* vertex_shader_source_;
	static const char* fragment_shader_source_;

	// uniform ids
	GLint unif_depth_texture_sampler_;
	GLint unif_rgba_texture_sampler_;

public:

	using Self = ShaderTranspQuad;
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ShaderTranspQuad);

	void set_rgba_sampler(GLuint rgba_samp);

	void set_depth_sampler(GLuint depth_samp);

	using Param = ShaderParamTranspQuad;

	static std::unique_ptr<Param> generate_param();

private:

	ShaderTranspQuad();
	static std::unique_ptr<ShaderTranspQuad> instance_;

};


class CGOGN_RENDERING_API FlatTransparencyDrawer
{
	int max_nb_layers_;

	/// rendering shader
	std::unique_ptr<cgogn::rendering::ShaderFlatTransp::Param> param_flat_;

	/// shader for quad blending  with opaque scene
	std::unique_ptr<cgogn::rendering::ShaderTranspQuad::Param> param_trq_;

	/// FBO
	std::unique_ptr<QOpenGLFramebufferObject> fbo_layer_;

	/// Occlusion query
	GLuint oq_transp;

	QOpenGLFunctions_3_3_Core* ogl33_;

	int width_;

	int height_;

public:

	~FlatTransparencyDrawer();

	/**
	 * @brief create and init
	 * @param w
	 * @param h
	 * @param ogl33
	 */
	FlatTransparencyDrawer(int w, int h, QOpenGLFunctions_3_3_Core* ogl33);


	/**
	 * @brief resize call_back need to be called when resize windows
	 * @param w
	 * @param h
	 */
	void resize(int w, int h);

	template<typename FUNC>
	void draw(const QMatrix4x4& proj, const QMatrix4x4& view, const FUNC& draw_func);

	inline void set_max_nb_layers(int nbl)
	{
		max_nb_layers_ = nbl;
	}

	inline void set_light_position(const QVector3D& l)
	{
		param_flat_->light_pos_ = l;
	}

	inline void set_front_color(const QColor& rgba)
	{
		param_flat_->front_color_ = rgba;
	}

	inline void set_back_color(const QColor& rgba)
	{
		param_flat_->back_color_ = rgba;
	}

	inline void set_ambiant_color(const QColor& rgba)
	{
		param_flat_->ambiant_color_ = rgba;
	}

	inline void set_position_vbo(cgogn::rendering::VBO* vbo_pos)
	{
		param_flat_->set_position_vbo(vbo_pos);
	}
};



template<typename FUNC>
void FlatTransparencyDrawer::draw(const QMatrix4x4& proj, const QMatrix4x4& view, const FUNC& draw_func)
{
	ogl33_->glEnable(GL_TEXTURE_2D);
	QVector<GLuint> textures = fbo_layer_->textures();
	GLenum buffs[2] = {GL_COLOR_ATTACHMENT0,GL_COLOR_ATTACHMENT2};

	param_flat_->rgba_texture_sampler_ = 0;
	param_flat_->depth_texture_sampler_ = 1;
	fbo_layer_->bind();

	GLenum clear_buff[1] = {GL_COLOR_ATTACHMENT3};
	ogl33_->glDrawBuffers(1,clear_buff);
	ogl33_->glClearColor(0.0f,0.0f,0.0f,1.0f);
	ogl33_->glClear(GL_COLOR_BUFFER_BIT);
	ogl33_->glClearColor(0.0f,0.0f,0.0f,0.0f);

	ogl33_->glDrawBuffers(1,buffs);
	ogl33_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	for (int p=0;p<max_nb_layers_;++p)
	{
		ogl33_->glDrawBuffers(1,buffs);
		ogl33_->glClear(GL_DEPTH_BUFFER_BIT);

		ogl33_->glDrawBuffers(2,buffs);
		param_flat_->layer_ = p;
		ogl33_->glActiveTexture(GL_TEXTURE0);
		ogl33_->glBindTexture(GL_TEXTURE_2D,textures[3]);
		ogl33_->glActiveTexture(GL_TEXTURE1);
		ogl33_->glBindTexture(GL_TEXTURE_2D,textures[1]);

		ogl33_->glBeginQuery(GL_SAMPLES_PASSED, oq_transp);
		param_flat_->bind(proj,view);
		draw_func();
		param_flat_->release();
		ogl33_->glEndQuery(GL_SAMPLES_PASSED);

		GLuint nb_samples;
		ogl33_->glGetQueryObjectuiv(oq_transp, GL_QUERY_RESULT, &nb_samples);

		if (nb_samples==0) // finished ?
			p = max_nb_layers_;
		else
		{
			ogl33_->glReadBuffer(GL_COLOR_ATTACHMENT2);
			ogl33_->glBindTexture(GL_TEXTURE_2D,textures[1]);
			ogl33_->glCopyTexImage2D(GL_TEXTURE_2D,0,GL_R32F,0,0,width_,height_,0);

			if (p==0)
			{
				ogl33_->glBindTexture(GL_TEXTURE_2D,textures[4]);
				ogl33_->glCopyTexImage2D(GL_TEXTURE_2D,0,GL_R32F,0,0,width_,height_,0);
			}

			ogl33_->glReadBuffer(GL_COLOR_ATTACHMENT0);
			ogl33_->glBindTexture(GL_TEXTURE_2D,textures[3]);
			ogl33_->glCopyTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,0,0,width_,height_,0);
		}
	}

	fbo_layer_->release();

	// real draw with blending with opaque object

	param_trq_->rgba_texture_sampler_ = 0;
	param_trq_->depth_texture_sampler_ = 1;

	ogl33_->glActiveTexture(GL_TEXTURE0);
	ogl33_->glBindTexture(GL_TEXTURE_2D,textures[3]);

	ogl33_->glActiveTexture(GL_TEXTURE1);
	ogl33_->glBindTexture(GL_TEXTURE_2D,textures[4]);

	ogl33_->glEnable(GL_BLEND);
	ogl33_->glBlendFunc(GL_ONE_MINUS_SRC_ALPHA, GL_SRC_ALPHA);
	param_trq_->bind(proj,view);
	ogl33_->glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	param_trq_->release();
	ogl33_->glDisable(GL_BLEND);
}



} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_FLAT_TR_DR_H_

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

#ifndef CGOGN_RENDERING_TRANSP_VOLUME_DRAWER_H_
#define CGOGN_RENDERING_TRANSP_VOLUME_DRAWER_H_

#include <cgogn/rendering/dll.h>

#include <cgogn/rendering/shaders/vbo.h>
#include <cgogn/rendering/transparency_shaders/shader_transparent_volumes.h>
#include <cgogn/rendering/transparency_shaders/shader_transparent_quad.h>
#include <cgogn/rendering/transparency_shaders/shader_copy_depth.h>

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/algos/centroid.h>
#include <cgogn/geometry/algos/ear_triangulation.h>

#include <QOpenGLFunctions_3_3_Core>
#include <QColor>
#include <QOpenGLFramebufferObject>
#include <QOpenGLTexture>

namespace cgogn
{

namespace rendering
{


class CGOGN_RENDERING_API VolumeTransparencyDrawer
{
protected:

	using Vec3f = std::array<float32, 3>;
	std::unique_ptr<VBO> vbo_pos_;
	QColor face_color_;
	float32 shrink_v_;

public:
	class CGOGN_RENDERING_API Renderer
	{
		friend class VolumeTransparencyDrawer;

		std::unique_ptr<ShaderTransparentVolumes::Param> param_transp_vol_;
		VolumeTransparencyDrawer* volume_drawer_data_;

		/// shader for quad blending  with opaque scene
		std::unique_ptr<cgogn::rendering::ShaderTranspQuad::Param> param_trq_;


		std::unique_ptr<ShaderCopyDepth::Param> param_copy_depth_;

		int max_nb_layers_;

		/// FBO
		std::unique_ptr<QOpenGLFramebufferObject> fbo_layer_;

		/// Occlusion query
		GLuint oq_transp;

		QOpenGLFunctions_3_3_Core* ogl33_;

		int width_;

		int height_;

		Renderer(VolumeTransparencyDrawer* tr, int w, int h, QOpenGLFunctions_3_3_Core* ogl33);


	public:
		void resize(int w, int h);

		~Renderer();

		/**
		 * @brief draw the transparent volumes mixed with opaque (also drawn separatly)
		 * @param projection projection matrix
		 * @param modelview modelview matrix
		 * @param of the func/lambda that draw opaque objects (use to get zbuffer, not drawn on screen)
		 */
		template<typename OPAQUE_FUNC>
		void draw_faces(const QMatrix4x4& projection, const QMatrix4x4& modelview, const OPAQUE_FUNC& of);

		/**
		 * @brief draw only the transparent volumes (bad mixing with opaque objects)
		 * @param projection projection matrix
		 * @param modelview modelview matrix
		 * @param of the func/lambda that draw opaque objects (use to get zbuffer, not drawn on screen)
		 */
		inline void draw_faces(const QMatrix4x4& projection, const QMatrix4x4& modelview)
		{
			draw_faces(projection,modelview,[]{});
		}

		void set_explode_volume(float32 x);

		void set_color(const QColor& rgb);

		void set_clipping_plane(const QVector4D& pl);

		void set_clipping_plane2(const QVector4D& pl);

		void set_thick_clipping_plane(const QVector4D& p, float32 th);

		void set_back_face_culling(bool cull);

		void set_lighted(bool lighted);

		/**
		 * @brief set the max number of layers (eq drawing passes)
		 * @param nbl
		 */
		void set_max_nb_layers(int nbl);

	};

	using Self = VolumeTransparencyDrawer;

	/**
	 * constructor, init all buffers (data and OpenGL) and shader
	 * @Warning need OpenGL context
	 */
	VolumeTransparencyDrawer();


	CGOGN_NOT_COPYABLE_NOR_MOVABLE(VolumeTransparencyDrawer);

	/**
	 * @brief generate a renderer (one per context)
	 * @return pointer on renderer
	 */
	inline std::unique_ptr<Renderer> generate_renderer(int w, int h, QOpenGLFunctions_3_3_Core* ogl33)
	{
		return std::unique_ptr<Renderer>(new Renderer(this,w,h,ogl33));
	}

	template <typename VEC3, typename MAP>
	void update_face(const MAP& m, const typename MAP::template VertexAttribute<VEC3>& position);
};





template <typename VEC3, typename MAP>
void VolumeTransparencyDrawer::update_face(const MAP& m, const typename MAP::template VertexAttribute<VEC3>& position)
{
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;
	using Volume = typename MAP::Volume;

	std::vector<Vec3f> out_pos;
	out_pos.reserve(1024 * 1024);

	std::vector<uint32> ear_indices;
	ear_indices.reserve(256);

	m.foreach_cell([&] (Volume v)
	{
		VEC3 CV = geometry::centroid<VEC3>(m, v, position);
		m.foreach_incident_face(v, [&] (Face f)
		{
			if (m.has_codegree(f, 3))
			{
				const VEC3& P1 = position[Vertex(f.dart)];
				const VEC3& P2 = position[Vertex(m.phi1(f.dart))];
				const VEC3& P3 = position[Vertex(m.phi1(m.phi1(f.dart)))];
				out_pos.push_back({float32(CV[0]), float32(CV[1]), float32(CV[2])});
				out_pos.push_back({float32(P1[0]), float32(P1[1]), float32(P1[2])});
				out_pos.push_back({float32(P2[0]), float32(P2[1]), float32(P2[2])});
				out_pos.push_back({float32(P3[0]), float32(P3[1]), float32(P3[2])});
			}
			else
			{
				ear_indices.clear();
				cgogn::geometry::append_ear_triangulation<VEC3>(m, f, position, ear_indices);
				for(std::size_t i = 0; i < ear_indices.size(); i += 3)
				{
					const VEC3& P1 = position[ear_indices[i]];
					const VEC3& P2 = position[ear_indices[i+1]];
					const VEC3& P3 = position[ear_indices[i+2]];
					out_pos.push_back({float32(CV[0]), float32(CV[1]), float32(CV[2])});
					out_pos.push_back({float32(P1[0]), float32(P1[1]), float32(P1[2])});
					out_pos.push_back({float32(P2[0]), float32(P2[1]), float32(P2[2])});
					out_pos.push_back({float32(P3[0]), float32(P3[1]), float32(P3[2])});
				}
			}
		});
	});

	uint32 nbvec = uint32(out_pos.size());

	vbo_pos_->allocate(nbvec, 3);
	vbo_pos_->bind();
	vbo_pos_->copy_data(0, nbvec * 12, out_pos[0].data());
	vbo_pos_->release();
}


template<typename OPAQUE_FUNC>
void VolumeTransparencyDrawer::Renderer::draw_faces(const QMatrix4x4& projection, const QMatrix4x4& modelview, const OPAQUE_FUNC& opaque_func)
{
	ogl33_->glEnable(GL_TEXTURE_2D);

	QOpenGLTexture depth_tex(QOpenGLTexture::Target2D);
	depth_tex.setFormat(QOpenGLTexture::D24);
	depth_tex.setSize(width_,height_);
	depth_tex.bind();
	ogl33_->glReadBuffer(GL_FRONT);
	ogl33_->glCopyTexImage2D(GL_TEXTURE_2D, 0,GL_DEPTH_COMPONENT24, 0, 0, width_, height_,0);
	depth_tex.release();
	param_copy_depth_->texture_ = &depth_tex;

	QVector<GLuint> textures = fbo_layer_->textures();
	GLenum buffs[2] = { GL_COLOR_ATTACHMENT0,GL_COLOR_ATTACHMENT2 };

	param_transp_vol_->rgba_texture_sampler_ = 0;
	param_transp_vol_->depth_texture_sampler_ = 1;
	fbo_layer_->bind();

	GLenum clear_buff[1] = { GL_COLOR_ATTACHMENT3 };
	ogl33_->glDrawBuffers(1, clear_buff);
	ogl33_->glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	ogl33_->glClear(GL_COLOR_BUFFER_BIT);

	ogl33_->glDrawBuffers(1, buffs);
	ogl33_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	GLenum opaq_buff[1] = { GL_COLOR_ATTACHMENT5 };

	for (int p = 0; p<max_nb_layers_; ++p)
	{
		ogl33_->glClear(GL_DEPTH_BUFFER_BIT);

		if (p > 0)
		{
			ogl33_->glDrawBuffers(1, opaq_buff);
			param_copy_depth_->bind(projection, modelview);
			ogl33_->glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
			param_copy_depth_->release();
		}

		ogl33_->glDrawBuffers(2, buffs);
		param_transp_vol_->layer_ = p;
		ogl33_->glActiveTexture(GL_TEXTURE0);
		ogl33_->glBindTexture(GL_TEXTURE_2D, textures[3]);
		ogl33_->glActiveTexture(GL_TEXTURE1);
		ogl33_->glBindTexture(GL_TEXTURE_2D, textures[1]);

		ogl33_->glBeginQuery(GL_SAMPLES_PASSED, oq_transp);

		param_transp_vol_->bind(projection, modelview);
		ogl33_->glDrawArrays(GL_LINES_ADJACENCY, 0, volume_drawer_data_->vbo_pos_->size());
		param_transp_vol_->release();

		ogl33_->glEndQuery(GL_SAMPLES_PASSED);

		GLuint nb_samples;
		ogl33_->glGetQueryObjectuiv(oq_transp, GL_QUERY_RESULT, &nb_samples);

		if (nb_samples == 0) // finished ?
		{
			p = max_nb_layers_;
		}
		else
		{
			ogl33_->glReadBuffer(GL_COLOR_ATTACHMENT2);
			ogl33_->glBindTexture(GL_TEXTURE_2D, textures[1]);
			ogl33_->glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, 0, 0, width_, height_, 0);

			if (p == 0)
			{
				ogl33_->glBindTexture(GL_TEXTURE_2D, textures[4]);
				ogl33_->glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, 0, 0, width_, height_, 0);
			}

			ogl33_->glReadBuffer(GL_COLOR_ATTACHMENT0);
			ogl33_->glBindTexture(GL_TEXTURE_2D, textures[3]);
			ogl33_->glCopyTexImage2D(GL_TEXTURE_2D, 0,/*GL_RGBA8*/GL_RGBA32F, 0, 0, width_, height_, 0);
		}
	}

	fbo_layer_->release();

	// real draw with blending with opaque object

	param_trq_->rgba_texture_sampler_ = 0;
	param_trq_->depth_texture_sampler_ = 1;

	ogl33_->glActiveTexture(GL_TEXTURE0);
	ogl33_->glBindTexture(GL_TEXTURE_2D, textures[3]);

	ogl33_->glActiveTexture(GL_TEXTURE1);
	ogl33_->glBindTexture(GL_TEXTURE_2D, textures[4]);

	ogl33_->glEnable(GL_BLEND);
	ogl33_->glBlendFunc(GL_ONE, GL_SRC_ALPHA);
	param_trq_->bind(projection, modelview);
	ogl33_->glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	param_trq_->release();
	ogl33_->glDisable(GL_BLEND);

}

} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_TRANSP_VOLUME_DRAWER_H_

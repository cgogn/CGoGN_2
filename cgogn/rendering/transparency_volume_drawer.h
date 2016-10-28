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
#include <cgogn/rendering/shaders/shader_program.h>

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/algos/centroid.h>
#include <cgogn/geometry/algos/ear_triangulation.h>

#include <QOpenGLFunctions_3_3_Core>
#include <QColor>
#include <QOpenGLFramebufferObject>

namespace cgogn
{

namespace rendering
{




class ShaderTransparentVolumes;

class ShaderParamTransparentVolumes : public ShaderParam
{
protected:

	void set_uniforms() override;

public:

	QColor color_;
	QVector4D plane_clip_;
	QVector4D plane_clip2_;
	QVector3D light_position_;
	float32 explode_factor_;

	GLint layer_;
	GLuint rgba_texture_sampler_;
	GLuint depth_texture_sampler_;
	bool bf_culling_;
	bool lighted_;

	ShaderParamTransparentVolumes(ShaderTransparentVolumes* sh);

	void set_position_vbo(VBO* vbo_pos);

};


class CGOGN_RENDERING_API ShaderTransparentVolumes : public ShaderProgram
{
	friend class ShaderParamTransparentVolumes;

protected:

	static const char* vertex_shader_source_;
	static const char* geometry_shader_source_;
	static const char* fragment_shader_source_;

	// uniform ids
	GLint unif_expl_v_;
	GLint unif_light_position_;
	GLint unif_plane_clip_;
	GLint unif_plane_clip2_;
	GLint unif_color_;
	GLint unif_bf_culling_;
	GLint unif_lighted_;
	GLint unif_layer_;
	GLint unif_depth_texture_sampler_;
	GLint unif_rgba_texture_sampler_;

public:

	using Self = ShaderTransparentVolumes;
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ShaderTransparentVolumes);

	enum
	{
		ATTRIB_POS = 0,
	};

	void set_explode_volume(float32 x);
	void set_light_position(const QVector3D& l);
	void set_plane_clip(const QVector4D& plane);
	void set_plane_clip2(const QVector4D& plane);
	void set_color(const QColor& rgb);

	void set_bf_culling(bool cull);
	void set_lighted(bool lighted);
	void set_layer(int layer);
	void set_rgba_sampler(GLuint rgba_samp);
	void set_depth_sampler(GLuint depth_samp);

	using Param = ShaderParamTransparentVolumes;

	static std::unique_ptr<Param> generate_param();

protected:

	ShaderTransparentVolumes();

	static std::unique_ptr<ShaderTransparentVolumes> instance_;

};


class ShaderTranspQuad2;

class CGOGN_RENDERING_API ShaderParamTranspQuad2 : public ShaderParam
{
protected:
	void set_uniforms() override;
public:
	GLuint rgba_texture_sampler_;
	GLuint depth_texture_sampler_;
	ShaderParamTranspQuad2(ShaderTranspQuad2* sh);
};


class CGOGN_RENDERING_API ShaderTranspQuad2 : public ShaderProgram
{
	friend class ShaderParamTranspQuad2;

protected:

	static const char* vertex_shader_source_;
	static const char* fragment_shader_source_;

	// uniform ids
	GLint unif_depth_texture_sampler_;
	GLint unif_rgba_texture_sampler_;

public:

	using Self = ShaderTranspQuad2;
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ShaderTranspQuad2);

	void set_rgba_sampler(GLuint rgba_samp);

	void set_depth_sampler(GLuint depth_samp);

	using Param = ShaderParamTranspQuad2;

	static std::unique_ptr<Param> generate_param();

private:

	ShaderTranspQuad2();
	static std::unique_ptr<ShaderTranspQuad2> instance_;

};















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
		std::unique_ptr<cgogn::rendering::ShaderTranspQuad2::Param> param_trq_;

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
		void draw_faces(const QMatrix4x4& projection, const QMatrix4x4& modelview);

		void set_explode_volume(float32 x);
		void set_color(const QColor& rgb);
		void set_clipping_plane(const QVector4D& pl);
		void set_clipping_plane2(const QVector4D& pl);
		void set_thick_clipping_plane(const QVector4D& p, float32 th);
		void set_back_face_culling(bool cull);
		void set_lighted(bool lighted);
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


} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_VOLUME_DRAWER_H_

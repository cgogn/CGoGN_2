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
		GLuint oq_transp_;

		QOpenGLFunctions_3_3_Core* ogl33_;

		int width_;

		int height_;

		GLuint depthTexture_;

		Renderer(VolumeTransparencyDrawer* tr);

	public:
		/**
		 * @brief resize (call from interface::resizeGL)
		 * @param w
		 * @param h
		 * @param ogl33
		 */
		void resize(int w, int h, QOpenGLFunctions_3_3_Core* ogl33);


		~Renderer();

		/**
		 * @brief draw only the transparent volumes (bad mixing with opaque objects)
		 * @param projection projection matrix
		 * @param modelview modelview matrix
		 */
		void draw_faces(const QMatrix4x4& projection, const QMatrix4x4& modelview);


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
	inline std::unique_ptr<Renderer> generate_renderer()
	{
		return std::unique_ptr<Renderer>(new Renderer(this));
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

#endif // CGOGN_RENDERING_TRANSP_VOLUME_DRAWER_H_

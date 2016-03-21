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

#ifndef RENDERING_VOLUME_RENDER_H_
#define RENDERING_VOLUME_RENDER_H_

#include <rendering/shaders/shader_explode_volumes.h>
#include <rendering/shaders/shader_explode_volumes_line.h>
#include <rendering/shaders/vbo.h>
#include <rendering/dll.h>
#include <QOpenGLFunctions_3_3_Core>
#include <QColor>

#include <geometry/algos/centroid.h>
#include <geometry/algos/ear_triangulation.h>

namespace cgogn
{

namespace rendering
{


class CGOGN_RENDERING_API VolumeRender
{
	using Vec3f = std::array<float,3>;

protected:

	ShaderExplodeVolumes* shader_expl_vol_;

	ShaderExplodeVolumesLine* shader_expl_vol_line_;

	VBO* vbo_pos_;
	VBO* vbo_col_;
	unsigned int vao1_;
	QColor face_color_;


	VBO* vbo_pos2_;
	unsigned int vao2_;
	QColor edge_color_;


	QOpenGLFunctions_3_3_Core* ogl33_;

	float shrink_v_;
	float shrink_f_;

	void init_with_color();

	void init_without_color();

	void init_edge();

public:

	/**
	 * constructor, init all buffers (data and OpenGL) and shader
	 * @Warning need OpenGL context
	 */
	VolumeRender(QOpenGLFunctions_3_3_Core* ogl33);

	/**
	 * release buffers and shader
	 */
	~VolumeRender();

	inline void set_explode_face(float x) { shrink_f_ = x; }

	inline void set_explode_volume(float x) { shrink_v_ = x; }

	inline void set_face_color(const QColor& rgb) { face_color_= rgb; }

	inline void set_edge_color(const QColor& rgb) { edge_color_= rgb; }

	template <typename VEC3, typename MAP>
	void update_face(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position);

	template <typename VEC3, typename MAP>
	void update_face(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position,
				const typename MAP::template VertexAttributeHandler<VEC3>& color);

	template <typename VEC3, typename MAP>
	void update_edge(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position);

	void draw_faces(const QMatrix4x4& projection, const QMatrix4x4& modelview);

	void draw_edges(const QMatrix4x4& projection, const QMatrix4x4& modelview);
};


template <typename VEC3, typename MAP>
void VolumeRender::update_face(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position)
{
	init_without_color();

	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;
	using Volume = typename MAP::Volume;
	using Scalar = typename VEC3::Scalar;

	std::vector<std::array<float,3>> out_pos;
	out_pos.reserve(1024*1024);

	std::vector<unsigned int> ear_indices;
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
				out_pos.push_back({float(CV[0]),float(CV[1]),float(CV[2])});
				out_pos.push_back({float(P1[0]),float(P1[1]),float(P1[2])});
				out_pos.push_back({float(P2[0]),float(P2[1]),float(P2[2])});
				out_pos.push_back({float(P3[0]),float(P3[1]),float(P3[2])});
			}
			else
			{
				ear_indices.clear();
				cgogn::geometry::compute_ear_triangulation<VEC3>(m,f,position,ear_indices);
				for(std::size_t i = 0; i < ear_indices.size(); i += 3)
				{
					const VEC3& P1 = position[ear_indices[i]];
					const VEC3& P2 = position[ear_indices[i+1]];
					const VEC3& P3 = position[ear_indices[i+2]];
					out_pos.push_back({float(CV[0]),float(CV[1]),float(CV[2])});
					out_pos.push_back({float(P1[0]),float(P1[1]),float(P1[2])});
					out_pos.push_back({float(P2[0]),float(P2[1]),float(P2[2])});
					out_pos.push_back({float(P3[0]),float(P3[1]),float(P3[2])});
				}
			}
		});
	});

	unsigned int nbvec = std::uint32_t(out_pos.size());
	vbo_pos_->allocate(nbvec,3);
	vbo_pos_->bind();
	vbo_pos_->copy_data(0, nbvec*12, out_pos[0].data());
	vbo_pos_->release();

}

template <typename VEC3, typename MAP>
void VolumeRender::update_face(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position,
						  const typename MAP::template VertexAttributeHandler<VEC3>& color)
{
	init_with_color();

	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;
	using Volume = typename MAP::Volume;
	using Scalar = typename VEC3::Scalar;

	std::vector<std::array<float,3>> out_pos;
	out_pos.reserve(1024*1024);

	std::vector<std::array<float,3>> out_color;
	out_color.reserve(1024*1024);

	std::vector<unsigned int> ear_indices;
	ear_indices.reserve(256);

	m.foreach_cell([&] (Volume v)
	{
		VEC3 CV = geometry::centroid<VEC3>(m,v,position);
		m.foreach_incident_face(v, [&] (Face f)
		{
			if (m.has_degree(f,3))
			{
				Dart d = f.dart;
				const VEC3& P1 = position[Vertex(d)];
				const VEC3& C1 = color[Vertex(d)];
				d = m.phi1(d);
				const VEC3& P2 = position[Vertex(d)];
				const VEC3& C2 = color[Vertex(d)];
				d = m.phi1(d);
				const VEC3& P3 = position[Vertex(d)];
				const VEC3& C3 = color[Vertex(d)];
				out_pos.push_back({float(CV[0]),float(CV[1]),float(CV[2])});
				out_pos.push_back({float(P1[0]),float(P1[1]),float(P1[2])});
				out_pos.push_back({float(P2[0]),float(P2[1]),float(P2[2])});
				out_pos.push_back({float(P3[0]),float(P3[1]),float(P3[2])});
				out_color.push_back({float(CV[0]),float(CV[1]),float(CV[2])});
				out_color.push_back({float(C1[0]),float(C1[1]),float(C1[2])});
				out_color.push_back({float(C2[0]),float(C2[1]),float(C2[2])});
				out_color.push_back({float(C3[0]),float(C3[1]),float(C3[2])});
			}
			else
			{
				ear_indices.clear();
				cgogn::geometry::compute_ear_triangulation<VEC3>(m,f,position,ear_indices);
				for(std::size_t i=0; i<ear_indices.size(); i+=3)
				{

					const VEC3& P1 = position[ear_indices[i]];
					const VEC3& C1 = color[ear_indices[i]];
					const VEC3& P2 = position[ear_indices[i+1]];
					const VEC3& C2 = color[ear_indices[i+1]];
					const VEC3& P3 = position[ear_indices[i+2]];
					const VEC3& C3 = color[ear_indices[i+2]];
					out_pos.push_back({float(CV[0]),float(CV[1]),float(CV[2])});
					out_pos.push_back({float(P1[0]),float(P1[1]),float(P1[2])});
					out_pos.push_back({float(P2[0]),float(P2[1]),float(P2[2])});
					out_pos.push_back({float(P3[0]),float(P3[1]),float(P3[2])});
					out_color.push_back({float(CV[0]),float(CV[1]),float(CV[2])});
					out_color.push_back({float(C1[0]),float(C1[1]),float(C1[2])});
					out_color.push_back({float(C2[0]),float(C2[1]),float(C2[2])});
					out_color.push_back({float(C3[0]),float(C3[1]),float(C3[2])});
				}
			}
		});
	});

	std::size_t nbvec = out_pos.size();
	vbo_pos_->allocate(nbvec,3);
	vbo_pos_->bind();
	vbo_pos_->copy_data(0, nbvec*12, out_pos[0].data());
	vbo_pos_->release();

	vbo_col_->allocate(nbvec,3);
	vbo_col_->bind();
	vbo_col_->copy_data(0, nbvec*12, out_color[0].data());
	vbo_col_->release();

}



template <typename VEC3, typename MAP>
void VolumeRender::update_edge(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position)
{
	init_edge();

	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	using Volume = typename MAP::Volume;
	using Scalar = typename VEC3::Scalar;

	std::vector<std::array<float,3>> out_pos;
	out_pos.reserve(1024*1024);

	std::vector<unsigned int> ear_indices;
	ear_indices.reserve(256);

	m.foreach_cell([&] (Volume v)
	{
		VEC3 CV = geometry::centroid<VEC3>(m,v,position);
		m.foreach_incident_edge(v, [&] (Edge e)
		{
			const VEC3& P1 = position[Vertex(e.dart)];
			const VEC3& P2 = position[Vertex(m.phi1(e.dart))];
			out_pos.push_back({float(CV[0]),float(CV[1]),float(CV[2])});
			out_pos.push_back({float(P1[0]),float(P1[1]),float(P1[2])});
			out_pos.push_back({float(P2[0]),float(P2[1]),float(P2[2])});
		});
	});

	unsigned int nbvec = std::uint32_t(out_pos.size());
	vbo_pos2_->allocate(nbvec,3);
	vbo_pos2_->bind();
	vbo_pos2_->copy_data(0, nbvec*12, out_pos[0].data());
	vbo_pos2_->release();
}


} // namespace rendering

} // namespace cgogn

#endif // RENDERING_VOLUME_RENDER_H_

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

#ifndef RENDERING_TOPO_RENDER_H_
#define RENDERING_TOPO_RENDER_H_

#include <rendering/shaders/shader_simple_color.h>
#include <rendering/shaders/shader_bold_line.h>
#include <rendering/shaders/shader_round_point.h>
#include <rendering/shaders/vbo.h>
#include <rendering/dll.h>
#include <QOpenGLFunctions_3_3_Core>
#include <QColor>

#include <geometry/algos/centroid.h>

namespace cgogn
{

namespace rendering
{


class CGOGN_RENDERING_API TopoRender
{
	using Vec3f = std::array<float,3>;

protected:

	static ShaderSimpleColor* shader_cpv_;
	static ShaderBoldLine* shader_bl_;
	static ShaderRoundPoint* shader_rp_;
	static int nb_instances_;

	VBO* vbo_topo_;

	unsigned int vao_bl_;
	unsigned int vao_rp_;

	QOpenGLFunctions_3_3_Core* ogl33_;


	QColor dart_color_;
	QColor phi2_color_;
	QColor phi3_color_;

	float shrink_v_;
	float shrink_f_;
	float shrink_e_;

public:

	/**
	 * constructor, init all buffers (data and OpenGL) and shader
	 * @Warning need OpenGL context
	 */
	TopoRender(QOpenGLFunctions_3_3_Core* ogl33);

	/**
	 * release buffers and shader
	 */
	~TopoRender();

	template <typename VEC3, typename MAP>
	void update_map2(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position);

	template <typename VEC3, typename MAP>
	void update_map3(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position);


	void draw(const QMatrix4x4& projection, const QMatrix4x4& modelview, bool with_blending=true);
};

template <typename VEC3, typename MAP>
void TopoRender::update_map2(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position)
{
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;
	using Scalar = typename VEC3::Scalar;

	Scalar opp_shrink_e = 1.0 -shrink_e_;
	Scalar opp_shrink_f = 1.0 - shrink_f_;

	std::vector<std::array<float,3>> out_pos;
	out_pos.reserve(1024*1024);

	std::vector<std::array<float,3>> out_pos2;
	out_pos2.reserve(1024*1024);


	std::vector<VEC3> local_vertices;
	local_vertices.reserve(256);

	m.foreach_cell([&] (Face f)
	{
		local_vertices.clear();
		VEC3 center;
		center.setZero();
		unsigned int count = 0u;
		m.foreach_incident_vertex(f, [&] (Vertex v)
		{
			local_vertices.push_back(position[v]);
			center += position[v];
			count++;
		});
		center /= Scalar(count);

		// phi2 mid-edge: N -> 2N-1
		for (unsigned int i=0; i<count; ++i)
			local_vertices.push_back((local_vertices[i]+local_vertices[(i+1)%count])/Scalar(2.0));

		// dart round point: 0 -> N-1
		for (unsigned int i=0; i<count; ++i)
			local_vertices[i] = local_vertices[i] * Scalar(shrink_f_) + center * (opp_shrink_f);

		//dart other extremety: 2N -> 3N-1
		for (unsigned int i=0; i<count; ++i)
			local_vertices.push_back(local_vertices[i]*(opp_shrink_e) + local_vertices[(i+1)%count]*Scalar(shrink_e_));

		//phi2 mid-dart: 3N -> 4N-1
		for (unsigned int i=0; i<count; ++i)
			local_vertices.push_back((local_vertices[i]+local_vertices[(2*count+i+1)%count])/Scalar(2.0));

		for (unsigned int i=0; i<count; ++i)
		{
			const VEC3& P1 = local_vertices[i];
			out_pos.push_back({float(P1[0]),float(P1[1]),float(P1[2])});
			const VEC3& P2 = local_vertices[2*count+i];
			out_pos.push_back({float(P2[0]),float(P2[1]),float(P2[2])});
			const VEC3& P3 = local_vertices[count+i];
			out_pos2.push_back({float(P3[0]),float(P3[1]),float(P3[2])});
			const VEC3& P4 = local_vertices[3*count+i];
			out_pos2.push_back({float(P4[0]),float(P4[1]),float(P4[2])});
		}
	});

	std::size_t nbvec = out_pos.size();
	vbo_topo_->allocate(nbvec*2,3);
	vbo_topo_->bind();
	vbo_topo_->copy_data(0, nbvec*12, out_pos[0].data());
	vbo_topo_->copy_data(nbvec*12, nbvec*12, out_pos2[0].data());
	vbo_topo_->bind();

}


template <typename VEC3, typename MAP>
void TopoRender::update_map3(MAP& m, const typename MAP::template VertexAttributeHandler<VEC3>& position)
{
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;
	using Volume = typename MAP::Volume;
	using Scalar = typename VEC3::Scalar;

	Scalar opp_shrink_e = 1.0 -shrink_e_;
	Scalar opp_shrink_f = 1.0 - shrink_f_;
	Scalar opp_shrink_v = 1.0 - shrink_v_;

	std::vector<std::array<float,3>> out_pos;
	out_pos.reserve(1024*1024);

	std::vector<std::array<float,3>> out_pos2;
	out_pos2.reserve(1024*1024);


	std::vector<VEC3> local_vertices;
	local_vertices.reserve(256);

	m.foreach_cell([&] (Volume v)
	{
		VEC3 center_vol = geometry::centroid<VEC3>(m,v,position);
		m.foreach_incident_face(v, [&] (Face f)
		{
			local_vertices.clear();
			VEC3 center;
			center.setZero();
			unsigned int count = 0u;
			m.foreach_incident_vertex(f, [&] (Vertex v)
			{
				local_vertices.push_back(position[v]);
				center += position[v];
				count++;
			});
			center /= Scalar(count);

			// phi2 mid-edge: N -> 2N-1
			for (unsigned int i=0; i<count; ++i)
				local_vertices.push_back((local_vertices[i]+local_vertices[(i+1)%count])/Scalar(2.0));

			//phi3: 2N -> 3N-1
			for (unsigned int i=0; i<count; ++i)
				local_vertices.push_back(local_vertices[count+i]* shrink_f_ + center * (opp_shrink_f));

			// dart round point: 0 -> N-1
			for (unsigned int i=0; i<count; ++i)
				local_vertices[i] = local_vertices[i] * shrink_f_ + center * (opp_shrink_f);

			//dart other extremety: 3N -> 4N-1
			for (unsigned int i=0; i<count; ++i)
				local_vertices.push_back(local_vertices[i]*(opp_shrink_e) + local_vertices[(i+1)%count]*shrink_e_);

			//phi2/3 mid-dart: 4N -> 5N-1
			for (unsigned int i=0; i<count; ++i)
				local_vertices.push_back((local_vertices[i]+local_vertices[(2*count+i+1)%count])/Scalar(2.0));

			for (unsigned int i=0; i<count; ++i)
			{
				const VEC3 P1 = local_vertices[i] * shrink_v_ + center_vol * (opp_shrink_v);
				out_pos.push_back({float(P1[0]),float(P1[1]),float(P1[2])});
				const VEC3 P2 = local_vertices[3*count+i] * shrink_v_ + center_vol * (opp_shrink_v);
				out_pos.push_back({float(P2[0]),float(P2[1]),float(P2[2])});
				const VEC3 P3 = local_vertices[count+i] * shrink_v_ + center_vol * (opp_shrink_v);
				out_pos2.push_back({float(P3[0]),float(P3[1]),float(P3[2])});
				const VEC3 P4 = local_vertices[4*count+i] * shrink_v_ + center_vol * (opp_shrink_v);
				out_pos2.push_back({float(P4[0]),float(P4[1]),float(P4[2])});
				const VEC3& P5 = local_vertices[2*count+i];
				out_pos2.push_back({float(P5[0]),float(P5[1]),float(P5[2])});
				out_pos2.push_back({float(P4[0]),float(P4[1]),float(P4[2])});
			}
		});

	});

	std::size_t nbvec = out_pos.size();
	vbo_topo_->allocate(nbvec*3,3);
	vbo_topo_->bind();
	vbo_topo_->copy_data(0, nbvec*12, out_pos[0].data());
	vbo_topo_->copy_data(nbvec*12, nbvec*24, out_pos2[0].data());
	vbo_topo_->bind();
}

} // namespace rendering

} // namespace cgogn

#endif // RENDERING_TOPO_RENDER_H_

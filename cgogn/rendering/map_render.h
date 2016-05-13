
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

#ifndef CGOGN_RENDERING_MAP_RENDER_H_
#define CGOGN_RENDERING_MAP_RENDER_H_

#include <cgogn/rendering/dll.h>

#include <cgogn/core/cmap/map_base.h> // impossible to include directly attribute_handler.h !

#include <cgogn/geometry/algos/ear_triangulation.h>

#include <cgogn/rendering/drawer.h>
#include <cgogn/rendering/shaders/vbo.h>

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>

namespace cgogn
{

namespace rendering
{

enum DrawingType
{
	POINTS = 0,
	LINES,
	TRIANGLES,
	SIZE_BUFFER
};

class CGOGN_RENDERING_API MapRender
{
protected:

	std::array<std::unique_ptr<QOpenGLBuffer>, SIZE_BUFFER>	indices_buffers_;
	std::array<bool, SIZE_BUFFER>							indices_buffers_uptodate_;
	std::array<uint32, SIZE_BUFFER>							nb_indices_;

public:

	using Self = MapRender;

	MapRender();
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MapRender);
	~MapRender();

	inline bool is_primitive_uptodate(DrawingType prim) { return indices_buffers_uptodate_[prim]; }

	inline void set_primitive_dirty(DrawingType prim) { indices_buffers_uptodate_[prim] = false; }

	template <typename MAP>
	inline void init_points(const MAP& m, std::vector<uint32>& table_indices)
	{
//		table_indices.reserve(m.get_nb_darts() / 6);
		m.foreach_cell([&] (typename MAP::Vertex v)
		{
			table_indices.push_back(m.get_embedding(v));
		});
	}

	template <typename MAP>
	inline void init_lines(const MAP& m, std::vector<uint32>& table_indices)
	{
		using Vertex = typename MAP::Vertex;
		using Edge = typename MAP::Edge;
//		table_indices.reserve(m.get_nb_darts() / 2);
		m.foreach_cell([&] (Edge e)
		{
			table_indices.push_back(m.get_embedding(Vertex(e.dart)));
			table_indices.push_back(m.get_embedding(Vertex(m.phi1(e.dart))));
		});
	}

	template <typename VEC3, typename MAP>
	inline void init_triangles(const MAP& m, std::vector<uint32>& table_indices)
	{
		using Vertex = typename MAP::Vertex;
		using Face = typename MAP::Face;
		// reserve more ?
//		table_indices.reserve(m.get_nb_darts() / 3);
		m.foreach_cell([&] (Face f)
		{
			Dart d0 = f.dart;
			Dart d1 = m.phi1(d0);
			Dart d2 = m.phi1(d1);
			do
			{
				table_indices.push_back(m.get_embedding(Vertex(d0)));
				table_indices.push_back(m.get_embedding(Vertex(d1)));
				table_indices.push_back(m.get_embedding(Vertex(d2)));
				d1 = d2;
				d2 = m.phi1(d1);
			} while(d2 != d0);
		});
	}

	template <typename VEC3, typename MAP>
	inline void init_triangles_ear(const MAP& m, std::vector<uint32>& table_indices, const typename MAP::template VertexAttribute<VEC3>* position)
	{
		using Vertex = typename MAP::Vertex;
		using Face = typename MAP::Face;
		// reserve more ?
//		table_indices.reserve(m.get_nb_darts() / 3);
		m.foreach_cell([&] (Face f)
		{
			if (m.has_codegree(f, 3))
			{
				table_indices.push_back(m.get_embedding(Vertex(f.dart)));
				table_indices.push_back(m.get_embedding(Vertex(m.phi1(f.dart))));
				table_indices.push_back(m.get_embedding(Vertex(m.phi1(m.phi1(f.dart)))));
			}
			else
				cgogn::geometry::compute_ear_triangulation<VEC3>(m, f, *position, table_indices);
		});
	}

	template <typename VEC3, typename MAP>
	inline void init_primitives(const MAP& m, DrawingType prim, const typename MAP::template VertexAttribute<VEC3>* position = nullptr)
	{
		std::vector<uint32> table_indices;

		switch (prim)
		{
			case POINTS:
				init_points(m, table_indices);
				break;
			case LINES:
				init_lines(m, table_indices);
				break;
			case TRIANGLES:
				if (position)
					init_triangles_ear<VEC3>(m, table_indices, position);
				else
					init_triangles<VEC3>(m, table_indices);
				break;
			default:
				break;
		}

		if (!indices_buffers_[prim]->isCreated())
			indices_buffers_[prim]->create();

		indices_buffers_uptodate_[prim] = true;
		nb_indices_[prim] = uint32(table_indices.size());
		indices_buffers_[prim]->bind();
		indices_buffers_[prim]->allocate(&(table_indices[0]), nb_indices_[prim] * sizeof(uint32));
		indices_buffers_[prim]->release();
	}

	void draw(DrawingType prim);
};

/**
 * @brief create embedding indices of vertices and faces for arch vertx of each face
 * @param m
 * @param position vertex positions use for ear triangulation)
 * @param indices1 embedding indices of vertices
 * @param indices2 embedding indices of faces
 */
template <typename VEC3, typename MAP>
void create_indices_vertices_faces(const MAP& m, const typename MAP::template VertexAttribute<VEC3>& position, std::vector<uint32>& indices1, std::vector<uint32>& indices2)
{
	using Vertex = typename MAP::Vertex;
	using Face = typename MAP::Face;

	indices1.reserve(m.nb_darts());
	indices2.reserve(m.nb_darts());
	indices1.clear();
	indices2.clear();

	//local vector for ear triangulation
	std::vector<uint32> local_vert_indices;
	local_vert_indices.reserve(256);

	m.foreach_cell([&] (Face f)
	{
		uint32 ef = m.get_embedding(Face(f.dart));
		if (m.has_codegree(f, 3))
		{
			indices1.push_back(m.get_embedding(Vertex(f.dart)));
			indices1.push_back(m.get_embedding(Vertex(m.phi1(f.dart))));
			indices1.push_back(m.get_embedding(Vertex(m.phi1(m.phi1(f.dart)))));
			indices2.push_back(ef);
			indices2.push_back(ef);
			indices2.push_back(ef);
		}
		else
		{
			cgogn::geometry::compute_ear_triangulation<VEC3>(m, f, position, local_vert_indices);
			for (uint32 i : local_vert_indices)
			{
				indices1.push_back(i);
				indices2.push_back(ef);
			}
		}
	});
}

template <typename VEC3, typename MAP>
void add_edge_to_drawer(const MAP& m, typename MAP::Edge e, const typename MAP::template VertexAttribute<VEC3>& position, DisplayListDrawer* dr)
{
	using Vertex = typename MAP::Vertex;
	dr->vertex3fv(position[Vertex(e.dart)]);
	dr->vertex3fv(position[Vertex(m.phi1(e.dart))]);
}

template <typename VEC3, typename MAP>
void add_face_to_drawer(const MAP& m, typename MAP::Face f, const typename MAP::template VertexAttribute<VEC3>& position, DisplayListDrawer* dr)
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	m.foreach_incident_edge(f, [&] (Edge e)
	{
		dr->vertex3fv(position[Vertex(e.dart)]);
		dr->vertex3fv(position[Vertex(m.phi1(e.dart))]);
	});
}

template <typename VEC3, typename MAP>
void add_volume_to_drawer(const MAP& m, typename MAP::Volume vo, const typename MAP::template VertexAttribute<VEC3>& position, DisplayListDrawer* dr)
{
	using Vertex = typename MAP::Vertex;
	using Edge = typename MAP::Edge;
	m.foreach_incident_edge(vo, [&] (Edge e)
	{
		dr->vertex3fv(position[Vertex(e.dart)]);
		dr->vertex3fv(position[Vertex(m.phi1(e.dart))]);
	});
}

} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_MAP_RENDER_H_

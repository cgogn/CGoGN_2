
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

#include <cgogn/core/cmap/map_base.h> // impossible to include directly attribute.h !

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
	BOUNDARY,
	SIZE_BUFFER
};

class CGOGN_RENDERING_API MapRender
{
protected:

	// indices used for sorting
	std::vector<std::array<uint32,3>> indices_tri_;
	std::vector<uint32> indices_points_;

	std::array<std::unique_ptr<QOpenGLBuffer>, SIZE_BUFFER>	indices_buffers_;
	std::array<bool, SIZE_BUFFER>							indices_buffers_uptodate_;
	std::array<uint32, SIZE_BUFFER>							nb_indices_;
	uint8													boundary_dimension_;

public:

	using Self = MapRender;

	MapRender();
	~MapRender();
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MapRender);

	inline bool is_primitive_uptodate(DrawingType prim) { return indices_buffers_uptodate_[prim]; }

	inline void set_primitive_dirty(DrawingType prim) { indices_buffers_uptodate_[prim] = false; }

protected:

	template <typename MAP, typename MASK>
	inline void init_points(const MAP& m, const MASK& mask, std::vector<uint32>& table_indices)
	{
//		table_indices.reserve(m.get_nb_darts() / 6);
		m.foreach_cell([&] (typename MAP::Vertex v)
		{
			table_indices.push_back(m.embedding(v));
		},
		mask);
	}

	template <typename MAP, typename MASK>
	inline void init_lines(const MAP& m, const MASK& mask, std::vector<uint32>& table_indices)
	{
		using Vertex = typename MAP::Vertex;
		using Edge = typename MAP::Edge;

//		table_indices.reserve(m.get_nb_darts() / 2);
		m.foreach_cell([&] (Edge e)
		{
			table_indices.push_back(m.embedding(Vertex(e.dart)));
			table_indices.push_back(m.embedding(Vertex(m.phi1(e.dart))));
		},
		mask);
	}

	template <typename MAP, typename MASK>
	inline void init_triangles(const MAP& m, const MASK& mask, std::vector<uint32>& table_indices)
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
				table_indices.push_back(m.embedding(Vertex(d0)));
				table_indices.push_back(m.embedding(Vertex(d1)));
				table_indices.push_back(m.embedding(Vertex(d2)));
				d1 = d2;
				d2 = m.phi1(d1);
			} while(d2 != d0);
		},
		mask);
	}

	template <typename VEC3, typename MAP, typename MASK>
	inline void init_triangles_ear(
		const MAP& m,
		const MASK& mask,
		std::vector<uint32>& table_indices,
		const typename MAP::template VertexAttribute<VEC3>* position
	)
	{
		using Vertex = typename MAP::Vertex;
		using Face = typename MAP::Face;

		// reserve more ?
//		table_indices.reserve(m.get_nb_darts() / 3);
		m.foreach_cell([&] (Face f)
		{
			if (m.has_codegree(f, 3))
			{
				table_indices.push_back(m.embedding(Vertex(f.dart)));
				table_indices.push_back(m.embedding(Vertex(m.phi1(f.dart))));
				table_indices.push_back(m.embedding(Vertex(m.phi1(m.phi1(f.dart)))));
			}
			else
				cgogn::geometry::append_ear_triangulation<VEC3>(m, f, *position, table_indices);
		},
		mask);
	}

	template <typename MAP, typename MASK>
	inline auto init_boundaries(const MAP& m, const MASK& mask, std::vector<uint32>& table_indices)
		-> typename std::enable_if<MAP::DIMENSION == 2 && std::is_same<MASK, typename MAP::BoundaryCache>::value, void>::type
	{
		using Vertex = typename MAP::Vertex;
		using Edge = typename MAP::Edge;
		using Face = typename MAP::Face;

		m.foreach_cell([&] (Face f)
		{
			m.foreach_incident_edge(f, [&] (Edge e)
			{
				table_indices.push_back(m.embedding(Vertex(e.dart)));
				table_indices.push_back(m.embedding(Vertex(m.phi1(e.dart))));
			});
		},
		mask);

		boundary_dimension_ = 1;
	}

	template <typename MAP, typename MASK>
	inline auto init_boundaries(const MAP& m, const MASK& /*mask*/, std::vector<uint32>& table_indices)
		-> typename std::enable_if<MAP::DIMENSION == 2 && !std::is_same<MASK, typename MAP::BoundaryCache>::value, void>::type
	{
		// if the given MASK is not a BoundaryCache, build a BoundaryCache and use it
		typename MAP::BoundaryCache bcache(m);
		init_boundaries(m, bcache, table_indices);
	}

	template <typename MAP, typename MASK>
	inline auto init_boundaries(const MAP& m, const MASK& mask, std::vector<uint32>& table_indices)
		-> typename std::enable_if<MAP::DIMENSION == 3 && std::is_same<MASK, typename MAP::BoundaryCache>::value, void>::type
	{
		using Vertex = typename MAP::Vertex;
		using Face = typename MAP::Face;
		using Volume = typename MAP::Volume;

		m.foreach_cell([&] (Volume v)
		{
			m.foreach_incident_face(v, [&] (Face f)
			{
				Dart d0 = f.dart;
				Dart d1 = m.phi1(d0);
				Dart d2 = m.phi1(d1);
				do
				{
					table_indices.push_back(m.embedding(Vertex(d0)));
					table_indices.push_back(m.embedding(Vertex(d1)));
					table_indices.push_back(m.embedding(Vertex(d2)));
					d1 = d2;
					d2 = m.phi1(d1);
				} while(d2 != d0);
			});
		},
		mask);

		boundary_dimension_ = 2;
	}

	template <typename MAP, typename MASK>
	inline auto init_boundaries(const MAP& m, const MASK& mask, std::vector<uint32>& table_indices)
		-> typename std::enable_if<MAP::DIMENSION == 3 && !std::is_same<MASK, typename MAP::BoundaryCache>::value, void>::type
	{
		unused_parameters(mask);
		// if the given MASK is not a BoundaryCache, build a BoundaryCache and use it
		typename MAP::BoundaryCache bcache(m);
		init_boundaries(m, bcache, table_indices);
	}

public:

	inline uint32 nb_indices(DrawingType prim) const
	{
		return nb_indices_[prim];
	}


	template <typename VEC3, typename MAP, typename MASK>
	inline void init_primitives(
		const MAP& m,
		const MASK& mask,
		DrawingType prim,
		const typename MAP::template VertexAttribute<VEC3>* position
	)
	{
		std::vector<uint32> table_indices;

		switch (prim)
		{
			case POINTS:
				init_points(m, mask, table_indices);
				indices_points_.clear();
				break;
			case LINES:
				init_lines(m, mask, table_indices);
				break;
			case TRIANGLES:
				if (position)
					init_triangles_ear<VEC3>(m, mask, table_indices, position);
				else
					init_triangles(m, mask, table_indices);
				indices_tri_.clear();
				break;
			case BOUNDARY:
				init_boundaries(m, mask, table_indices);
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

	template <typename VEC3, typename MAP>
	inline void init_primitives(
		const MAP& m,
		DrawingType prim,
		const typename MAP::template VertexAttribute<VEC3>* position
	)
	{
		init_primitives<VEC3>(m, AllCellsFilter(), prim, position);
	}

	template <typename MAP, typename MASK>
	inline void init_primitives(
		const MAP& m,
		const MASK& mask,
		DrawingType prim
	)
	{
		std::vector<uint32> table_indices;

		switch (prim)
		{
			case POINTS:
				init_points(m, mask, table_indices);
				indices_points_.clear();
				break;
			case LINES:
				init_lines(m, mask, table_indices);
				break;
			case TRIANGLES:
				init_triangles(m, mask, table_indices);
				indices_tri_.clear();
				break;
			case BOUNDARY:
				init_boundaries(m, mask, table_indices);
				break;
			default:
				break;
		}

		if (table_indices.empty())
			return;

		if (!indices_buffers_[prim]->isCreated())
			indices_buffers_[prim]->create();

		indices_buffers_uptodate_[prim] = true;
		nb_indices_[prim] = uint32(table_indices.size());
		indices_buffers_[prim]->bind();
		indices_buffers_[prim]->allocate(&(table_indices[0]), nb_indices_[prim] * sizeof(uint32));
		indices_buffers_[prim]->release();
	}

	template <typename MAP>
	inline void init_primitives(
		const MAP& m,
		DrawingType prim
	)
	{
		init_primitives(m, AllCellsFilter(), prim);
	}

	void draw(DrawingType prim);
};

/**
 * @brief transform position with modelview matrix
 * @param map
 * @param pos_in input position
 * @param pos_out transformed positions
 * @param view modelview matrix
 */
template <typename VEC3, typename MAP>
void transform_position(const MAP& map, const typename MAP::template VertexAttribute<VEC3>& pos_in, typename MAP::template VertexAttribute<VEC3>& pos_out, const QMatrix4x4& view)
{
	map.template const_attribute_container<MAP::Vertex::ORBIT>().parallel_foreach_index( [&] (uint32 i)
	{
		QVector3D P = view.map(QVector3D(pos_in[i][0],pos_in[i][1],pos_in[i][2]));
		pos_out[i]= VEC3(P[0],P[1],P[2]);
	});
}

/**
 * @brief create embedding indices of vertices and faces for each vertex of each face
 * @param m the map
 * @param position vertex positions use for ear triangulation
 * @param indices1 embedding indices of vertices
 * @param indices2 embedding indices of faces
 */
template <typename VEC3, typename MAP>
void create_indices_vertices_faces(
	const MAP& m,
	const typename MAP::template VertexAttribute<VEC3>& position,
	std::vector<uint32>& indices1,
	std::vector<uint32>& indices2
)
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
		uint32 ef = m.embedding(Face(f.dart));
		if (m.has_codegree(f, 3))
		{
			indices1.push_back(m.embedding(Vertex(f.dart)));
			indices1.push_back(m.embedding(Vertex(m.phi1(f.dart))));
			indices1.push_back(m.embedding(Vertex(m.phi1(m.phi1(f.dart)))));
			indices2.push_back(ef);
			indices2.push_back(ef);
			indices2.push_back(ef);
		}
		else
		{
			cgogn::geometry::append_ear_triangulation<VEC3>(m, f, position, local_vert_indices);
			for (uint32 i : local_vert_indices)
			{
				indices1.push_back(i);
				indices2.push_back(ef);
			}
		}
	});
}

template <typename VEC3, typename MAP>
void add_to_drawer(const MAP& m, typename MAP::Edge e, const typename MAP::template VertexAttribute<VEC3>& position, DisplayListDrawer* dr)
{
	using Vertex = typename MAP::Vertex;

	dr->vertex3fv(position[Vertex(e.dart)]);
	dr->vertex3fv(position[Vertex(m.phi1(e.dart))]);
}

template <typename VEC3, typename MAP>
void add_to_drawer(const MAP& m, typename MAP::Face f, const typename MAP::template VertexAttribute<VEC3>& position, DisplayListDrawer* dr)
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
void add_to_drawer(const MAP& m, typename MAP::Volume vo, const typename MAP::template VertexAttribute<VEC3>& position, DisplayListDrawer* dr)
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

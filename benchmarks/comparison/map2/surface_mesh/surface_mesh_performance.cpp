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

#include "surface_mesh_performance.h"

bool Performance2_SurfaceMesh::read_mesh(const std::string& filename)
{
	const bool res = mesh->read(filename);
	points   = mesh->vertex_property<Point>("v:point");
	vnormals = mesh->vertex_property<Point>("v:normal");
	fnormals = mesh->face_property<Point>("f:normal");
	return res;
}

void Performance2_SurfaceMesh::clear_mesh()
{
	mesh.reset(new Surface_mesh());
}

BENCHMARK_F(Performance2_SurfaceMesh, circulator)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		Surface_mesh::Vertex_iterator vit, vend=mesh->vertices_end();
		Surface_mesh::Face_iterator   fit, fend=mesh->faces_end();

		Surface_mesh::Face_around_vertex_circulator   vfit, vfend;
		Surface_mesh::Vertex_around_face_circulator   fvit, fvend;
		int counter = 0;

		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
		{
			vfit = vfend = mesh->faces(*vit);
			if (vfit) do
			{
				++counter;
			}
			while (++vfit != vfend);
		}

		for (fit=mesh->faces_begin(); fit!=fend; ++fit)
		{
			fvit = fvend = mesh->vertices(*fit);
			do
			{
				--counter;
			}
			while (++fvit != fvend);
		}
	}
}

BENCHMARK_F(Performance2_SurfaceMesh, barycenter)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		Surface_mesh::Vertex_iterator vit, vend=mesh->vertices_end();

		Point p(0,0,0);

		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
			p += points[*vit];

		p /= mesh->n_vertices();

		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
			points[*vit] -= p;
	}
}

BENCHMARK_F(Performance2_SurfaceMesh, normal)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		Surface_mesh::Vertex_iterator vit, vend=mesh->vertices_end();
		Surface_mesh::Face_iterator   fit, fend=mesh->faces_end();
		Surface_mesh::Face_around_vertex_circulator vfit, vfend;

		for (fit=mesh->faces_begin(); fit!=fend; ++fit)
		{
			Surface_mesh::Halfedge h = mesh->halfedge(*fit);
			Point p0 = points[mesh->to_vertex(h)];
			h = mesh->next_halfedge(h);
			Point p1 = points[mesh->to_vertex(h)];
			p1 -= p0;
			h = mesh->next_halfedge(h);
			Point p2 = points[mesh->to_vertex(h)];
			p2 -= p0;
			fnormals[*fit] = cross(p1, p2).normalize();
		}

		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
		{
			Point n(0,0,0);
			vfit = vfend = mesh->faces(*vit);
			if (vfit) do
			{
				n += fnormals[*vfit];
			}
			while (++vfit != vfend);
			vnormals[*vit] = n.normalize();
		}
	}
}

BENCHMARK_F(Performance2_SurfaceMesh, smoothing)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		Surface_mesh::Vertex_iterator vit, vend=mesh->vertices_end();
		Surface_mesh::Vertex_around_vertex_circulator vvit, vvend;

		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
		{
			if (!mesh->is_boundary(*vit))
			{
				Point  p(0,0,0);
				Scalar c(0);
				vvit = vvend = mesh->vertices(*vit);
				do
				{
					p += points[*vvit];
					++c;
				}
				while (++vvit != vvend);
				p /= c;
				points[*vit] = p;
			}
		}
	}
}

BENCHMARK_F(Performance2_SurfaceMesh, subdivision)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		state.ResumeTiming();
		// reserve memory
		int nv = mesh->n_vertices();
		int ne = mesh->n_edges();
		int nf = mesh->n_faces();
		mesh->reserve(nv+nf, ne+3*nf, 3*nf);


		// iterators
		Surface_mesh::Vertex_iterator vit, vend=mesh->vertices_end();
		Surface_mesh::Face_iterator fit, fend=mesh->faces_end();
		Surface_mesh::Edge_iterator eit, eend=mesh->edges_end();


		// compute new positions of old vertices
		Surface_mesh::Vertex_property<Point> new_pos = mesh->add_vertex_property<Point>("v:np");
		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
		{
		  if (!mesh->is_boundary(*vit))
		  {
			Scalar n = mesh->valence(*vit);
			Scalar alpha = (4.0 - 2.0*cos(2.0*M_PI/n)) / 9.0;
			Point  p(0,0,0);
			Surface_mesh::Vertex_around_vertex_circulator vvit=mesh->vertices(*vit), vvend=vvit;
			do
			{
			  p += points[*vvit];
			}
			while (++vvit != vvend);
			p = (1.0f-alpha)*points[*vit] + alpha/n*p;
			new_pos[*vit] = p;
		  }
		}


		// split faces
		for (fit=mesh->faces_begin(); fit!=fend; ++fit)
		{
		  Point  p(0,0,0);
		  Scalar c(0);
		  Surface_mesh::Vertex_around_face_circulator fvit = mesh->vertices(*fit), fvend=fvit;
		  do
		  {
			p += points[*fvit];
			++c;
		  }
		  while (++fvit!=fvend);
		  p /= c;

		  mesh->split(*fit, p);
		}


		// set new positions of old vertices
		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
		  if (!mesh->is_boundary(*vit))
			points[*vit] = new_pos[*vit];
		mesh->remove_vertex_property(new_pos);


		// flip old edges
		for (eit=mesh->edges_begin(); eit!=eend; ++eit)
		  if (mesh->is_flip_ok(*eit))
			mesh->flip(*eit);
	}
}

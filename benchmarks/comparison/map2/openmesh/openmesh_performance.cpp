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

#include "openmesh_performance.h"

void Performance2_OpenMesh::clear_mesh()
{
	mesh.reset(new Mesh());
}

bool Performance2_OpenMesh::read_mesh(const std::string& filename)
{
	const bool res = OpenMesh::IO::read_mesh(*mesh, filename);
	mesh->request_vertex_status();
	mesh->request_edge_status();
	mesh->request_face_status();
	mesh->request_vertex_normals();
	mesh->request_face_normals();
	return res;
}


BENCHMARK_F(Performance2_OpenMesh, circulator)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		Mesh::VIter vit, vend=mesh->vertices_end();
		Mesh::FIter fit, fend=mesh->faces_end();
		int counter = 0;

		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
			for (Mesh::VFIter vfit=mesh->vf_iter(vit); vfit; ++vfit)
				++counter;

		for (fit=mesh->faces_begin(); fit!=fend; ++fit)
			for (Mesh::FVIter fvit=mesh->fv_iter(fit); fvit; ++fvit)
				--counter;
	}
}

BENCHMARK_F(Performance2_OpenMesh, barycenter)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		Mesh::VIter vit, vend=mesh->vertices_end();
		Mesh::Point p(0,0,0);

		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
			p += mesh->point(vit);

		p /= mesh->n_vertices();

		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
			mesh->point(vit) -= p;
	}
}

BENCHMARK_F(Performance2_OpenMesh, normal)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		Mesh::VIter vit, vend=mesh->vertices_end();
		Mesh::FIter fit, fend=mesh->faces_end();
		Mesh::VFIter vfit;

		for (fit=mesh->faces_begin(); fit!=fend; ++fit)
		{
			Mesh::HalfedgeHandle h = mesh->halfedge_handle(fit);
			Mesh::Point p0 = mesh->point(mesh->to_vertex_handle(h));
			h = mesh->next_halfedge_handle(h);
			Mesh::Point p1 = mesh->point(mesh->to_vertex_handle(h));
			h = mesh->next_halfedge_handle(h);
			Mesh::Point p2 = mesh->point(mesh->to_vertex_handle(h));
			mesh->set_normal(fit, ((p2-=p1)%(p0-=p1)).normalize());
		}

		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
		{
			Mesh::Point n(0,0,0);
			for (vfit=mesh->vf_iter(vit); vfit; ++vfit)
				n += mesh->normal(vfit);
			mesh->set_normal(vit, n.normalize());
		}
	}
}

BENCHMARK_F(Performance2_OpenMesh, smoothing)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		Mesh::VIter vit, vend=mesh->vertices_end();

		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
		{
			if (!mesh->is_boundary(vit))
			{
				Mesh::Point  p(0,0,0);
				Mesh::Scalar c(0);
				for (Mesh::VVIter vvit = mesh->vv_iter(vit); vvit; ++vvit)
				{
					p += mesh->point(vvit);
					++c;
				}
				p /= c;
				mesh->point(vit) = p;
			}
		}
	}
}

BENCHMARK_F(Performance2_OpenMesh, subdivision)(benchmark::State& state)
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
		Mesh::VIter vit, vend=mesh->vertices_end();
		Mesh::FIter fit, fend=mesh->faces_end();
		Mesh::EIter eit, eend=mesh->edges_end();

		// compute new positions of old vertices
		OpenMesh::VPropHandleT<Mesh::Point> new_pos;
		mesh->add_property(new_pos);
		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
		{
			if (!mesh->is_boundary(vit))
			{
				Mesh::Scalar n = mesh->valence(vit);
				Mesh::Scalar alpha = (4.0 - 2.0*cos(2.0*M_PI/n)) / 9.0;
				Mesh::Point  p(0,0,0);
				for (Mesh::VVIter vvit = mesh->vv_iter(vit); vvit; ++vvit)
					p += mesh->point(vvit);
				p = (1.0f-alpha)*mesh->point(vit) + alpha/n*p;
				mesh->property(new_pos, vit) = p;
			}
		}

		// split faces
		for (fit=mesh->faces_begin(); fit!=fend; ++fit)
		{
			Mesh::Point  p(0,0,0);
			Mesh::Scalar c(0);
			for (Mesh::FVIter fvit=mesh->fv_iter(fit); fvit; ++fvit)
			{
				p += mesh->point(fvit);
				++c;
			}
			p /= c;

			mesh->split(fit, p);
		}

		// set new positions of old vertices
		for (vit=mesh->vertices_begin(); vit!=vend; ++vit)
			if (!mesh->is_boundary(vit))
				mesh->point(vit) = mesh->property(new_pos, vit);
		mesh->remove_property(new_pos);


		// flip old edges
		for (eit=mesh->edges_begin(); eit!=eend; ++eit)
			if (mesh->is_flip_ok(eit))
				mesh->flip(eit);
	}
}

BENCHMARK_F(Performance2_OpenMesh, collapse)(benchmark::State& state)
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
		Mesh::VIter vit, vend=mesh->vertices_end();
		Mesh::FIter fit, fend=mesh->faces_end();

		// split faces
		Mesh::Point p(0,0,0);
		for (fit=mesh->faces_begin(); fit!=fend; ++fit)
		  mesh->split(fit, p);

		// collapse new edges
		vit = vend; vend=mesh->vertices_end();
		for (; vit!=vend; ++vit)
		  mesh->collapse(mesh->halfedge_handle(vit));

		// remove deleted items
		mesh->garbage_collection();
	}
}

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

#include "openvolumemesh_performance.h"
#include <cgogn/core/utils/string.h>

using namespace OpenVolumeMesh;

bool Performance3_OpenVolumeMesh::read_mesh(const std::string& filename)
{
	const std::string new_name = cgogn::remove_extension(filename) + std::string(".ovm");
	const bool checkTopo = false;
	const bool computeBottomUpIncidences = true; //  Incidence Dependent Iterators need bottom-up incidences to be computed before they can be used.

	OpenVolumeMesh::IO::FileManager fileManager;
	const bool res = fileManager.readFile(new_name, *mesh, checkTopo, computeBottomUpIncidences);

	return res;
}

void Performance3_OpenVolumeMesh::clear_mesh()
{
	mesh.reset(new Mesh());
}

void Performance3_OpenVolumeMesh::edge_collapse(const EdgeHandle& _eh)
{
	Vec3 p = mesh->barycenter(_eh);

	// Get vertices incident to edge _eh
	const VertexHandle cvh = mesh->edge(_eh).from_vertex();
	const VertexHandle tvh = mesh->edge(_eh).to_vertex();

	const HalfEdgeHandle he = mesh->halfedge_handle(_eh, 0);

	std::set<CellHandle> toBeProcessed;

	// Get all cells incident to the center vertex
	for(VertexCellIter vc_it = mesh->vc_iter(cvh); vc_it.valid(); ++vc_it)
	{
		toBeProcessed.insert(*vc_it);
	}

	// Exclude cells incident to edge _eh from this set
	// These are the cells that need new geometry
	for(HalfEdgeCellIter hec_it = mesh->hec_iter(he); hec_it.valid(); ++hec_it)
	{
		toBeProcessed.erase(*hec_it);
	}

	// Create a new cell for each of the cells in toBeProcessed
	// The new cells are now incident to the top vertex instead
	// of the one in the center
	std::vector< std::vector<VertexHandle> > newCells;
	for(std::set<CellHandle>::const_iterator c_it = toBeProcessed.begin(), c_end = toBeProcessed.end(); c_it != c_end; ++c_it)
	{

		std::vector<VertexHandle> newVertices;

		const std::vector<HalfFaceHandle>& hfs = mesh->cell(*c_it).halffaces();
		for(HalfFaceVertexIter hfv_it = mesh->hfv_iter(*hfs.begin()); hfv_it.valid(); ++hfv_it)
		{
			VertexHandle nvh = (*hfv_it == cvh ? tvh : *hfv_it);
			nvh.idx((nvh.idx() > cvh.idx() ? nvh.idx() - 1 : nvh.idx()));
			newVertices.push_back(nvh);
		}

		HalfFaceHandle curHF = *hfs.begin();
		HalfEdgeHandle curHE = *mesh->halfface(curHF).halfedges().begin();
		curHF = mesh->adjacent_halfface_in_cell(curHF, curHE);
		curHE = mesh->opposite_halfedge_handle(curHE);
		curHE = mesh->next_halfedge_in_halfface(curHE, curHF);

		VertexHandle nextVH = mesh->halfedge(curHE).to_vertex();

		nextVH = (nextVH == cvh ? tvh : nextVH);
		nextVH.idx((nextVH.idx() > cvh.idx() ? nextVH.idx() - 1 : nextVH.idx()));

		newVertices.push_back(nextVH);

		newCells.push_back(newVertices);
	}

	// Set position of top vertex to be the barycenter
	// of edge _eh
	mesh->set_vertex(tvh, p);

	// Delete center vertex
	// This deletes all incident edges, faces, and cells, too
	mesh->delete_vertex(cvh);

	// Create a new tet for each of the cells in toBeProcessed
	for(std::vector< std::vector<VertexHandle> >::const_iterator nc_it = newCells.begin(),
		nc_end = newCells.end(); nc_it != nc_end; ++nc_it)
	{
		if(!createTet(*nc_it))
		{
			std::cerr << "Error: Could not create tet!" << std::endl;
			return;
		}
	}
}

// Find the mesh's shortest edge
OpenVolumeMesh::EdgeHandle Performance3_OpenVolumeMesh::getShortestEdge()
{
	double max_weight = std::numeric_limits<double>::max();
	EdgeHandle eh = Mesh::InvalidEdgeHandle;
	bool boundary=false;

	for(OpenVolumeMesh::EdgeIter e_it = mesh->edges_begin(),
		e_end = mesh->edges_end(); e_it != e_end; ++e_it)
	{

		const OpenVolumeMesh::VertexHandle& v0 = mesh->edge(*e_it).from_vertex();
		const OpenVolumeMesh::VertexHandle& v1 = mesh->edge(*e_it).to_vertex();

		boundary=false;
		if ((*boundary_vertex)[v0]==0)
		{
			if (mesh->is_boundary(v0))
			{
				boundary=true;
				(*boundary_vertex)[v0]=1;
			}
			else
			{
				(*boundary_vertex)[v0]=2;
			}
		}
		else
		{
			boundary = ((*boundary_vertex)[v0]==1);
		}
		if ((*boundary_vertex)[v1]==0)
		{
			if (mesh->is_boundary(v1))
			{
				boundary=true;
				(*boundary_vertex)[v1]=1;
			}
			else
			{
				(*boundary_vertex)[v1]=2;
			}
		}
		else
		{
			boundary = ((*boundary_vertex)[v1]==1);
		}

		if (boundary) continue;

		double l = mesh->length(*e_it);
		if(l < max_weight)
		{
			max_weight = l;
			eh = *e_it;
		}
	}

	return eh;
}

/*
* Create a new tetrahedron with the specified vertices.
*
*     v2    v3
*       O--O
*      / \ |
*     /   \|
*    O-----O
*   v0     v1
*
*   The vertices have to be in the order depicted above.
*/
OpenVolumeMesh::CellHandle Performance3_OpenVolumeMesh::createTet(const std::vector<OpenVolumeMesh::VertexHandle>& _vs)
{
	std::vector<VertexHandle> tvs;
	tvs.resize(3);

	tvs[0] = _vs[0];
	tvs[1] = _vs[1];
	tvs[2] = _vs[2];

	HalfFaceHandle hf0 = mesh->halfface(tvs);

	if(!hf0.is_valid())
	{
		// Create face
		hf0 = mesh->halfface_handle(mesh->add_face(tvs), 0);
	}

	tvs[0] = _vs[0];
	tvs[1] = _vs[2];
	tvs[2] = _vs[3];

	HalfFaceHandle hf1 = mesh->halfface(tvs);

	if(!hf1.is_valid())
	{
		// Create face
		hf1 = mesh->halfface_handle(mesh->add_face(tvs), 0);
	}

	tvs[0] = _vs[1];
	tvs[1] = _vs[3];
	tvs[2] = _vs[2];

	HalfFaceHandle hf2 = mesh->halfface(tvs);

	if(!hf2.is_valid())
	{
		// Create face
		hf2 = mesh->halfface_handle(mesh->add_face(tvs), 0);
	}

	tvs[0] = _vs[0];
	tvs[1] = _vs[3];
	tvs[2] = _vs[1];

	HalfFaceHandle hf3 = mesh->halfface(tvs);

	if(!hf3.is_valid())
	{
		// Create face
		hf3 = mesh->halfface_handle(mesh->add_face(tvs), 0);
	}

	if(hf0.is_valid() && hf1.is_valid() &&
			hf2.is_valid() && hf3.is_valid())
	{

		std::vector<HalfFaceHandle> hfs;
		hfs.push_back(hf0); hfs.push_back(hf1);
		hfs.push_back(hf2); hfs.push_back(hf3);

		return mesh->add_cell(hfs, true);
	}

	std::cerr << "Could not create cell!" << std::endl;
	return Mesh::InvalidCellHandle;
}

BENCHMARK_F(Performance3_OpenVolumeMesh, circulator)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		int counter = 0;

		//for each vertex enumerate its incident volumes
		for(VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
		{
			for(VertexCellIter vc_it = mesh->vc_iter(*v_it); vc_it.valid(); ++vc_it)
			{
				++counter;
			}
		}

		//for each volumes enumerate its vertices
		for(CellIter c_it = mesh->cells_begin(); c_it != mesh->cells_end(); ++c_it)
		{
			for(CellVertexIter cv_it = mesh->cv_iter(*c_it); cv_it.valid(); ++cv_it)
			{
				--counter;
			}
		}
	}
}

BENCHMARK_F(Performance3_OpenVolumeMesh, circulator2)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		int counter = 0;

		//for each vertex enumerate its vertices adjacent through a common volume.
		for(VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
		{
			for(VertexCellIter vc_it = mesh->vc_iter(*v_it); vc_it.valid(); ++vc_it)
			{
				for(CellVertexIter cv_it = mesh->cv_iter(*vc_it); cv_it.valid(); ++cv_it)
				{
					++counter;
				}
			}
		}
	}
}

BENCHMARK_F(Performance3_OpenVolumeMesh, barycenter)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		Vec3 sum(0.0, 0.0, 0.0);

		//for each volumes
		for(CellIter c_it = mesh->cells_begin(); c_it != mesh->cells_end(); ++c_it)
		{
			Vec3 p = mesh->barycenter(*c_it);
			sum += p;
		}
	}
}

BENCHMARK_F(Performance3_OpenVolumeMesh, smoothing)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		VertexPropertyT<Vec3> position_smoothing = mesh->request_vertex_property<Vec3>("positionSmoothing");

		for(VertexIter v_it = mesh->vertices_begin(), v_end = mesh->vertices_end() ;
			v_it != v_end ;
			++v_it)
		{
			Vec3 p(0.0);
			unsigned int c = 0;

			for(VertexOHalfEdgeIter vh_it = mesh->voh_iter(*v_it); vh_it.valid(); ++vh_it)
			{
				p += mesh->vertex(mesh->halfedge(*vh_it).to_vertex());
				++c;
			}

			p /= double(c);

			position_smoothing[*v_it] = p;
		}

		// Update positions
		for(VertexIter v_it = mesh->vertices_begin(), v_end = mesh->vertices_end(); v_it != v_end; ++v_it)
		{
			mesh->set_vertex(*v_it, position_smoothing[*v_it]);
		}
	}
}

BENCHMARK_F(Performance3_OpenVolumeMesh, split_tet)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		state.ResumeTiming();
		mesh->enable_edge_bottom_up_incidences(false);

		unsigned int n = mesh->n_cells();
		const unsigned int n_c = n;

		for(CellIter c_it = mesh->cells_begin(); n-- > 0; ++c_it)
		{
			/*
	   * Compute geometry of inner cells and add them
	   * to the mesh afterwards
	   *
	   * In the end, all cells up until cell
	   * with index n_c will be deleted in one chunk
	   */

			Vec3 c = mesh->barycenter(*c_it);

			// Center vertex
			VertexHandle vhc = mesh->add_vertex(c);

			// Get first half-face
			HalfFaceHandle curHF = *mesh->cell(*c_it).halffaces().begin();

			HalfFaceHandle ohf0, ohf1, ohf2, ohf3;
			ohf0 = curHF;

			// Get first half-edge
			HalfEdgeHandle curHE = *mesh->halfface(curHF).halfedges().begin();

			VertexHandle vh0 = mesh->halfedge(curHE).from_vertex();

			curHE = mesh->next_halfedge_in_halfface(curHE, curHF);

			VertexHandle vh1 = mesh->halfedge(curHE).from_vertex();

			curHE = mesh->next_halfedge_in_halfface(curHE, curHF);

			VertexHandle vh2 = mesh->halfedge(curHE).from_vertex();

			curHF = mesh->adjacent_halfface_in_cell(curHF, curHE);
			curHE = mesh->opposite_halfedge_handle(curHE);
			curHE = mesh->next_halfedge_in_halfface(curHE, curHF);

			VertexHandle vh3 = mesh->halfedge(curHE).to_vertex();

			ohf3 = curHF;

			curHF = mesh->adjacent_halfface_in_cell(curHF, curHE);
			curHE = mesh->opposite_halfedge_handle(curHE);
			curHE = mesh->next_halfedge_in_halfface(curHE, curHF);
			curHE = mesh->next_halfedge_in_halfface(curHE, curHF);

			ohf1 = curHF;

			curHF = mesh->adjacent_halfface_in_cell(curHF, curHE);

			ohf2 = curHF;

			std::vector<VertexHandle> vs(3);

			vs[0] = vh2; vs[1] = vh1; vs[2] = vhc;
			FaceHandle fh0 = mesh->add_face(vs);

			vs[0] = vh0; vs[1] = vh2; vs[2] = vhc;
			FaceHandle fh1 = mesh->add_face(vs);

			vs[0] = vh1; vs[1] = vh0; vs[2] = vhc;
			FaceHandle fh2 = mesh->add_face(vs);

			vs[0] = vh2; vs[1] = vh3; vs[2] = vhc;
			FaceHandle fh3 = mesh->add_face(vs);

			vs[0] = vh3; vs[1] = vh1; vs[2] = vhc;
			FaceHandle fh4 = mesh->add_face(vs);

			vs[0] = vh3; vs[1] = vh0; vs[2] = vhc;
			FaceHandle fh5 = mesh->add_face(vs);

			std::vector<HalfFaceHandle> hfs(4);

			hfs[0] = mesh->halfface_handle(fh0, 0);
			hfs[1] = mesh->halfface_handle(fh1, 0);
			hfs[2] = mesh->halfface_handle(fh2, 0);
			hfs[3] = ohf0;
			mesh->add_cell(hfs);

			hfs[0] = mesh->halfface_handle(fh3, 0);
			hfs[1] = mesh->halfface_handle(fh4, 0);
			hfs[2] = mesh->halfface_handle(fh0, 1);
			hfs[3] = ohf1;
			mesh->add_cell(hfs);

			hfs[0] = mesh->halfface_handle(fh2, 1);
			hfs[1] = mesh->halfface_handle(fh4, 1);
			hfs[2] = mesh->halfface_handle(fh5, 0);
			hfs[3] = ohf2;
			mesh->add_cell(hfs);

			hfs[0] = mesh->halfface_handle(fh1, 1);
			hfs[1] = mesh->halfface_handle(fh3, 1);
			hfs[2] = mesh->halfface_handle(fh5, 1);
			hfs[3] = ohf3;
			mesh->add_cell(hfs);
		}

		CellIter first = mesh->cells_begin();
		CellIter last = mesh->cells_begin();

		// Delete all former cells in one chunk
		last += n_c;
		mesh->delete_cell_range(first, last);

		mesh->enable_edge_bottom_up_incidences(true);
	}
}

BENCHMARK_F(Performance3_OpenVolumeMesh, collapse)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		state.ResumeTiming();
		OpenVolumeMesh::VertexPropertyT<int> tmp= mesh->request_vertex_property<int>("boundaryVertex");
		boundary_vertex = &tmp;

		for(VertexIter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it)
			(*boundary_vertex)[*v_it] = 0;

		mesh->enable_bottom_up_incidences(true);

		for(unsigned int i = 0; i < collapse_nb_it_; ++i)
		{
			EdgeHandle eh = this->getShortestEdge();

			if(!eh.is_valid())
			{
				std::cerr << "No valid edge anymore, aborting at step "<<i << std::endl;
				return;
			}

			(*boundary_vertex)[mesh->edge(eh).from_vertex()]=0;
			(*boundary_vertex)[mesh->edge(eh).to_vertex()]=0;

			edge_collapse(eh);
		}

		boundary_vertex = NULL;
	}
}

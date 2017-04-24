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

#ifndef BENCHMARK_COMPARISON_OPENVOLUMEMESH_H
#define BENCHMARK_COMPARISON_OPENVOLUMEMESH_H

#define INCLUDE_TEMPLATES

#include <iostream>
#include <vector>
#include <limits>

#include "performance.h"
#include <OpenVolumeMesh/Geometry/VectorT.hh>
#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>
#include <OpenVolumeMesh/Attribs/StatusAttrib.hh>
#include <OpenVolumeMesh/FileManager/FileManager.hh>

class Performance3_OpenVolumeMesh : public Performance3
{

public:
	using Vec3 =  OpenVolumeMesh::Geometry::VectorT<Real,3>;
	using Mesh = OpenVolumeMesh::GeometryKernel<Vec3>;

	using HalfEdgeHandle = OpenVolumeMesh::HalfEdgeHandle;
	using HalfFaceHandle = OpenVolumeMesh::HalfFaceHandle;

	using VertexHandle = OpenVolumeMesh::VertexHandle;
	using EdgeHandle = OpenVolumeMesh::EdgeHandle;
	using CellHandle = OpenVolumeMesh::CellHandle;

	inline Performance3_OpenVolumeMesh() : Performance3() {}

protected:
	bool read_mesh(const std::string& filename) override;
	void clear_mesh() override;
	void edge_collapse(const EdgeHandle& _eh);
	EdgeHandle getShortestEdge();
	CellHandle createTet(const std::vector<OpenVolumeMesh::VertexHandle>& _vs);

	OpenVolumeMesh::VertexPropertyT<int>* boundary_vertex;
	std::unique_ptr<Mesh> mesh;
};

#endif // BENCHMARK_COMPARISON_OPENVOLUMEMESH_H

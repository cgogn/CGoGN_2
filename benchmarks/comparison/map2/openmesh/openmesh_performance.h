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

#ifndef BENCHMARK_COMPARISON_OPENMESH_H
#define BENCHMARK_COMPARISON_OPENMESH_H

#include "performance.h"
#include "OpenMesh/Core/IO/MeshIO.hh"
#include "OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh"
#include <memory>

class Performance2_OpenMesh : public Performance2
{

public:
	struct MyTraits : public OpenMesh::DefaultTraits
	{
		using Vec3 = OpenMesh::VectorT<Real,3>;
		using Point = Vec3;
		using Normal = Vec3;
	};

	using Mesh = OpenMesh::TriMesh_ArrayKernelT<MyTraits>;

	inline Performance2_OpenMesh() : Performance2() {}

protected:
	bool read_mesh(const std::string& filename) override;
	void clear_mesh() override;

	std::unique_ptr<Mesh> mesh;
};

#endif // BENCHMARK_COMPARISON_OPENMESH_H

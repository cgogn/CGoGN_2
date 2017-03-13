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

#ifndef BENCHMARK_COMPARISON_CGOGN1_PERFORMANCE3_H
#define BENCHMARK_COMPARISON_CGOGN1_PERFORMANCE3_H

#include <performance.h>
#include <iostream>

#include "Topology/generic/parameters.h"
#include "Topology/map/embeddedMap3.h"

class Performance3_CGoGN1 : public Performance3
{

public:
	struct PFP
	{
		using MAP = CGoGN::EmbeddedMap3;
		using REAL = Real;
		using VEC3 = CGoGN::Geom::Vector<3,Real> ;
	};

	using Dart = CGoGN::Dart;
	using Map = PFP::MAP;
	using Vertex = CGoGN::Vertex;
	using Edge = CGoGN::Edge;
	using Face = CGoGN::Face;
	using Volume = CGoGN::Vol;
	using Vec3 = PFP::VEC3;

	template<typename T>
	using VertexAttribute = CGoGN::VertexAttribute<T, Map>;

	template<typename T>
	using FaceAttribute = CGoGN::FaceAttribute<T, Map>;

	inline Performance3_CGoGN1() : Performance3() {}

protected:
	bool read_mesh(const std::string& filename) override;
	void clear_mesh() override;

protected:
	std::unique_ptr<Map> map;
	VertexAttribute<Vec3> position;
	VertexAttribute<Vec3> position_smoothing;
	VertexAttribute<int> boundary_vertex;
	Dart getShortestEdge();
};

#endif // BENCHMARK_COMPARISON_CGOGN1_PERFORMANCE3_H

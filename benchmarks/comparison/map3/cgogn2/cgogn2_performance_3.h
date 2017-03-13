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

#ifndef BENCHMARK_COMPARISON_CGOGN2_PERFORMANCE3_H
#define BENCHMARK_COMPARISON_CGOGN2_PERFORMANCE3_H

#include <performance.h>

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/geometry/types/eigen.h>

class Performance3_CGoGN2 : public Performance3
{

public:
	using Dart = cgogn::Dart;
	using Map = cgogn::CMap3;
	using Vertex = Map::Vertex;
	using Edge = Map::Edge;
	using Face = Map::Face;
	using Volume = Map::Volume;
	using Vec3 = Eigen::Matrix<Real,3,1,0,3,1>;

	std::unique_ptr<Map> map;
	Map::VertexAttribute<Vec3> position;
	Map::VertexAttribute<Vec3> position_smoothing;
	Map::VertexAttribute<int> boundary_vertex;
	std::unique_ptr<cgogn::CellCache<Map>> cache;


	inline Performance3_CGoGN2() : Performance3() {}

protected:
	bool read_mesh(const std::string& filename) override;
	void clear_mesh() override;
	Dart getShortestEdge();
};

#endif // BENCHMARK_COMPARISON_CGOGN2_PERFORMANCE3_H

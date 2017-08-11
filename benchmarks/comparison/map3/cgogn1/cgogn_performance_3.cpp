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

#include "cgogn_performance_3.h"
#include "Topology/generic/traversor/traversor3.h"
#include "Topology/generic/traversor/traversorCell.h"
#include "Topology/generic/cellmarker.h"

#include "Geometry/vector_gen.h"

#include "Algo/Modelisation/tetrahedralization.h"

#include "Algo/Import/import.h"
#include "Algo/Export/exportVol.h"

#include "Container/fakeAttribute.h"

using namespace CGoGN;

void Performance3_CGoGN1::clear_mesh()
{
	if (position.isValid())
		map->removeAttribute(position);
	if (position_smoothing.isValid())
		map->removeAttribute(position_smoothing);
	map.reset(new Map());
}

bool Performance3_CGoGN1::read_mesh(const std::string& filename)
{
	std::vector<std::string> attribute_names;
	const bool res = Algo::Volume::Import::importMesh<PFP>(*map, filename, attribute_names);
	position = map->getAttribute<Vec3, VERTEX, Map>(attribute_names[0]);

	map->enableQuickTraversal<Map, VERTEX>();
	map->enableQuickTraversal<Map, VOLUME>();

	position_smoothing = map->addAttribute<Vec3, VERTEX, Map>("positionSmoothing");

	map->enableQuickTraversal<Map, VERTEX>();
	map->enableQuickTraversal<Map, FACE>();

	return res && position.isValid();
}

CGoGN::Dart Performance3_CGoGN1::getShortestEdge()
{
	double weight = std::numeric_limits<double>::max();
	Dart dart = NIL;
	bool boundary=false;

	TraversorE<Map> te(*map);
	for(Dart dit = te.begin() ; dit != te.end() ; dit = te.next())
	{
		Dart dit1 = map->phi1(dit);

		boundary=false;
		if (boundary_vertex[dit]==0)
		{
			if (map->isBoundaryVertex(dit))
			{
				boundary=true;
				boundary_vertex[dit]=1;
			}
			else
			{
				boundary_vertex[dit]=2;
			}
		}
		else
		{
			boundary = (boundary_vertex[dit]==1);
		}
		if (boundary_vertex[dit1]==0)
		{
			if (map->isBoundaryVertex(dit1))
			{
				boundary=true;
				boundary_vertex[dit1]=1;
			}
			else
			{
				boundary_vertex[dit1]=2;
			}
		}
		else
		{
			boundary = (boundary_vertex[dit1]==1);
		}

		if (boundary) continue;

		Vec3 p1 = position[dit];
		Vec3 p0 = position[dit1];
		Vec3 v = (p1 - p0);

		double w = sqrt(v.norm2());
		if(w < weight)
		{
			weight = w;
			dart = dit;
		}
	}

	return dart;
}

BENCHMARK_F(Performance3_CGoGN1, circulator)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		int counter = 0;


		//for each vertex enumerate its incident volumes
		TraversorV<Map> tv(*map);
		for(Dart dit = tv.begin() ; dit != tv.end() ; dit = tv.next())
		{
			Traversor3VW<Map> tvw(*map,dit);
			for(Dart ditvw = tvw.begin() ; ditvw != tvw.end() ; ditvw = tvw.next())
			{
				++counter;
			}
		}

		//for each volumes enumerate its vertices
		TraversorW<Map> tw(*map);
		for(Dart dit = tw.begin() ; dit != tw.end() ; dit = tw.next())
		{
			Traversor3WV<Map> twv(*map,dit);
			for(Dart ditwv = twv.begin() ; ditwv != twv.end() ; ditwv = twv.next())
			{
				--counter;
			}
		}
	}
}

BENCHMARK_F(Performance3_CGoGN1, circulator2)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		int counter = 0;
		//for each vertex enumerate its incident volumes
		TraversorV<Map> tv(*map);
		for(Dart dit = tv.begin() ; dit != tv.end() ; dit = tv.next())
		{
			Traversor3VVaW<Map> twv(*map,dit);
			for(Dart ditwv = twv.begin() ; ditwv != twv.end() ; ditwv = twv.next())
			{
				++counter;
			}
		}
	}
}

BENCHMARK_F(Performance3_CGoGN1, barycenter)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		Vec3 sum(0.0, 0.0, 0.0);

		TraversorW<Map> tw(*map);
		for(Dart dit = tw.begin() ; dit != tw.end() ; dit = tw.next())
		{
			Vec3 p = Algo::Surface::Geometry::volumeCentroid<PFP>(*map,dit,position);
			sum += p;
		}
	}
}

BENCHMARK_F(Performance3_CGoGN1, smoothing)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		//laplacian smoothing
		TraversorV<Map> tv(*map);
		for(Dart dit = tv.begin() ; dit != tv.end() ; dit = tv.next())
		{
			Vec3 p(0.0);
			unsigned int c = 0;

			Traversor3VVaE<Map> trav3VVaE(*map, dit);
			for(Dart dit3VVaF = trav3VVaE.begin() ; dit3VVaF != trav3VVaE.end() ; dit3VVaF = trav3VVaE.next())
			{
				p += position[dit3VVaF];
				++c;
			}
			p /= double(c);

			position_smoothing[dit] = p;
		}

		map->swapAttributes(position, position_smoothing);
	}
}

BENCHMARK_F(Performance3_CGoGN1, split_tet)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);

		map->disableQuickTraversal<VOLUME>();
		map->disableQuickTraversal<VERTEX>();

		state.ResumeTiming();

		TraversorW<Map> tW(*map);
		for(Dart dit = tW.begin() ; dit != tW.end() ; dit = tW.next())
		{
			Vec3 volCenter(0., 0., 0.);
			volCenter += position[dit];
			volCenter += position[map->phi1(dit)];
			volCenter += position[map->phi_1(dit)];
			volCenter += position[map->phi_1(map->phi2(dit))];
			volCenter /= 4;

			Dart dres = Algo::Volume::Modelisation::Tetrahedralization::flip1To4<PFP>(*map, dit);
			position[dres] = volCenter;
		}
	}
}

BENCHMARK_F(Performance3_CGoGN1, collapse)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		boundary_vertex = map->addAttribute<int, VERTEX, Map>("boundaryVertex");
		state.ResumeTiming();

		TraversorE<Map> te(*map);
		for(Dart dit = te.begin() ; dit != te.end() ; dit = te.next())
			boundary_vertex[dit]=0;

		for(unsigned int i = 0; i < this->collapse_nb_it_; ++i)
		{
			Dart dit = getShortestEdge();

			if(dit == NIL)
			{
				std::cerr << "No valid edge anymore, aborting at step "<<i << std::endl;
				return;
			}

			boundary_vertex[dit]=0;
			boundary_vertex[map->phi_1(dit)]=0;

			map->collapseEdge(dit);
		}
	}
}


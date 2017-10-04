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

#include "cgogn1_performance.h"

#include "Topology/generic/traversor/traversor2.h"
#include "Topology/generic/traversor/traversorCell.h"
#include "Topology/generic/cellmarker.h"

#include "Geometry/vector_gen.h"

#include "Algo/Import/import.h"
#include "Algo/Export/export.h"

#include "Algo/Modelisation/subdivision.h"
#include <vector>
#include <string>

using namespace CGoGN;


void Performance2_CGoGN1::clear_mesh()
{
	if (position.isValid())
		map->removeAttribute(position);
	if (normalV.isValid())
		map->removeAttribute(normalV);
	if (normalF.isValid())
		map->removeAttribute(normalF);
	map.reset(new Map());
}

bool Performance2_CGoGN1::read_mesh(const std::string& filename)
{
	std::vector<std::string> attribute_names;
	const bool res = Algo::Surface::Import::importMesh<PFP>(*map, filename, attribute_names);

	position = map->getAttribute<Vec3, VERTEX, Map>(attribute_names[0]);

	normalV = map->addAttribute<Vec3, VERTEX, Map>("normalV");
	normalF = map->addAttribute<Vec3, FACE, Map>("normalF");

	map->enableQuickTraversal<Map, VERTEX>();
	map->enableQuickTraversal<Map, FACE>();

	return res && position.isValid();
}

BENCHMARK_F(Performance2_CGoGN1, circulator)(benchmark::State& state)
{
	int counter = 0;

	while (state.KeepRunning())
	{
		int counter = 0;

		TraversorV<Map> tv(*map);
		for(Dart dit = tv.begin(); dit != tv.end(); dit = tv.next())
		{
			Traversor2VF<Map> trav(*map, dit);
			for(Dart ditF = trav.begin(); ditF != trav.end(); ditF = trav.next())
				++counter;
		}

		TraversorF<Map> tf(*map);
		for(Dart dit = tf.begin(); dit != tf.end(); dit = tf.next())
		{
			Traversor2FV<Map> trav(*map, dit);
			for(Dart ditV = trav.begin(); ditV != trav.end(); ditV = trav.next())
				--counter;
		}
	}
}

BENCHMARK_F(Performance2_CGoGN1, barycenter)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		Vec3 p(0.0,0.0,0.0);

		unsigned int size = position.end();
		unsigned int count = 0;
		for(unsigned int i = position.begin(); i != size; position.next(i))
		{
			p += position[i];
			++count;
		}
		p /= Real(count);

		for(unsigned int i = position.begin(); i != size; position.next(i))
			position[i] -= p;
	}
}

BENCHMARK_F(Performance2_CGoGN1, normal)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		TraversorF<Map> tf(*map);
		for(Dart dit = tf.begin(); dit != tf.end(); dit = tf.next())
		{
			const Vec3& p1 = position[dit];
			const Vec3& p2 = position[map->phi1(dit)];
			const Vec3& p3 = position[map->phi_1(dit)];
			normalF[dit] = ((p2 - p1) ^ (p3 - p1)).normalized();
		}

		TraversorV<Map> tv(*map);
		for(Dart dit = tv.begin(); dit != tv.end(); dit = tv.next())
		{
			Vec3 n(0.0);

			Traversor2VF<Map> trav(*map, dit);
			for(Dart ditF = trav.begin(); ditF != trav.end(); ditF = trav.next())
				n += normalF[ditF];

			normalV[dit] = n.normalized();
		}
	}
}

BENCHMARK_F(Performance2_CGoGN1, smoothing)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		TraversorV<Map> tv(*map);
		for(Dart dit = tv.begin(); dit != tv.end(); dit = tv.next())
		{
			Vec3 p(0.0,0.0,0.0);
			unsigned int c = 0;

			Traversor2VVaE<Map> trav2VVaE(*map, dit);
			for(Dart ditVV = trav2VVaE.begin() ; ditVV != trav2VVaE.end() ; ditVV = trav2VVaE.next())
			{
				p += position[ditVV];
				++c;
			}
			p /= Real(c);
			position[dit] = p;
		}
	}
}

BENCHMARK_F(Performance2_CGoGN1	, subdivision)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		VertexAutoAttribute<Vec3, Map> new_position(*map);
		state.ResumeTiming();

		//compute new positions of old vertices
		TraversorV<Map> tv(*map);
		for(Dart d = tv.begin(); d != tv.end(); d = tv.next())
		{
			unsigned int n = 0;
			Vec3 p(0.0,0.0,0.0);
			Traversor2VVaE<Map> trav2VVaE(*map, d);
			for(Dart ditVV = trav2VVaE.begin() ; ditVV != trav2VVaE.end() ; ditVV = trav2VVaE.next())
			{
				p += position[ditVV];
				++n;
			}
			const Real alpha = (4.0 - 2.0*cos(2.0*M_PI/Real(n))) / 9.0;

			new_position[d] = (1.0f-alpha)*position[d] + alpha/Real(n)*p;
		}

		Dart last = map->end();

		//split faces

		AttributeHandler<Dart, FACE, Map> qtF(map.get(), const_cast<AttributeMultiVector<Dart>*>(map->getQuickTraversal<FACE>()));
		unsigned int nbf = qtF.nbElements();

		for(unsigned int i = qtF.begin(); i != nbf; ++i)
		{
			Dart d = qtF[i];

			unsigned int c = 0;
			Vec3 p(0.0,0.0,0.0);

			//triangule face
			Traversor2FV<Map> trav(*map, d);
			for(Dart ditV = trav.begin(); ditV != trav.end(); ditV = trav.next())
			{
				p += position[ditV];
				++c;
			}

			p /= Real(c);
			Dart cd = Algo::Surface::Modelisation::trianguleFace<PFP>(*map, d);

			new_position[cd] = p;
		}

		map->swapAttributes(position, new_position);

		// flip old edges
		for(Dart d = map->begin(); d != last; map->next(d))
		{
			if(d < map->phi2(d))
				map->flipEdge(d);
		}
	}
}

BENCHMARK_F(Performance2_CGoGN1, collapse)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		map->updateQuickTraversal<Map, FACE>();
		state.ResumeTiming();

		//split faces
		DartMarker<Map> me(*map);

		Vec3 p(0,0,0);

		AttributeHandler<Dart, FACE, Map> qtF(map.get(), const_cast<AttributeMultiVector<Dart>*>(map->getQuickTraversal<FACE>()));
		unsigned int nbf = qtF.end();

		for(unsigned int i = qtF.begin(); i != nbf; ++i)
		{
			Dart d = qtF[i];
			Dart cd = Algo::Surface::Modelisation::trianguleFace<PFP>(*map, d);
			position[cd] = p;
			me.markOrbit<VERTEX>(cd);
		}

		map->disableQuickTraversal<VERTEX>();

		TraversorV<Map> travV(*map);
		for(Dart d = travV.begin() ; d != travV.end() ; d = travV.next())
		{
			if(me.isMarked(d))
				map->deleteVertex(d);
		}
	}
}

BENCHMARK_F(Performance2_CGoGN1, remesh)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		map->updateQuickTraversal<Map, FACE>();
		state.ResumeTiming();

		unsigned int nbOps = 1000;

		TraversorF<Map> travF(*map);
		for(Dart d = travF.begin(); d != travF.end() && nbOps != 0; d = travF.next())
		{
			Dart cd = Algo::Surface::Modelisation::trianguleFace<PFP>(*map, d);
			--nbOps;
		}

		nbOps = 1000;

		TraversorE<Map> travE(*map);
		for(Dart d = travE.begin(); d != travE.end() && nbOps != 0; d = travE.next())
		{
			if(map->edgeCanCollapse(d))
			{
				map->collapseEdge(d);
				--nbOps;
			}
		}
	}
}


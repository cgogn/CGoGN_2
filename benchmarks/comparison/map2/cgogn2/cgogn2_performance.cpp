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

#include "cgogn2_performance.h"
#include <cgogn/core/utils/masks.h>
#include <cgogn/geometry/functions/normal.h>
#include <cgogn/modeling/algos/refinements.h>
#include <cgogn/core/utils/timer.h>
#include <cgogn/io/map_import.h>


void Performance2_CGoGN2::clear_mesh()
{
	cache.reset();
	position = Map::VertexAttribute<Vec3>();
	normalV = Map::VertexAttribute<Vec3>();
	normalF = Map::FaceAttribute<Vec3>();
	map = cgogn::make_unique<Map>();
}

bool Performance2_CGoGN2::read_mesh(const std::string& filename)
{
	cgogn::io::import_surface<Vec3>(*map, filename);
	position = map->get_attribute<Vec3,Vertex>("position");
	if (map->has_attribute(Vertex::ORBIT, "normal"))
		map->remove_attribute(Vertex::ORBIT, "normal");

	normalV = map->add_attribute<Vec3, Vertex>("normalV");
	normalF = map->add_attribute<Vec3, Face>("normalF");

	cache = cgogn::make_unique<cgogn::CellCache<Map>>(*map);
	cache->build<Vertex>();
	cache->build<Edge>();
	cache->build<Face>();
	return position.is_valid();
}

BENCHMARK_F(Performance2_CGoGN2, circulator)(benchmark::State& state)
{
	int counter = 0;

	while (state.KeepRunning())
	{
		map->foreach_cell([&](Vertex v)
		{
			map->foreach_incident_face(v,[&](Face)
			{
				++counter;
			});
		}, *cache);

		map->foreach_cell([&](Face f)
		{
			map->foreach_incident_vertex(f,[&](Vertex)
			{
				--counter;
			});
		}, *cache);
	}
}

BENCHMARK_F(Performance2_CGoGN2, barycenter)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		unsigned count = 0u;
		Vec3 p(0., 0., 0.);
		for (auto& pos : position)
		{
			p += pos;
			++count;
		}

		p /= count;

		for (auto& pos : position)
			pos -=p;
	}
}

BENCHMARK_F(Performance2_CGoGN2, normal)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		map->foreach_cell([&](Face f)
		{
			const Vertex v(f.dart);
			normalF[f] = cgogn::geometry::normal(position[v],
												 position[map->phi1(v.dart)],
					position[map->phi_1(v.dart)]).normalized();
		}, *cache);

		//		map->parallel_foreach_cell([&](Vertex v, cgogn::uint32)
		map->foreach_cell([&](Vertex v)
		{
			Vec3 n(0.,0.,0.);
			map->foreach_incident_face(v,[&](Face f)
			{
				n += normalF[f];
			});

			normalV[v] = n.normalized();
		}, *cache);
	}
}

BENCHMARK_F(Performance2_CGoGN2, smoothing)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		map->foreach_cell([&](Vertex v)
		{
			Vec3 p(0., 0., 0.);
			unsigned int c = 0;
			map->foreach_adjacent_vertex_through_edge(v,[&](Vertex va)
			{
				p += position[va];
				++c;
			});
			p /= c;
			position[v] = p;
		},*cache);
	}
}

BENCHMARK_F(Performance2_CGoGN2, subdivision)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		Map::VertexAttribute<Vec3> new_position = map->add_attribute<Vec3,Vertex>("new_position");
		state.ResumeTiming();

		//compute new positions of old vertices

		map->foreach_cell([&](Vertex v/*, cgogn::uint32*/)
		{
			cgogn::uint32 n(0u);
			Vec3 p (0., 0., 0.);
			map->foreach_adjacent_vertex_through_edge(v, [&](Vertex va)
			{
				p+= position[va];
				++n;
			});

			const Real alpha = (Real(4) - Real(2)*cos(Real(2)*Real(M_PI)/Real(n))) / Real(9);
			new_position[v] = (Real(1)-alpha)*position[v] + alpha/Real(n)*p;
		}/*, *cache*/);

		//split faces
		map->foreach_cell([&](Face f)
		{
			cgogn::uint32 c = 0;
			Vec3 p (0., 0., 0.);
			map->foreach_incident_vertex(f, [&](Vertex v)
			{
				p += position[v];
				++c;
			});
			p/= c;
			const Vertex v = cgogn::modeling::triangule(*map,f);
			new_position[v] = p;
		},*cache);

		map->swap_attributes(position, new_position);

		map->foreach_cell([&](Edge e)
		{
			map->flip_edge(e);
		}, *cache);
	}
}

//BENCHMARK_F(Performance2_CGoGN2, collapse)(benchmark::State& state)
//{
//	while (state.KeepRunning())
//	{
//		state.PauseTiming();
//		this->SetUp(state);
//		state.ResumeTiming();

//		//split faces
//		cgogn::DartMarker<Map> dm(*map);


//		Vec3 p(0., 0., 0.);
//		map->foreach_cell([&](Face f)
//		{
//			const Vertex v = cgogn::modeling::triangule(*map,f);
//			position[v] = p;
//			dm.mark_orbit(v);
//		}, *cache);

//		map->foreach_cell([&](Vertex v)
//		{
//			if (dm.is_marked(v.dart))
//				map->delete_vertex(v); // TODO: add this function to cgogn_2
//		});
//	}
//}

BENCHMARK_F(Performance2_CGoGN2, remesh)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		state.ResumeTiming();

		unsigned int nbOps = 1000;

		map->foreach_cell([&](Face f) -> bool
		{
			cgogn::modeling::triangule(*map,f);
			--nbOps;
			if (nbOps == 0)
				return false;
			else
				return true;
		}, *cache);

		nbOps = 1000;

		map->foreach_cell([&](Edge e) -> bool
		{
			if (map->edge_can_collapse(e))
			{
				map->collapse_edge(e);
				--nbOps;
			}
			if (nbOps == 0)
				return false;
			else
				return true;
		});
	}
}

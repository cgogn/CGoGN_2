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

#include "cgogn2_performance_3.h"
#include <cgogn/core/utils/masks.h>
#include <cgogn/core/utils/timer.h>
#include <cgogn/geometry/functions/normal.h>
#include <cgogn/modeling/algos/refinements.h>
#include <cgogn/modeling/algos/tetrahedralization.h>
#include <cgogn/io/map_import.h>

void Performance3_CGoGN2::clear_mesh()
{
	cache.reset();
	position = Map::VertexAttribute<Vec3>();
	position_smoothing = Map::VertexAttribute<Vec3>();
	map = cgogn::make_unique<Map>();
}

bool Performance3_CGoGN2::read_mesh(const std::string& filename)
{
	cgogn::io::import_volume<Vec3>(*map, filename);

	position = map->get_attribute<Vec3,Vertex>("position");
	position_smoothing = map->add_attribute<Vec3, Vertex>("position_smoothing");
	cache = cgogn::make_unique<cgogn::CellCache<Map>>(*map);
	cache->build<Vertex>();
	cache->build<Edge>();
	cache->build<Face>();
	cache->build<Volume>();
	return position.is_valid();
}

cgogn::Dart Performance3_CGoGN2::getShortestEdge()
{
	double weight = std::numeric_limits<double>::max();
	Dart dart;
	bool boundary=false;


	map->foreach_cell([&](Edge e)
	{
		Dart dit1 = map->phi1(e.dart);

		boundary=false;
		if (boundary_vertex[e.dart]==0)
		{
			if (map->is_incident_to_boundary(Vertex(e.dart)))
			{
				boundary=true;
				boundary_vertex[e.dart]=1;
			}
			else
			{
				boundary_vertex[e.dart]=2;
			}
		}
		else
		{
			boundary = (boundary_vertex[e.dart]==1);
		}
		if (boundary_vertex[dit1]==0)
		{
			if (map->is_incident_to_boundary(Vertex(dit1)))
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

		if (!boundary) return;

		Vec3 p1 = position[e.dart];
		Vec3 p0 = position[dit1];

		const double w = std::sqrt((p1 - p0).squaredNorm());
		if(w < weight)
		{
			weight = w;
			dart = e.dart;
		}
	}, *cache);

	return dart;
}

BENCHMARK_F(Performance3_CGoGN2, circulator)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		int counter = 0;

		map->foreach_cell([&](Vertex v)
		{
			map->foreach_incident_volume(v,[&](Volume)
			{
				++ counter;
			});
		}, *cache);

		map->foreach_cell([&](Volume w)
		{
			map->foreach_incident_vertex(w,[&](Vertex)
			{
				--counter;
			});
		}, *cache);
	}
}

BENCHMARK_F(Performance3_CGoGN2, circulator2)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		int counter = 0;
		map->foreach_cell([&](Vertex v)
		{
			map->foreach_adjacent_vertex_through_volume(v,[&](Vertex)
			{
				++ counter;
			});
		}, *cache);
	}
}

BENCHMARK_F(Performance3_CGoGN2, barycenter)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		Vec3 sum(0.0, 0.0, 0.0);

		map->foreach_cell([&](Volume w)
		{
			sum += cgogn::geometry::centroid<Vec3>(*map, w, position);
		}, *cache);
	}
}

BENCHMARK_F(Performance3_CGoGN2, smoothing)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		map->foreach_cell([&](Vertex v)
		{
			map->foreach_adjacent_vertex_through_volume(v,[&](Vertex)
			{
				Vec3 p(0., 0., 0.);
				cgogn::uint32 c = 0;

				map->foreach_adjacent_vertex_through_edge(v,[&](Vertex avte)
				{
					p += position[avte];
					++c;
				});
				p /= Real(c);
				position_smoothing[v] = p;
			});
		}, *cache);

		map->swap_attributes(position, position_smoothing);
	}
}

BENCHMARK_F(Performance3_CGoGN2, split_tet)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		state.ResumeTiming();

		map->foreach_cell([&](Volume w)
		{
			Vec3 volCenter(0., 0., 0.);
			volCenter += position[w.dart];
			volCenter += position[map->phi1(w.dart)];
			volCenter += position[map->phi_1(w.dart)];
			volCenter += position[map->phi_1(map->phi2(w.dart))];
			volCenter /= Real(4);

			const Vertex v = cgogn::modeling::flip_14(*map,w);
			position[v] = volCenter;
		}, *cache);
	}
}

//BENCHMARK_F(Performance3_CGoGN2, collapse)(benchmark::State& state)
//{
//	while (state.KeepRunning())
//	{
//		state.PauseTiming();
//		this->SetUp(state);
//		boundary_vertex = map->add_attribute<int, Vertex>("boundaryVertex");
//		state.ResumeTiming();

//		map->foreach_cell([&](Edge e)
//		{
//			boundary_vertex[Vertex(e.dart)] = 0;
//		});

//		for(unsigned int i = 0; i < this->collapse_nb_it_; ++i)
//		{
//			const Dart dit = getShortestEdge();

//			if(dit.is_nil())
//				break;

//			boundary_vertex[dit]=0;
//			boundary_vertex[map->phi_1(dit)]=0;

////			map->collapseEdge(Edge(dit)); // TODO : CMap3::collapseEdge
//		}
//	}
//}


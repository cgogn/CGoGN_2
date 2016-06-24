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

#include <chrono>
#include <vector>

#include <cgogn/core/utils/logger.h>
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/cmap/cmap3_tetra.h>
#include <cgogn/io/map_import.h>
#include <cgogn/geometry/algos/centroid.h>
#include <cgogn/geometry/algos/filtering.h>

#include <benchmark/benchmark.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using namespace cgogn::numerics;


using Map3 = cgogn::CMap3<cgogn::DefaultMapTraits>;
Map3 bench_map;

using Vertex = Map3::Vertex;
const cgogn::Orbit VERTEX = Vertex::ORBIT;

using Face = Map3::Face;
const cgogn::Orbit FACE = Face::ORBIT;

using Volume = Map3::Volume;
const cgogn::Orbit VOLUME = Volume::ORBIT;

template <typename T>
using VertexAttribute = Map3::VertexAttribute<T>;

template <typename T>
using VolumeAttribute = Map3::VolumeAttribute<T>;



using TMap3 = cgogn::CMap3Tetra<cgogn::DefaultMapTraits>;
TMap3 bench_tetra_map;

using TVertex = TMap3::Vertex;
const cgogn::Orbit TVERTEX = TVertex::ORBIT;

using TFace = TMap3::Face;
const cgogn::Orbit TFACE = TFace::ORBIT;

using TVolume = TMap3::Volume;
const cgogn::Orbit TVOLUME = TVolume::ORBIT;

template <typename T>
using TVertexAttribute = TMap3::VertexAttribute<T>;

template <typename T>
using TVolumeAttribute = TMap3::VolumeAttribute<T>;


const uint32 ITERATIONS = 1u;

//using Vec3 = Eigen::Vector3d;
using Vec3 = cgogn::geometry::Vec_T<std::array<float64,3>>;





static void BENCH_mark_cc_poly(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		cgogn::Dart d;
		bench_map.foreach_cell_until([&] (Volume v)
		{
			d = v.dart;
			return false;
		});

		cgogn::DartMarker<Map3> ccm(bench_map);
		ccm.mark_orbit(Map3::ConnectedComponent(d));
	}
}



static void BENCH_mark_cc_tetra(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		cgogn::Dart d;
		bench_tetra_map.foreach_cell_until([&] (Volume v)
		{
			d = v.dart;
			return false;
		});

		cgogn::DartMarker<TMap3> ccm(bench_tetra_map);
		ccm.mark_orbit(TMap3::ConnectedComponent(d));
	}
}



static void BENCH_vol_centroid_poly(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		VolumeAttribute<Vec3> centroids = bench_map.get_attribute<Vec3, VOLUME>("centroids");
		cgogn_assert(centroids.is_valid());
		state.ResumeTiming();

		bench_map.foreach_cell([&] (Volume v)
		{
			centroids[v] = cgogn::geometry::centroid<Vec3>(bench_map, v, vertex_position);
		});
	}
}



static void BENCH_vol_centroid_tetra(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		TVertexAttribute<Vec3> vertex_position = bench_tetra_map.get_attribute<Vec3, TVERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		TVolumeAttribute<Vec3> centroids = bench_tetra_map.get_attribute<Vec3, TVOLUME>("centroids");
		cgogn_assert(centroids.is_valid());
		state.ResumeTiming();

		bench_tetra_map.foreach_cell([&] (Volume v)
		{
			centroids[v] = cgogn::geometry::centroid<Vec3>(bench_tetra_map, v, vertex_position);
		});
	}
}

static void BENCH_vol_centroid_poly_pure_topo(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		VolumeAttribute<Vec3> centroids = bench_map.get_attribute<Vec3, VOLUME>("centroids");
		cgogn_assert(centroids.is_valid());
		state.ResumeTiming();

		bench_map.foreach_cell<cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Volume v)
		{
			centroids[v] = cgogn::geometry::centroid<Vec3>(bench_map, v, vertex_position);
		});
	}
}



static void BENCH_vol_centroid_tetra_pure_topo(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		TVertexAttribute<Vec3> vertex_position = bench_tetra_map.get_attribute<Vec3, TVERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		TVolumeAttribute<Vec3> centroids = bench_tetra_map.get_attribute<Vec3, TVOLUME>("centroids");
		cgogn_assert(centroids.is_valid());
		state.ResumeTiming();

		bench_tetra_map.foreach_cell<cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Volume v)
		{
			centroids[v] = cgogn::geometry::centroid<Vec3>(bench_tetra_map, v, vertex_position);
		});
	}
}



static void BENCH_vol_centroid_shrink_poly(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		VertexAttribute<Vec3> vertex_position2 = bench_map.get_attribute<Vec3, VERTEX>("position2");
		cgogn_assert(vertex_position2.is_valid());

		VolumeAttribute<Vec3> centroids = bench_map.get_attribute<Vec3, VOLUME>("centroids");
		cgogn_assert(centroids.is_valid());

		state.ResumeTiming();

		bench_map.foreach_cell<cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Volume v)
		{
			Vec3 c = cgogn::geometry::centroid<Vec3>(bench_map, v, vertex_position);

			bench_map.foreach_incident_vertex(v, [&] ( Vertex x)
			{
				vertex_position2[x] = vertex_position[x]*0.8 + c*0.2;
			});
		});
	}
}



static void BENCH_vol_centroid_shrink_tetra(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		TVertexAttribute<Vec3> vertex_position = bench_tetra_map.get_attribute<Vec3, TVERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		TVertexAttribute<Vec3> vertex_position2 = bench_tetra_map.get_attribute<Vec3, TVERTEX>("position2");
		cgogn_assert(vertex_position2.is_valid());
		TVolumeAttribute<Vec3> centroids = bench_tetra_map.get_attribute<Vec3, TVOLUME>("centroids");
		cgogn_assert(centroids.is_valid());
		state.ResumeTiming();

		bench_tetra_map.foreach_cell<cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Volume v)
		{
			Vec3 c = cgogn::geometry::centroid<Vec3>(bench_tetra_map, v, vertex_position);

			bench_tetra_map.foreach_incident_vertex(v, [&] ( Vertex x)
			{
				vertex_position2[x] = vertex_position[x]*0.8 + c*0.2;
			});
		});
	}
}




static void BENCH_vertices_filter_poly(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		VertexAttribute<Vec3> vertex_position2 = bench_map.get_attribute<Vec3, VERTEX>("position2");
		cgogn_assert(vertex_position2.is_valid());

		state.ResumeTiming();

		cgogn::geometry::filter_average<Vec3>(bench_map, vertex_position, vertex_position2);
	}
}


static void BENCH_vertices_filter_tetra(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		TVertexAttribute<Vec3> vertex_position = bench_tetra_map.get_attribute<Vec3, TVERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		TVertexAttribute<Vec3> vertex_position2 = bench_tetra_map.get_attribute<Vec3, TVERTEX>("position2");
		cgogn_assert(vertex_position2.is_valid());

		state.ResumeTiming();

		cgogn::geometry::filter_average<Vec3>(bench_tetra_map, vertex_position, vertex_position2);
	}
}


BENCHMARK(BENCH_mark_cc_poly);
BENCHMARK(BENCH_mark_cc_tetra);

BENCHMARK(BENCH_vol_centroid_poly);
BENCHMARK(BENCH_vol_centroid_tetra);

BENCHMARK(BENCH_vol_centroid_poly_pure_topo);
BENCHMARK(BENCH_vol_centroid_tetra_pure_topo);

BENCHMARK(BENCH_vol_centroid_shrink_poly);
BENCHMARK(BENCH_vol_centroid_shrink_tetra);

BENCHMARK(BENCH_vertices_filter_poly)->UseRealTime();
BENCHMARK(BENCH_vertices_filter_tetra)->UseRealTime();


int main(int argc, char** argv)
{
	::benchmark::Initialize(&argc, argv);
	std::string volumeMesh;

	if (argc < 2)
	{
		cgogn_log_info("bench_tetra_map") << "USAGE: " << argv[0] << " [filename]";
		volumeMesh = std::string(DEFAULT_MESH_PATH) + std::string("tet/volumic_hand.tet");
		cgogn_log_info("bench_multithreading") << "Using default mesh : \"" << volumeMesh << "\".";
	}
	else
		volumeMesh = std::string(argv[1]);

	cgogn::io::import_volume<Vec3>(bench_map, volumeMesh);
	bench_map.add_attribute<Vec3, VOLUME>("centroids");
	bench_map.add_attribute<Vec3, VERTEX>("normal");
	bench_map.add_attribute<Vec3, VERTEX>("position2");

	cgogn::io::import_volume<Vec3>(bench_tetra_map, volumeMesh);
	bench_tetra_map.add_attribute<Vec3, TVOLUME>("centroids");
	bench_tetra_map.add_attribute<Vec3, TVERTEX>("normal");
	bench_tetra_map.add_attribute<Vec3, TVERTEX>("position2");


	::benchmark::RunSpecifiedBenchmarks();
	return 0;
}

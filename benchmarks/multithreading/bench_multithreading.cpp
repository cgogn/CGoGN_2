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
#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/io/map_import.h>
#include <cgogn/geometry/algos/normal.h>

#include <benchmark/benchmark.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using namespace cgogn::numerics;


using Map2 = cgogn::CMap2;
Map2 bench_map;

using Vertex = Map2::Vertex;
const cgogn::Orbit VERTEX = Vertex::ORBIT;

using Face = Map2::Face;
const cgogn::Orbit FACE = Face::ORBIT;

//const uint32 ITERATIONS = 1u;

//using Vec3 = Eigen::Vector3d;
using Vec3 = cgogn::geometry::Vec_T<std::array<float64,3>>;

template <typename T>
using VertexAttribute = Map2::VertexAttribute<T>;
template <typename T>
using FaceAttribute = Map2::FaceAttribute<T>;

static void BENCH_enqueue(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		cgogn::ThreadPool* tp = cgogn::thread_pool();
		state.ResumeTiming();
		tp->enqueue([](){;});
	}
}

static void BENCH_Dart_count_single_threaded(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		unsigned nb_darts = 0u;
		bench_map.foreach_dart([&nb_darts] (cgogn::Dart) { nb_darts++; });
	}
}

static void BENCH_Dart_count_multi_threaded(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		uint32 nb_darts_2 = 0u;
		std::vector<uint32> nb_darts_per_thread(cgogn::thread_pool()->nb_workers()/* + 2 ??? */);
		for (auto& n : nb_darts_per_thread)
			n = 0u;
		nb_darts_2 = 0u;
		bench_map.parallel_foreach_dart([&nb_darts_per_thread] (cgogn::Dart)
		{
			nb_darts_per_thread[cgogn::current_thread_index()]++;
		});
		for (uint32 n : nb_darts_per_thread)
			nb_darts_2 += n;

		cgogn_assert(nb_darts_2 == bench_map.nb_darts());
	}
}

template <cgogn::TraversalStrategy STRATEGY>
static void BENCH_faces_normals_single_threaded(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		FaceAttribute<Vec3> face_normal = bench_map.get_attribute<Vec3, FACE>("normal");
		cgogn_assert(face_normal.is_valid());
		state.ResumeTiming();

		bench_map.template foreach_cell<STRATEGY>([&] (Face f)
		{
			face_normal[f] = cgogn::geometry::normal<Vec3>(bench_map, f, vertex_position);
		});
	}
}

static void BENCH_faces_normals_cache_single_threaded(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		FaceAttribute<Vec3> face_normal = bench_map.get_attribute<Vec3, FACE>("normal");
		cgogn_assert(face_normal.is_valid());

		Map2::CellCache cache(bench_map);
		cache.template build<Face>();
		state.ResumeTiming();

		bench_map.foreach_cell([&] (Face f)
		{
			face_normal[f] = cgogn::geometry::normal<Vec3>(bench_map, f, vertex_position);
		},
		cache);
	}
}

template <cgogn::TraversalStrategy STRATEGY>
static void BENCH_faces_normals_multi_threaded(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		FaceAttribute<Vec3> face_normal_mt = bench_map.get_attribute<Vec3, FACE>("normal_mt");
		cgogn_assert(face_normal_mt.is_valid());
		state.ResumeTiming();

		bench_map.template parallel_foreach_cell<STRATEGY>([&] (Face f)
		{
			face_normal_mt[f] = cgogn::geometry::normal<Vec3>(bench_map, f, vertex_position);
		});

		{
			state.PauseTiming();

			FaceAttribute<Vec3> face_normal = bench_map.get_attribute<Vec3, FACE>("normal");
			bench_map.template foreach_cell<cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Face f)
			{
				Vec3 error = face_normal[f] - face_normal_mt[f];
				if (!cgogn::almost_equal_absolute(error.squaredNorm(), 0., 1e-9 ))
				{
					cgogn_log_warning("bench_multithreading") << "There was an error during computation of normals.";
//					std::cerr << "face_normal " << face_normal[f] << std::endl;
//					std::cerr << "face_normal_mt " << face_normal_mt[f] << std::endl;
				}

			});
			state.ResumeTiming();
		}
	}
}

static void BENCH_faces_normals_cache_multi_threaded(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		FaceAttribute<Vec3> face_normal = bench_map.get_attribute<Vec3, FACE>("normal");
		cgogn_assert(face_normal.is_valid());

		Map2::CellCache cache(bench_map);
		cache.template build<Face>();
		state.ResumeTiming();

		bench_map.parallel_foreach_cell([&] (Face f)
		{
			face_normal[f] = cgogn::geometry::normal<Vec3>(bench_map, f, vertex_position);
		},
		cache);
	}
}


template <cgogn::TraversalStrategy STRATEGY>
static void BENCH_vertices_normals_single_threaded(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		VertexAttribute<Vec3> vertices_normal = bench_map.get_attribute<Vec3, VERTEX>("normal");
		cgogn_assert(vertices_normal.is_valid());
		state.ResumeTiming();

		bench_map.template foreach_cell<STRATEGY>([&] (Vertex v)
		{
			vertices_normal[v] = cgogn::geometry::normal<Vec3>(bench_map, v, vertex_position);
		});
	}
}

static void BENCH_vertices_normals_cache_single_threaded(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		VertexAttribute<Vec3> vertices_normal = bench_map.get_attribute<Vec3, VERTEX>("normal");
		cgogn_assert(vertices_normal.is_valid());

		Map2::CellCache cache(bench_map);
		cache.template build<Vertex>();
		state.ResumeTiming();

		bench_map.foreach_cell([&] (Vertex v)
		{
			vertices_normal[v] = cgogn::geometry::normal<Vec3>(bench_map, v, vertex_position);
		},
		cache);
	}
}

template <cgogn::TraversalStrategy STRATEGY>
static void BENCH_vertices_normals_multi_threaded(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		VertexAttribute<Vec3> vertices_normal_mt = bench_map.get_attribute<Vec3, VERTEX>("normal_mt");
		cgogn_assert(vertices_normal_mt.is_valid());
		state.ResumeTiming();

		bench_map.template parallel_foreach_cell<STRATEGY>([&] (Vertex v)
		{
			vertices_normal_mt[v] = cgogn::geometry::normal<Vec3>(bench_map, v, vertex_position);
		});

		{
			state.PauseTiming();

			VertexAttribute<Vec3> vertices_normal = bench_map.get_attribute<Vec3, VERTEX>("normal");
			bench_map.template foreach_cell<cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Vertex v)
			{
				Vec3 error = vertices_normal[v] - vertices_normal_mt[v];
				if (!cgogn::almost_equal_absolute(error.squaredNorm(), 0., 1e-9 ))
				{
					cgogn_log_warning("bench_multithreading") << "There was an error during computation of vertices normals.";
//					std::cerr << "vertices_normal " << vertices_normal[v] << std::endl;
//					std::cerr << "vertices_normal_mt " << vertices_normal_mt[v] << std::endl;
				}

			});
			state.ResumeTiming();
		}
	}
}

static void BENCH_vertices_normals_cache_multi_threaded(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		VertexAttribute<Vec3> vertices_normal_mt = bench_map.get_attribute<Vec3, VERTEX>("normal_mt");
		cgogn_assert(vertices_normal_mt.is_valid());

		Map2::CellCache cache(bench_map);
		cache.template build<Vertex>();
		state.ResumeTiming();

		bench_map.parallel_foreach_cell([&] (Vertex v)
		{
			vertices_normal_mt[v] = cgogn::geometry::normal<Vec3>(bench_map, v, vertex_position);
		},
		cache);

		{
			state.PauseTiming();

			VertexAttribute<Vec3> vertices_normal = bench_map.get_attribute<Vec3, VERTEX>("normal");
			bench_map.foreach_cell([&] (Vertex v)
			{
				Vec3 error = vertices_normal[v] - vertices_normal_mt[v];
				if (!cgogn::almost_equal_absolute(error.squaredNorm(), 0., 1e-9 ))
				{
					cgogn_log_warning("bench_multithreading") << "There was an error during computation of vertices normals.";
//					std::cerr << "vertices_normal " << vertices_normal[v] << std::endl;
//					std::cerr << "vertices_normal_mt " << vertices_normal_mt[v] << std::endl;
				}

			}, cache);
			state.ResumeTiming();
		}
	}
}

BENCHMARK(BENCH_enqueue)->UseRealTime();

BENCHMARK(BENCH_Dart_count_single_threaded);
BENCHMARK(BENCH_Dart_count_multi_threaded)->UseRealTime();

BENCHMARK_TEMPLATE(BENCH_faces_normals_single_threaded, cgogn::TraversalStrategy::FORCE_DART_MARKING)->UseRealTime();
BENCHMARK_TEMPLATE(BENCH_faces_normals_multi_threaded, cgogn::TraversalStrategy::FORCE_DART_MARKING)->UseRealTime();
BENCHMARK_TEMPLATE(BENCH_faces_normals_single_threaded, cgogn::TraversalStrategy::FORCE_CELL_MARKING)->UseRealTime();
BENCHMARK_TEMPLATE(BENCH_faces_normals_multi_threaded, cgogn::TraversalStrategy::FORCE_CELL_MARKING)->UseRealTime();
BENCHMARK(BENCH_faces_normals_cache_single_threaded)->UseRealTime();
BENCHMARK(BENCH_faces_normals_cache_multi_threaded)->UseRealTime();

BENCHMARK_TEMPLATE(BENCH_vertices_normals_single_threaded, cgogn::TraversalStrategy::FORCE_DART_MARKING)->UseRealTime();
BENCHMARK_TEMPLATE(BENCH_vertices_normals_multi_threaded, cgogn::TraversalStrategy::FORCE_DART_MARKING)->UseRealTime();
BENCHMARK_TEMPLATE(BENCH_vertices_normals_single_threaded, cgogn::TraversalStrategy::FORCE_CELL_MARKING)->UseRealTime();
BENCHMARK_TEMPLATE(BENCH_vertices_normals_multi_threaded, cgogn::TraversalStrategy::FORCE_CELL_MARKING)->UseRealTime();
BENCHMARK(BENCH_vertices_normals_cache_single_threaded)->UseRealTime();
BENCHMARK(BENCH_vertices_normals_cache_multi_threaded)->UseRealTime();


int main(int argc, char** argv)
{
	::benchmark::Initialize(&argc, argv);
	std::string surfaceMesh;

	if (argc < 2)
	{
		cgogn_log_info("bench_multithreading") << "USAGE: " << argv[0] << " [filename]";
		surfaceMesh = std::string(DEFAULT_MESH_PATH) + std::string("off/aneurysm_3D.off");
		cgogn_log_info("bench_multithreading") << "Using default mesh : \"" << surfaceMesh << "\".";
	}
	else
		surfaceMesh = std::string(argv[1]);

	cgogn::io::import_surface<Vec3>(bench_map, surfaceMesh);

	bench_map.add_attribute<Vec3, FACE>("normal");
	bench_map.add_attribute<Vec3, FACE>("normal_mt");
	bench_map.add_attribute<Vec3, VERTEX>("normal");
	bench_map.add_attribute<Vec3, VERTEX>("normal_mt");

	::benchmark::RunSpecifiedBenchmarks();
	return 0;
}

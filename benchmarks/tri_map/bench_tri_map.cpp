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
#include <cgogn/core/cmap/cmap2_tri.h>
#include <cgogn/io/map_import.h>
#include <cgogn/geometry/algos/normal.h>
#include <cgogn/geometry/algos/filtering.h>

#include <benchmark/benchmark.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using namespace cgogn::numerics;


using Map2 = cgogn::CMap2;
Map2 bench_map;

using Vertex = Map2::Vertex;
const cgogn::Orbit VERTEX = Vertex::ORBIT;

using Face = Map2::Face;
const cgogn::Orbit FACE = Face::ORBIT;

template <typename T>
using VertexAttribute = Map2::VertexAttribute<T>;

template <typename T>
using FaceAttribute = Map2::FaceAttribute<T>;


using TMap2 = cgogn::CMap2Tri;
TMap2 bench_tri_map;

using TVertex = TMap2::Vertex;
const cgogn::Orbit TVERTEX = TVertex::ORBIT;

using TFace = TMap2::Face;
const cgogn::Orbit TFACE = TFace::ORBIT;

template <typename T>
using TVertexAttribute = TMap2::VertexAttribute<T>;

template <typename T>
using TFaceAttribute = TMap2::FaceAttribute<T>;


//const uint32 ITERATIONS = 1u;

//using Vec3 = Eigen::Vector3d;
using Vec3 = cgogn::geometry::Vec_T<std::array<float64,3>>;


static void BENCH_faces_normals_poly(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		FaceAttribute<Vec3> face_normal = bench_map.get_attribute<Vec3, FACE>("normal");
		cgogn_assert(face_normal.is_valid());
		state.ResumeTiming();

		bench_map.foreach_cell<cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Face f)
		{
			face_normal[f] = cgogn::geometry::normal<Vec3>(bench_map, f, vertex_position);
		});
	}
}

static void BENCH_vertices_normals_poly(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		VertexAttribute<Vec3> vertex_position = bench_map.get_attribute<Vec3, VERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		VertexAttribute<Vec3> vertices_normal = bench_map.get_attribute<Vec3, VERTEX>("normal");
		cgogn_assert(vertices_normal.is_valid());
		state.ResumeTiming();

		bench_map.foreach_cell<cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Vertex v)
		{
			vertices_normal[v] = cgogn::geometry::normal<Vec3>(bench_map, v, vertex_position);
		});
	}
}

static void BENCH_faces_normals_tri(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		TVertexAttribute<Vec3> vertex_position = bench_tri_map.get_attribute<Vec3, TVERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		TFaceAttribute<Vec3> face_normal = bench_tri_map.get_attribute<Vec3, TFACE>("normal");
		cgogn_assert(face_normal.is_valid());
		state.ResumeTiming();

		bench_tri_map.foreach_cell<cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (TFace f)
		{
			face_normal[f] = cgogn::geometry::normal<Vec3>(bench_tri_map, f, vertex_position);
		});
	}
}

static void BENCH_vertices_normals_tri(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		TVertexAttribute<Vec3> vertex_position = bench_tri_map.get_attribute<Vec3, TVERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		TVertexAttribute<Vec3> vertices_normal = bench_tri_map.get_attribute<Vec3, TVERTEX>("normal");
		cgogn_assert(vertices_normal.is_valid());
		state.ResumeTiming();

		bench_tri_map.foreach_cell<cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (TVertex v)
		{
			vertices_normal[v] = cgogn::geometry::normal<Vec3>(bench_tri_map, v, vertex_position);
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

		cgogn::geometry::filter_taubin<Vec3>(bench_map, vertex_position, vertex_position2);
	}
}

static void BENCH_vertices_filter_tri(benchmark::State& state)
{
	while(state.KeepRunning())
	{
		state.PauseTiming();
		TVertexAttribute<Vec3> vertex_position = bench_tri_map.get_attribute<Vec3, TVERTEX>("position");
		cgogn_assert(vertex_position.is_valid());
		TVertexAttribute<Vec3> vertex_position2 = bench_tri_map.get_attribute<Vec3, TVERTEX>("position2");
		cgogn_assert(vertex_position2.is_valid());

		state.ResumeTiming();

		cgogn::geometry::filter_taubin<Vec3>(bench_tri_map, vertex_position, vertex_position2);
	}
}

BENCHMARK(BENCH_faces_normals_poly);
BENCHMARK(BENCH_faces_normals_tri);
BENCHMARK(BENCH_vertices_normals_poly);
BENCHMARK(BENCH_vertices_normals_tri);
BENCHMARK(BENCH_vertices_filter_poly)->UseRealTime();
BENCHMARK(BENCH_vertices_filter_tri)->UseRealTime();

int main(int argc, char** argv)
{
	::benchmark::Initialize(&argc, argv);
	std::string surfaceMesh;

	if (argc < 2)
	{
		cgogn_log_info("bench_tri_map") << "USAGE: " << argv[0] << " [filename]";
		surfaceMesh = std::string(DEFAULT_MESH_PATH) + std::string("off/aneurysm_3D.off");
		cgogn_log_info("bench_multithreading") << "Using default mesh : \"" << surfaceMesh << "\".";
	}
	else
		surfaceMesh = std::string(argv[1]);

	cgogn::io::import_surface<Vec3>(bench_map, surfaceMesh);
	bench_map.add_attribute<Vec3, FACE>("normal");
	bench_map.add_attribute<Vec3, VERTEX>("normal");
	bench_map.add_attribute<Vec3, VERTEX>("position2");

	cgogn::io::import_surface<Vec3>(bench_tri_map, surfaceMesh);
	bench_tri_map.add_attribute<Vec3, FACE>("normal");
	bench_tri_map.add_attribute<Vec3, VERTEX>("normal");
	bench_tri_map.add_attribute<Vec3, VERTEX>("position2");

	::benchmark::RunSpecifiedBenchmarks();
	return 0;
}

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

#include <core/cmap/cmap2.h>
#include <core/cmap/sanity_check.h>
#include <io/map_import.h>
#include <geometry/algos/normal.h>


#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)



using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;

const cgogn::Orbit VERTEX = Map2::VERTEX;
using Vertex = cgogn::Cell<VERTEX>;

const cgogn::Orbit FACE = Map2::FACE;
using Face = cgogn::Cell<FACE>;

const unsigned int ITERATIONS = 1u;

using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

template <typename T>
using VertexAttributeHandler = Map2::VertexAttributeHandler<T>;
template <typename T>
using FaceAttributeHandler = Map2::FaceAttributeHandler<T>;

int main(int argc, char** argv)
{
	std::string surfaceMesh;
	if (argc < 2)
	{
		std::cout << "USAGE: " << argv[0] << " [filename]" << std::endl;
		surfaceMesh = std::string(DEFAULT_MESH_PATH) + std::string("aneurysm3D_1.off");
		std::cout << "Using default mesh : " << surfaceMesh << std::endl;
	}
	else
		surfaceMesh = std::string(argv[1]);

	Map2 map;
	cgogn::io::import_surface<Vec3>(map, surfaceMesh);

	{
		// COUNTING DARTS SINGLE THREAD
		unsigned int nb_darts = 0u;
		{
			std::chrono::time_point<std::chrono::system_clock> start, end;
			start = std::chrono::system_clock::now();
			for (unsigned int i = 0u; i < ITERATIONS; ++i)
			{
				nb_darts = 0u;
				map.foreach_dart([&nb_darts] (cgogn::Dart) { nb_darts++; });
			}
			end =  std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = end - start;
			std::cout << std::fixed << "SINGLE THREAD  "<< elapsed_seconds.count() << "s  || nb darts :" << nb_darts  << std::endl;
		}
		// END COUNTING DARTS SINGLE THREAD

		// COUNTING DARTS MULTI-THREADS
		unsigned int nb_darts_2 = 0u;
		std::vector<unsigned int> nb_darts_per_thread(cgogn::get_thread_pool()->get_nb_threads());
		{
			std::chrono::time_point<std::chrono::system_clock> start, end;
			start = std::chrono::system_clock::now();

			for (unsigned int i = 0u; i < ITERATIONS; ++i)
			{
				for (auto& n : nb_darts_per_thread)
					n = 0u;
				nb_darts_2 = 0u;
				//				clock_gettime(CLOCK_REALTIME,&tbegin);
				map.parallel_foreach_dart([&nb_darts_per_thread] (cgogn::Dart, unsigned int thread_index)
				{
					cgogn_assert(thread_index< 7);
					nb_darts_per_thread[thread_index]++;
				});
				//				clock_gettime(CLOCK_REALTIME,&tend);
				//				std::cout << __FILE__ << ":" << __LINE__ << " : "  << (1000000000u*(tend.tv_sec - tbegin.tv_sec) +tend.tv_nsec - tbegin.tv_nsec)/1000u << " microseconds."<<std::endl;
				for (unsigned int n : nb_darts_per_thread)
					nb_darts_2 += n;
				if (nb_darts_2 != nb_darts)
					std::cerr << "There was an error during the dart counting" << std::endl;
			}

			end =  std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = end - start;
			std::cout << std::fixed << cgogn::NB_THREADS << "      THREADS "<< elapsed_seconds.count() << "s  || nb darts :" << nb_darts_2  << std::endl;
		}
		// END COUNTING DARTS MULTI-THREADS

		VertexAttributeHandler<Vec3> vertex_position = map.get_attribute<Vec3, Map2::VERTEX>("position");
		FaceAttributeHandler<Vec3> face_normal = map.add_attribute<Vec3, Map2::FACE>("normal");
		FaceAttributeHandler<Vec3> face_normal_mt = map.add_attribute<Vec3, Map2::FACE>("normal_mt");



		// DART MARKING
		{
			std::chrono::time_point<std::chrono::system_clock> start, end;
			start = std::chrono::system_clock::now();

			for (unsigned int i = 0u ; i < ITERATIONS; ++i)
			{
				map.template foreach_cell<FACE, cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Face f)
				{
					face_normal[f] = cgogn::geometry::face_normal<Vec3>(map, f, vertex_position);
				});
			}
			end =  std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = end - start;
			std::cout << std::fixed << "SINGLE THREAD  "<< elapsed_seconds.count() << "s  || compute_normal_faces dart marking"  << std::endl;
		}

		{
			std::chrono::time_point<std::chrono::system_clock> start, end;
			start = std::chrono::system_clock::now();
			for (unsigned int i = 0u ; i < ITERATIONS; ++i)
			{
				map.template parallel_foreach_cell<FACE, cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Face f, unsigned int)
				{
					face_normal_mt[f] = cgogn::geometry::face_normal<Vec3>(map, f, vertex_position);
				});
			}


			end =  std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = end - start;
			std::cout << std::fixed << cgogn::NB_THREADS << "      THREADS "<< elapsed_seconds.count() << "s  || compute_normal_faces dart marking" << std::endl;
		}

			// END DART MARKING

		{
			// CHECKING NORMALS
			map.template foreach_cell<FACE, cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Face f)
			{
				Vec3 error = face_normal[f] - face_normal_mt[f];
				if (!cgogn::almost_equal_absolute(error.squaredNorm(), 0., 1e-9 ))
				{
					std::cerr << __FILE__ << ":" << __LINE__ << " : there was an error during computation of normals" << std::endl;
					std::abort;
				}

			});
		}

			// CELL MARKING

		{
			std::chrono::time_point<std::chrono::system_clock> start, end;
			start = std::chrono::system_clock::now();

			for (unsigned int i = 0u ; i < ITERATIONS; ++i)
			{
				map.template foreach_cell<FACE, cgogn::TraversalStrategy::FORCE_CELL_MARKING>([&] (Face f)
				{
					face_normal[f] = cgogn::geometry::face_normal<Vec3>(map, f, vertex_position);
				});
			}
			end =  std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = end - start;
			std::cout << std::fixed << "SINGLE THREAD  "<< elapsed_seconds.count() << "s  || compute_normal_faces cell marking"  << std::endl;
		}

		{
			std::chrono::time_point<std::chrono::system_clock> start, end;
			start = std::chrono::system_clock::now();
			for (unsigned int i = 0u ; i < ITERATIONS; ++i)
			{
				map.template parallel_foreach_cell<FACE, cgogn::TraversalStrategy::FORCE_CELL_MARKING>([&] (Face f, unsigned int)
				{
					face_normal_mt[f] = cgogn::geometry::face_normal<Vec3>(map, f, vertex_position);
				});
			}


			end =  std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = end - start;
			std::cout << std::fixed << cgogn::NB_THREADS << "      THREADS "<< elapsed_seconds.count() << "s  || compute_normal_faces cell marking" << std::endl;
		}

		// END CELL MARKING


		{
			// CHECKING NORMALS
			map.template foreach_cell<FACE, cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Face f)
			{
				Vec3 error = face_normal[f] - face_normal_mt[f];
				if (!cgogn::almost_equal_absolute(error.squaredNorm(), 0., 1e-9 ))
				{
					std::cerr << __FILE__ << ":" << __LINE__ << " : there was an error during computation of normals" << std::endl;
					std::abort;
				}

			});
		}

		map.enable_topo_cache<FACE>();


//		// TOPO CACHE

		{
			std::chrono::time_point<std::chrono::system_clock> start, end;
			start = std::chrono::system_clock::now();

			for (unsigned int i = 0u ; i < ITERATIONS; ++i)
			{
				map.template foreach_cell<FACE, cgogn::TraversalStrategy::FORCE_TOPO_CACHE>([&] (Face f)
				{
					face_normal[f] = cgogn::geometry::face_normal<Vec3>(map, f, vertex_position);
				});
			}
			end =  std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = end - start;
			std::cout << std::fixed << "SINGLE THREAD  "<< elapsed_seconds.count() << "s  || compute_normal_faces topo cache"  << std::endl;
		}

		{
			std::chrono::time_point<std::chrono::system_clock> start, end;
			start = std::chrono::system_clock::now();
			for (unsigned int i = 0u ; i < ITERATIONS; ++i)
			{
				map.template parallel_foreach_cell<FACE, cgogn::TraversalStrategy::FORCE_TOPO_CACHE>([&] (Face f, unsigned int)
				{
					face_normal_mt[f] = cgogn::geometry::face_normal<Vec3>(map, f, vertex_position);
				});
			}


			end =  std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = end - start;
			std::cout << std::fixed << cgogn::NB_THREADS << "      THREADS "<< elapsed_seconds.count() << "s  || compute_normal_faces topo cache" << std::endl;
		}

			// END TOPO CACHE
		{
			// CHECKING NORMALS
			map.template foreach_cell<FACE, cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Face f)
			{
				Vec3 error = face_normal[f] - face_normal_mt[f];
				if (!cgogn::almost_equal_absolute(error.squaredNorm(), 0., 1e-9 ))
				{
					std::cerr << __FILE__ << ":" << __LINE__ << " : there was an error during computation of normals" << std::endl;
					std::abort;
				}

			});
		}



		VertexAttributeHandler<Vec3> vertex_normal = map.add_attribute<Vec3, VERTEX>("normal");
		VertexAttributeHandler<Vec3> vertex_normal_mt = map.add_attribute<Vec3, VERTEX>("normal_mt");


		// VERTICES NORMALS



		// DART MARKING

	{
		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();

		for (unsigned int i = 0u ; i < ITERATIONS; ++i)
		{
			map.template foreach_cell<VERTEX, cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Vertex v)
			{
				vertex_normal[v] = cgogn::geometry::vertex_normal<Vec3>(map, v, vertex_position);
			});
		}
		end =  std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		std::cout << std::fixed << "SINGLE THREAD  "<< elapsed_seconds.count() << "s  || compute_normal_vertices dart marking"  << std::endl;
	}

	{
		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();
		for (unsigned int i = 0u ; i < ITERATIONS; ++i)
		{
			map.template parallel_foreach_cell<VERTEX, cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Vertex v, unsigned int)
			{
				vertex_normal_mt[v] = cgogn::geometry::vertex_normal<Vec3>(map, v, vertex_position);
			});
		}

		// END DART MARKING

		end =  std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		std::cout << std::fixed << cgogn::NB_THREADS << "      THREADS "<< elapsed_seconds.count() << "s  || compute_normal_vertices dart marking" << std::endl;
	}


	{
		// CHECKING  VERTEX NORMALS
		map.template foreach_cell<VERTEX, cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Vertex v)
		{
			Vec3 error = vertex_normal[v] - vertex_normal_mt[v];
			if (!cgogn::almost_equal_absolute(error.squaredNorm(), 0., 1e-9 ))
			{
				std::cerr << __FILE__ << ":" << __LINE__ << " : there was an error during computation of normals" << std::endl;
				std::cerr << "vertex_normal " << vertex_normal[v] << std::endl;
				std::cerr << "vertex_normal_mt " <<vertex_normal_mt[v] << std::endl;
				std::abort;
			}

		});
	}


		// CELL MARKING

	{
		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();

		for (unsigned int i = 0u ; i < ITERATIONS; ++i)
		{
			map.template foreach_cell<VERTEX, cgogn::TraversalStrategy::FORCE_CELL_MARKING>([&] (Vertex v)
			{
				vertex_normal[v] = cgogn::geometry::vertex_normal<Vec3>(map, v, vertex_position);
			});
		}
		end =  std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		std::cout << std::fixed << "SINGLE THREAD  "<< elapsed_seconds.count() << "s  || compute_normal_vertices cell marking"  << std::endl;
	}

	{
		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();
		for (unsigned int i = 0u ; i < ITERATIONS; ++i)
		{
			map.template parallel_foreach_cell<VERTEX, cgogn::TraversalStrategy::FORCE_CELL_MARKING>([&] (Vertex v, unsigned int)
			{
				vertex_normal_mt[v] = cgogn::geometry::vertex_normal<Vec3>(map, v, vertex_position);
			});
		}


		end =  std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		std::cout << std::fixed << cgogn::NB_THREADS << "      THREADS "<< elapsed_seconds.count() << "s  || compute_normal_vertices cell marking" << std::endl;
	}

	// END CELL MARKING


	{
		// CHECKING  VERTEX NORMALS
		map.template foreach_cell<VERTEX, cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Vertex v)
		{
			Vec3 error = vertex_normal[v] - vertex_normal_mt[v];
			if (!cgogn::almost_equal_absolute(error.squaredNorm(), 0., 1e-9 ))
			{
				std::cerr << __FILE__ << ":" << __LINE__ << " : there was an error during computation of normals" << std::endl;
				std::cerr << "vertex_normal " << vertex_normal[v] << std::endl;
				std::cerr << "vertex_normal_mt " <<vertex_normal_mt[v] << std::endl;
				std::abort;
			}

		});
	}




		// TOPO CACHE
		map.enable_topo_cache<VERTEX>();

	{
		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();

		for (unsigned int i = 0u ; i < ITERATIONS; ++i)
		{
			map.template foreach_cell<VERTEX, cgogn::TraversalStrategy::FORCE_TOPO_CACHE>([&] (Vertex v)
			{
				vertex_normal[v] = cgogn::geometry::vertex_normal<Vec3>(map, v, vertex_position);
			});
		}
		end =  std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		std::cout << std::fixed << "SINGLE THREAD  "<< elapsed_seconds.count() << "s  || compute_normal_vertices topo cache"  << std::endl;
	}

	{
		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();
		for (unsigned int i = 0u ; i < ITERATIONS; ++i)
		{
			map.template parallel_foreach_cell<VERTEX, cgogn::TraversalStrategy::FORCE_TOPO_CACHE>([&] (Vertex v, unsigned int)
			{
				vertex_normal_mt[v] = cgogn::geometry::vertex_normal<Vec3>(map, v, vertex_position);
			});
		}


		end =  std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		std::cout << std::fixed << cgogn::NB_THREADS << "      THREADS "<< elapsed_seconds.count() << "s  || compute_normal_vertices topo cache" << std::endl;
	}


	{
		// CHECKING  VERTEX NORMALS
		map.template foreach_cell<VERTEX, cgogn::TraversalStrategy::FORCE_DART_MARKING>([&] (Vertex v)
		{
			Vec3 error = vertex_normal[v] - vertex_normal_mt[v];
			if (!cgogn::almost_equal_absolute(error.squaredNorm(), 0., 1e-9 ))
			{
				std::cerr << __FILE__ << ":" << __LINE__ << " : there was an error during computation of normals" << std::endl;
				std::cerr << "vertex_normal " << vertex_normal[v] << std::endl;
				std::cerr << "vertex_normal_mt " <<vertex_normal_mt[v] << std::endl;
				std::abort;
			}

		});
	}



	}



	return 0;
}

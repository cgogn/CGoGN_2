
#include <chrono>
#include <ctime>
#include <vector>

#include <cgogn/core/utils/logger.h>
#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/io/map_import.h>
//#include <cgogn/io/map_export.h>
#include <cgogn/geometry/algos/normal.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using namespace cgogn::numerics;

using Map2 = cgogn::CMap2;

using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<float64,3>>;

template <typename T>
using VertexAttribute = Map2::VertexAttribute<T>;
template <typename T>
using FaceAttribute = Map2::FaceAttribute<T>;

int main(int argc, char** argv)
{
	std::string surfaceMesh;
	if (argc < 2)
	{
		cgogn_log_info("cmap2_import") << "USAGE: " << argv[0] << " [filename]";
		surfaceMesh = std::string(DEFAULT_MESH_PATH) + std::string("off/aneurysm_3D.off");
		cgogn_log_info("cmap2_import") << "Using default mesh : " << surfaceMesh;
	}
	else
		surfaceMesh = std::string(argv[1]);

	Map2 map;

	for (uint32 k = 0; k < 2; ++k)
	{
		cgogn::io::import_surface<Vec3>(map, surfaceMesh);

		uint32 nb_darts = 0;
		map.foreach_dart([&nb_darts] (cgogn::Dart) { nb_darts++; });
		cgogn_log_info("cmap2_import") << "nb darts -> " << nb_darts;

		uint32 nb_darts_2 = 0;
		std::vector<uint32> nb_darts_per_thread(cgogn::thread_pool()->nb_workers());
		for (uint32& n : nb_darts_per_thread)
			n = 0;
		map.parallel_foreach_dart([&nb_darts_per_thread] (cgogn::Dart)
		{
			nb_darts_per_thread[cgogn::current_thread_index()]++;
		});
		for (uint32 n : nb_darts_per_thread)
			nb_darts_2 += n;
		cgogn_log_info("cmap2_import")<< "nb darts // -> " << nb_darts_2;

		VertexAttribute<Vec3> vertex_position = map.get_attribute<Vec3, Map2::Vertex>("position");
		VertexAttribute<Vec3> vertex_normal = map.add_attribute<Vec3, Map2::Vertex>("normal");
		FaceAttribute<Vec3> face_normal = map.add_attribute<Vec3, Map2::Face>("normal");

		cgogn_log_info("cmap2_import")  << "Map integrity : " << std::boolalpha << map.check_map_integrity();

		uint32 nb_vertices = 0;
		Map2::CellCache cache(map);
		cache.build<Map2::Vertex>();
		map.foreach_cell([&nb_vertices] (Map2::Vertex) { nb_vertices++; }, cache);
		cgogn_log_info("cmap2_import") << "nb vertices -> " << nb_vertices;

		uint32 nb_boundary_faces = 0;
		Map2::BoundaryCache bcache(map);
		map.foreach_cell([&nb_boundary_faces] (Map2::Boundary) { nb_boundary_faces++; }, bcache);
		cgogn_log_info("cmap2_import") << "nb boundary faces -> " << nb_boundary_faces;

		uint32 nb_faces = 0;
		map.foreach_cell([&nb_faces] (Map2::Face) { nb_faces++;});
		cgogn_log_info("cmap2_import") << "nb faces -> " << nb_faces;

		uint32 nb_faces_2 = 0;
		std::vector<uint32> nb_faces_per_thread(cgogn::thread_pool()->nb_workers());
		for (uint32& n : nb_faces_per_thread)
			n = 0;
		map.parallel_foreach_cell([&nb_faces_per_thread] (Map2::Face)
		{
			nb_faces_per_thread[cgogn::current_thread_index()]++;
		});
		for (uint32 n : nb_faces_per_thread)
			nb_faces_2 += n;
		cgogn_log_info("cmap2_import") << "nb faces // -> " << nb_faces_2;

		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();

		for	(uint32 i = 0; i < 10; ++i)
			cgogn::geometry::compute_normal<Vec3>(map, vertex_position, face_normal);

		for	(uint32 i = 0; i < 10; ++i)
			cgogn::geometry::compute_normal<Vec3>(map, vertex_position, vertex_normal);

		end = std::chrono::system_clock::now();
		std::chrono::duration<float64> elapsed_seconds = end - start;
		cgogn_log_info("cmap2_import") << "elapsed time: " << elapsed_seconds.count() << "s";
	}

	return 0;
}

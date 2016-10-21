
#include <chrono>
#include <ctime>

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/io/map_import.h>


#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)
using namespace cgogn::numerics;

using Map3 = cgogn::CMap3;

using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<float64,3>>;

template <typename T>
using VertexAttribute = Map3::VertexAttribute<T>;
template <typename T>
using FaceAttribute = Map3::FaceAttribute<T>;

int main(int argc, char** argv)
{
	std::string volumeMesh;
	if (argc < 2)
	{
		cgogn_log_info("cmap3_import") << "USAGE: " << argv[0] << " [filename]";
		volumeMesh = std::string(DEFAULT_MESH_PATH) + std::string("medit/hex_dominant.mesh");
		cgogn_log_info("cmap3_import") << "Using default mesh \"" << volumeMesh << "\".";
	}
	else
		volumeMesh = std::string(argv[1]);

	Map3 map;

	for (uint32 k = 0; k < 2; ++k)
	{
		cgogn::io::import_volume<Vec3>(map, volumeMesh);

		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();

		VertexAttribute<Vec3> vertex_position = map.get_attribute<Vec3, Map3::Vertex>("position");

//		map.enable_topo_cache<Map3::Volume::ORBIT>();
//		map.enable_topo_cache<Map3::Face::ORBIT>();
//		map.enable_topo_cache<Map3::Vertex::ORBIT>();
//		map.enable_topo_cache<Map3::Edge::ORBIT>();

		uint32 nbw = 0u;
		map.foreach_cell([&nbw] (Map3::Volume)
		{
			++nbw;
		});

		uint32 nbf = 0u;
		map.foreach_cell([&] (Map3::Face f)
		{
			++nbf;
			Vec3 v1 = vertex_position[Map3::Vertex(map.phi1(f.dart))] - vertex_position[Map3::Vertex(f.dart)];
			Vec3 v2 = vertex_position[Map3::Vertex(map.phi_1(f.dart))] - vertex_position[Map3::Vertex(f.dart)];
		});

		uint32 nbv = 0;
		map.foreach_cell([&] (Map3::Vertex v)
		{
			++nbv;
			uint32 nb_incident = 0;
			map.foreach_incident_face(v, [&] (Map3::Face /*f*/)
			{
				++nb_incident;
			});
		});

		uint32 nbe = 0;
		map.foreach_cell([&nbe] (Map3::Edge)
		{
			++nbe;
		});

		cgogn_log_info("cmap3_import") << "nb vertices -> " << nbv;
		cgogn_log_info("cmap3_import") << "nb edges -> " << nbe;
		cgogn_log_info("cmap3_import") << "nb faces -> " << nbf;
		cgogn_log_info("cmap3_import") << "nb volumes -> " << nbw;

		end = std::chrono::system_clock::now();
		std::chrono::duration<float64> elapsed_seconds = end - start;
		cgogn_log_info("cmap3_import") << "elapsed time: " << elapsed_seconds.count() << "s";

	}

	return 0;
}

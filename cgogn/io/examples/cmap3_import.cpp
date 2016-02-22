
#include <chrono>
#include <ctime>

#include <core/cmap/cmap3.h>
#include <io/map_import.h>


#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)


using Map3 = cgogn::CMap3<cgogn::DefaultMapTraits>;

//using Vec3 = Eigen::Vector3d;
using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

template <typename T>
using VertexAttributeHandler = Map3::VertexAttributeHandler<T>;
template <typename T>
using FaceAttributeHandler = Map3::FaceAttributeHandler<T>;

int main(int argc, char** argv)
{
	std::string volumeMesh;
	if (argc < 2)
	{
		std::cout << "USAGE: " << argv[0] << " [filename]" << std::endl;
		volumeMesh = std::string(DEFAULT_MESH_PATH) + std::string("liverHexa.vtu");
		std::cout << "Using default mesh : " << volumeMesh << std::endl;
	}
	else
		volumeMesh = std::string(argv[1]);

	Map3 map;

	for (unsigned int k = 0; k < 2; ++k)
	{
		cgogn::io::import_volume<Vec3>(map, volumeMesh);

		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();

		VertexAttributeHandler<Vec3> vertex_position = map.get_attribute<Vec3, Map3::VERTEX>("position");

		map.enable_topo_cache<Map3::VOLUME>();
		map.enable_topo_cache<Map3::FACE>();
		map.enable_topo_cache<Map3::VERTEX>();
		map.enable_topo_cache<Map3::EDGE>();


		unsigned int nbw = 0u;
		map.foreach_cell([&nbw] (Map3::Volume)
		{
			++nbw;
		});

		unsigned int nbf = 0u;
		map.foreach_cell([&] (Map3::Face f)
		{
			++nbf;
			Vec3 v1 = vertex_position[map.phi1(f.dart)] - vertex_position[f.dart];
			Vec3 v2 = vertex_position[map.phi_1(f.dart)] - vertex_position[f.dart];
		});

		unsigned int nbv = 0;
		map.foreach_cell([&] (Map3::Vertex v)
		{
			++nbv;
			unsigned int nb_incident = 0;
			map.foreach_incident_face(v, [&] (Map3::Face f)
			{
				++nb_incident;
			});
		});

		unsigned int nbe = 0;
		map.foreach_cell([&nbe] (Map3::Edge)
		{
			++nbe;
		});

		std::cout << "nb vertices -> " << nbv << std::endl;
		std::cout << "nb edges -> " << nbe << std::endl;
		std::cout << "nb faces -> " << nbf << std::endl;
		std::cout << "nb volumes -> " << nbw << std::endl;

		end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

	}

	return 0;
}


#include <chrono>
#include <ctime>

#include <core/map/cmap2.h>

CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_ON
#include <Eigen/Dense>
CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_OFF

#include <io/map_import.h>


#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

struct MyDataTraits : public cgogn::CMap2DataTraits
{
	using Vec3 = Eigen::Vector3d;
};
using Map2 = cgogn::CMap2_T<MyDataTraits, cgogn::CMap2TopoTraits<MyDataTraits>> ;

using Vec3 = Map2::DataTraits::Vec3;
using SurfaceImport = cgogn::io::SurfaceImport<Map2::DataTraits, Map2::TopoTraits>;
template<typename T>
using VertexAttributeHandler = Map2::VertexAttributeHandler<T>;
template<typename T>
using FaceAttributeHandler = Map2::FaceAttributeHandler<T>;

int main(int argc, char** argv)
{
	std::string surfaceMesh;
	if (argc < 2)
	{
		std::cout << "USAGE: " << argv[0] << " [filename]" << std::endl;
		surfaceMesh = std::string(DEFAULT_MESH_PATH) + std::string("aneurysm3D_1.off");
		std::cout << "Using default mesh : " << surfaceMesh << std::endl;
	} else {
		surfaceMesh = std::string(argv[1]);
	}

	cgogn::thread_start();
	Map2 map;
	for (int k = 0 ; k < 2 ; ++k)
	{

		cgogn::io::import_surface(map,surfaceMesh);

		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();

		VertexAttributeHandler<Vec3> vertex_position = map.get_attribute<Vec3, Map2::VERTEX>("position");
		VertexAttributeHandler<Vec3> vertex_normal = map.add_attribute<Vec3, Map2::VERTEX>("normal");
		FaceAttributeHandler<Vec3> face_normal = map.add_attribute<Vec3, Map2::FACE>("normal");

		map.enable_topo_cache<Map2::FACE>();
		map.enable_topo_cache<Map2::VERTEX>();
		map.enable_topo_cache<Map2::EDGE>();

		unsigned int nbf = 0;

		for	(unsigned int i = 0; i < 10; ++i)
		{
			map.foreach_cell<Map2::FACE>([&] (Map2::Face f)
			{
				++nbf;
				Vec3 v1 = vertex_position[map.phi1(f.dart)] - vertex_position[f.dart];
				Vec3 v2 = vertex_position[map.phi_1(f.dart)] - vertex_position[f.dart];
				face_normal[f] = v1.cross(v2);
			});
		}

		unsigned int nbv = 0;

		for	(unsigned int i = 0; i < 10; ++i)
		{
			map.foreach_cell<Map2::VERTEX>([&] (Map2::Vertex v)
			{
				++nbv;
				Vec3 sum({0, 0, 0});
				unsigned int nb_incident = 0;
				map.foreach_incident_face(v, [&] (Map2::Face f)
				{
					++nb_incident;
					sum += face_normal[f];
				});
				vertex_normal[v] = sum / nb_incident;
			});
		}

		unsigned int nbe = 0;

		for	(unsigned int i = 0; i < 10; ++i)
		{
			map.foreach_cell<Map2::EDGE>([&nbe] (Map2::Edge)
			{
				++nbe;
			});
		}

		std::cout << "nb vertices -> " << nbv << std::endl;
		std::cout << "nb edges -> " << nbe << std::endl;
		std::cout << "nb faces -> " << nbf << std::endl;

		end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
	}
	cgogn::thread_stop();
	return 0;
}


#include <chrono>
#include <ctime>

#include <core/cmap/cmap3.h>

CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_ON
#include <Eigen/Dense>
CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_OFF

#include <io/map_import.h>


#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)


using Map3 = cgogn::CMap3<cgogn::DefaultMapTraits>;

using Vec3 = Eigen::Vector3d;
//using Vec3 = std::array<double,3>;

template<typename T>
using VertexAttributeHandler = Map3::VertexAttributeHandler<T>;
template<typename T>
using FaceAttributeHandler = Map3::FaceAttributeHandler<T>;

int main(int argc, char** argv)
{
	std::string volumeMesh;
	if (argc < 2)
	{
		std::cout << "USAGE: " << argv[0] << " [filename]" << std::endl;
		volumeMesh = std::string(DEFAULT_MESH_PATH) + std::string("Armadillo_Tetra_4406.vtu");
		std::cout << "Using default mesh : " << volumeMesh << std::endl;
	} else {
		volumeMesh = std::string(argv[1]);
	}

	cgogn::thread_start();
	Map3 map;
	//	for (int k = 0 ; k < 2 ; ++k)
	//	{
	cgogn::io::import_volume<Vec3>(map, volumeMesh);

//	std::cout << "nb vertices -> " << nbv << std::endl;
//	std::cout << "nb edges -> " << nbe << std::endl;
//	std::cout << "nb faces -> " << nbf << std::endl;

	//	}
	cgogn::thread_stop();
	return 0;
}

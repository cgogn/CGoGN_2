
#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/io/map_import.h>
#include <cgogn/modeling/algos/pliant_remeshing.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

using Vertex = cgogn::CMap2::Vertex;
template <typename T>
using VertexAttribute = cgogn::CMap2::VertexAttribute<T>;

int main(int argc, char** argv)
{
	std::string surface_mesh;
	if (argc < 2)
	{
		cgogn_log_info("cmap2_import") << "USAGE: " << argv[0] << " [filename]";
		surface_mesh = std::string(DEFAULT_MESH_PATH) + std::string("off/aneurysm_3D.off");
		cgogn_log_info("cmap2_import") << "Using default mesh : " << surface_mesh;
	}
	else
		surface_mesh = std::string(argv[1]);

	cgogn::CMap2 map;

	cgogn::io::import_surface<Vec3>(map, surface_mesh);

	VertexAttribute<Vec3> vertex_position = map.get_attribute<Vec3, Vertex>("position");
	cgogn::modeling::pliant_remeshing<Vec3>(map, vertex_position);
}

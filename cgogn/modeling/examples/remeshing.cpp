
#include <core/cmap/cmap2.h>
#include <io/map_import.h>
#include <geometry/algos/normal.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;

using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

template <typename T>
using VertexAttributeHandler = Map2::VertexAttributeHandler<T>;
template <typename T>
using FaceAttributeHandler = Map2::FaceAttributeHandler<T>;

int main(int argc, char** argv)
{

}

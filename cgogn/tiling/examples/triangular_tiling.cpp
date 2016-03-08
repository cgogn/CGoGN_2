
#include <core/cmap/cmap2.h>
#include <io/map_export.h>
#include <tiling/triangular.h>

struct MyMapTraits : public cgogn::DefaultMapTraits
{
	static const unsigned int CHUNK_SIZE = 8192;
};

using Map2 = cgogn::CMap2<MyMapTraits>;

using Vec3 = Eigen::Vector3d;

template <typename T>
using VertexAttributeHandler = Map2::VertexAttributeHandler<T>;

int main(int argc, char** argv)
{
	int x = 10;
	int y = 10;
	bool close = true;

	Map2 map;
	cgogn::tiling::Grid<Map2> g(map, x, y, close);

	VertexAttributeHandler<Vec3> vertex_grid = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("grid");
	g.embed_into_grid<Vec3>(vertex_grid, 10.0f, 10.0f, 0.0f);

	VertexAttributeHandler<Vec3> vertex_twisted_strip = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("twisted_strip");
	g.embed_into_twisted_strip<Vec3>(vertex_twisted_strip, 10.0f, 5.0f, 3.0f);

	VertexAttributeHandler<Vec3> vertex_helicoid = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("helicoid");
	g.embed_into_helicoid<Vec3>(vertex_helicoid, 10.0f, 5.0f, 15.0f, 3.0f, 1);

	// cgogn::io::export_off<Vec3, Map2>(map, vertex_grid, "grid.off");
	// cgogn::io::export_off<Vec3, Map2>(map, vertex_twisted_strip, "twisted_strip.off");
	// cgogn::io::export_off<Vec3, Map2>(map, vertex_helicoid, "helicoid.off");

	return 0;
}

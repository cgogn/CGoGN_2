#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/io/map_export.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/modeling/tiling/triangular_grid.h>
#include <cgogn/modeling/tiling/triangular_cylinder.h>
#include <cgogn/modeling/tiling/triangular_tore.h>
#include <cgogn/modeling/tiling/triangular_cube.h>

using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;

using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

template <typename T>
using VertexAttribute = Map2::VertexAttribute<T>;

int main(int , char** )
{
	int x = 100;
	int y = 100;

	const cgogn::Orbit vertorb = Map2::Vertex::ORBIT;

	{
		Map2 map;
		cgogn::modeling::TriangularGrid<Map2> g(map, x, y);


		VertexAttribute<Vec3> vertex_grid = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("grid");
		g.embed_into_grid(vertex_grid, 10.0f, 10.0f, 0.0f);

		VertexAttribute<Vec3> vertex_twisted_strip = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("twisted_strip");
		g.embed_into_twisted_strip<Vec3>(vertex_twisted_strip, 10.0f, 5.0f, 3.1f);

		VertexAttribute<Vec3> vertex_helicoid = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("helicoid");
		g.embed_into_helicoid<Vec3>(vertex_helicoid, 10.0f, 5.0f, 15.0f, 3.0f, 1);

		cgogn::io::export_surface(map, cgogn::io::ExportOptions("grid.off", {vertorb, "grid"}, {}, false, false));
		cgogn::io::export_surface(map, cgogn::io::ExportOptions("twisted_strip.off", {vertorb, "twisted_strip"}, {}, false, false));
		cgogn::io::export_surface(map, cgogn::io::ExportOptions("helicoid.off", {vertorb, "helicoid"}, {}, false, false));
	}

	{
		Map2 map;
		cgogn::modeling::TriangularCylinder<Map2> g(map, x, y);

		g.close_top();
		g.close_bottom();
		g.triangule_top();
		g.triangule_bottom();
		VertexAttribute<Vec3> vertex_cylinder = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("cylinder");
		g.embed_into_cylinder(vertex_cylinder, 10.0f, 8.0f, 5.0f);

		VertexAttribute<Vec3> vertex_sphere = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("sphere");
		g.embed_into_sphere<Vec3>(vertex_sphere, 10.0f);

		VertexAttribute<Vec3> vertex_cone = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("cone");
		g.embed_into_cone<Vec3>(vertex_cone, 10.0f, 5.0f);

		cgogn::io::export_surface(map, cgogn::io::ExportOptions("cylinder.off", {vertorb, "cylinder"}, {}, false, false));
		cgogn::io::export_surface(map, cgogn::io::ExportOptions("sphere.off", {vertorb, "sphere"}, {}, false, false));
		cgogn::io::export_surface(map, cgogn::io::ExportOptions("cone.off", {vertorb, "cone"}, {}, false, false));
	}

	{
		Map2 map;
		cgogn::modeling::TriangularTore<Map2> g(map, x, y);

		g.vertices();

		VertexAttribute<Vec3> vertex_tore = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("tore");
		g.embed_into_tore(vertex_tore, 10.0f, 4.0f);

		cgogn::io::export_surface(map, cgogn::io::ExportOptions("tore.off", {vertorb, "tore"}, {}, false, false));
	}

	/*	{
		Map2 map;
		cgogn::modeling::TriangularCube<Map2> g(map, x, y, x);

		VertexAttribute<Vec3> vertex_tore = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("cube");
		g.embed_into_cube(vertex_tore, 10.0f, 4.0f, 5.0f);

		std::vector<std::pair<cgogn::Orbit,std::string>> att_vec;
		att_vec.push_back(std::make_pair(cgogn::Orbit(Map2::Vertex::ORBIT), std::string("cube")));
		cgogn::io::export_surface(map, cgogn::io::ExportOptions("cube.off", att_vec, false));
	}
*/
	return 0;
}

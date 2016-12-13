#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/io/map_export.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/modeling/tiling/triangular_grid.h>
#include <cgogn/modeling/tiling/triangular_cylinder.h>
#include <cgogn/modeling/tiling/triangular_tore.h>
//#include <cgogn/modeling/tiling/triangular_cube.h>

using Map2 = cgogn::CMap2;

using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

using int32 = cgogn::numerics::int32;

template <typename T>
using VertexAttribute = Map2::VertexAttribute<T>;

int main(int , char** )
{
	int32 x = 100;
	int32 y = 100;

	const cgogn::Orbit vertorb = Map2::Vertex::ORBIT;

	{
		Map2 map;
		VertexAttribute<Vec3> vertex_grid = map.add_attribute<Vec3, Map2::Vertex>("grid");
		VertexAttribute<Vec3> vertex_twisted_strip = map.add_attribute<Vec3, Map2::Vertex>("twisted_strip");
		VertexAttribute<Vec3> vertex_helicoid = map.add_attribute<Vec3, Map2::Vertex>("helicoid");

		//		map.add_attribute<int32, Map2::CDart>("darts");
		map.add_attribute<int32, Map2::Edge>("edges");
		map.add_attribute<int32, Map2::Face>("faces");
		map.add_attribute<int32, Map2::Volume>("volumes");

		cgogn::modeling::TriangularGrid<Map2> g(map, x, y);

		std::cout << "is good ? " << std::boolalpha << map.check_map_integrity() << std::endl;

		g.embed_into_grid(vertex_grid, 10.0f, 10.0f, 0.0f);
		g.embed_into_twisted_strip<Vec3>(vertex_twisted_strip, 10.0f, 5.0f, 3.1f);
		g.embed_into_helicoid<Vec3>(vertex_helicoid, 10.0f, 5.0f, 15.0f, 3.0f, 1);

		cgogn::io::export_surface(map, cgogn::io::ExportOptions::create().filename("grid.off").position_attribute(vertorb, "grid").binary(false));
		cgogn::io::export_surface(map, cgogn::io::ExportOptions::create().filename("twisted_strip.off").position_attribute(vertorb, "twisted_strip").binary(false));
		cgogn::io::export_surface(map, cgogn::io::ExportOptions::create().filename("helicoid.off").position_attribute(vertorb, "helicoid").binary(false));
	}

	{
		Map2 map;
		VertexAttribute<Vec3> vertex_cylinder = map.add_attribute<Vec3, Map2::Vertex>("cylinder");
		VertexAttribute<Vec3> vertex_sphere = map.add_attribute<Vec3, Map2::Vertex>("sphere");
		VertexAttribute<Vec3> vertex_cone = map.add_attribute<Vec3, Map2::Vertex>("cone");

		//		map.add_attribute<int32, Map2::CDart>("darts");
		map.add_attribute<int32, Map2::Edge>("edges");
		map.add_attribute<int32, Map2::Face>("faces");
		map.add_attribute<int32, Map2::Volume>("volumes");

		cgogn::modeling::TriangularCylinder<Map2> g(map, x, y);

		std::cout << "is good ? " << std::boolalpha << map.check_embedding_integrity() << std::endl;

		g.close_top();
		g.close_bottom();
		g.triangule_top();
		g.triangule_bottom();

		g.embed_into_cylinder(vertex_cylinder, 10.0f, 8.0f, 5.0f);
		g.embed_into_sphere<Vec3>(vertex_sphere, 10.0f);
		g.embed_into_cone<Vec3>(vertex_cone, 10.0f, 5.0f);

		cgogn::io::export_surface(map, cgogn::io::ExportOptions::create().filename("cylinder.off").position_attribute(vertorb, "cylinder").binary(false));
		cgogn::io::export_surface(map, cgogn::io::ExportOptions::create().filename("sphere.off").position_attribute(vertorb, "sphere").binary(false));
		cgogn::io::export_surface(map, cgogn::io::ExportOptions::create().filename("cone.off").position_attribute(vertorb, "cone").binary(false));
	}

	{
		Map2 map;
		VertexAttribute<Vec3> vertex_tore = map.add_attribute<Vec3, Map2::Vertex>("tore");

		//		map.add_attribute<int32, Map2::CDart>("darts");
		map.add_attribute<int32, Map2::Edge>("edges");
		map.add_attribute<int32, Map2::Face>("faces");
		map.add_attribute<int32, Map2::Volume>("volumes");

		cgogn::modeling::TriangularTore<Map2> g(map, x, y);

		std::cout << "is good ? " << std::boolalpha << map.check_embedding_integrity() << std::endl;

		g.embed_into_tore(vertex_tore, 10.0f, 4.0f);

		cgogn::io::export_surface(map, cgogn::io::ExportOptions::create().filename("tore.off").position_attribute(vertorb, "tore").binary(false));
	}

	/*	{
				Map2 map;
				cgogn::modeling::TriangularCube<Map2> g(map, x, y, x);

				VertexAttribute<Vec3> vertex_tore = map.add_attribute<Vec3, Map2::Vertex>("cube");
				g.embed_into_cube(vertex_tore, 10.0f, 4.0f, 5.0f);

				std::vector<std::pair<cgogn::Orbit,std::string>> att_vec;
				att_vec.push_back(std::make_pair(cgogn::Orbit(Map2::Vertex::ORBIT), std::string("cube")));
				cgogn::io::export_surface(map, cgogn::io::ExportOptions("cube.off", att_vec, false));
		}
*/
	return 0;
}

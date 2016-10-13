#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/io/map_export.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/modeling/tiling/triangular_grid.h>
#include <cgogn/modeling/tiling/triangular_cylinder.h>
#include <cgogn/modeling/tiling/triangular_tore.h>

using Map2 = cgogn::CMap2<cgogn::DefaultMapTraits>;

using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

template <typename T>
using VertexAttribute = Map2::VertexAttribute<T>;

int main(int , char** )
{
	int x = 100;
	int y = 100;

	{
		std::cout << "grid tiling" << std::endl;
		Map2 map;
		cgogn::modeling::TriangularGrid<Map2> g(map, x, y);


		VertexAttribute<Vec3> vertex_grid = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("grid");
		g.embed_into_grid(vertex_grid, 10.0f, 10.0f, 0.0f);

		VertexAttribute<Vec3> vertex_twisted_strip = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("twisted_strip");
		g.embed_into_twisted_strip<Vec3>(vertex_twisted_strip, 10.0f, 5.0f, 3.1f);

		VertexAttribute<Vec3> vertex_helicoid = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("helicoid");
		g.embed_into_helicoid<Vec3>(vertex_helicoid, 10.0f, 5.0f, 15.0f, 3.0f, 1);

		{
			std::vector<std::pair<cgogn::Orbit,std::string>> att_vec;
			att_vec.push_back(std::make_pair(cgogn::Orbit(Map2::Vertex::ORBIT), std::string("grid")));
			cgogn::io::export_surface(map, cgogn::io::ExportOptions("grid.off", att_vec, false));
		}

		{
			std::vector<std::pair<cgogn::Orbit,std::string>> att_vec;
			att_vec.push_back(std::make_pair(cgogn::Orbit(Map2::Vertex::ORBIT), std::string("twisted_strip")));
			cgogn::io::export_surface(map, cgogn::io::ExportOptions("twisted_strip.off", att_vec, false));
		}

		{
			std::vector<std::pair<cgogn::Orbit,std::string>> att_vec;
			att_vec.push_back(std::make_pair(cgogn::Orbit(Map2::Vertex::ORBIT), std::string("helicoid")));
			cgogn::io::export_surface(map, cgogn::io::ExportOptions("helicoid.off", att_vec, false));
		}
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

		{
			std::vector<std::pair<cgogn::Orbit,std::string>> att_vec;
			att_vec.push_back(std::make_pair(cgogn::Orbit(Map2::Vertex::ORBIT), std::string("cylinder")));
			cgogn::io::export_surface(map, cgogn::io::ExportOptions("cylinder.off", att_vec, false));
		}

		{
			std::vector<std::pair<cgogn::Orbit,std::string>> att_vec;
			att_vec.push_back(std::make_pair(cgogn::Orbit(Map2::Vertex::ORBIT), std::string("sphere")));
			cgogn::io::export_surface(map, cgogn::io::ExportOptions("sphere.off", att_vec, false));
		}

		{
			std::vector<std::pair<cgogn::Orbit,std::string>> att_vec;
			att_vec.push_back(std::make_pair(cgogn::Orbit(Map2::Vertex::ORBIT), std::string("cone")));
			cgogn::io::export_surface(map, cgogn::io::ExportOptions("cone.off", att_vec, false));
		}
	}

	{
		Map2 map;
		cgogn::modeling::TriangularTore<Map2> g(map, x, y);

		VertexAttribute<Vec3> vertex_tore = map.add_attribute<Vec3, Map2::Vertex::ORBIT>("tore");
		g.embed_into_tore(vertex_tore, 10.0f, 4.0f);

		std::vector<std::pair<cgogn::Orbit,std::string>> att_vec;
		att_vec.push_back(std::make_pair(cgogn::Orbit(Map2::Vertex::ORBIT), std::string("tore")));
		cgogn::io::export_surface(map, cgogn::io::ExportOptions("tore.off", att_vec, false));
	}
    return 0;
}

#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/modeling/tiling/square_tore.h>
#include <cgogn/geometry/algos/area.h>

#include <cgogn/geometry/algos/angle.h>


using namespace cgogn;

template <typename T>
using VertexAttribute = CMap2::VertexAttribute<T>;
using Vertex = CMap2::Vertex;
using Edge = CMap2::Edge;
using Face = CMap2::Face;

using StdArrayd = cgogn::geometry::Vec_T<std::array<double,3>>;
using EigenVec3d = Eigen::Vector3d;


int main()
{

//	Eigen::Vector2f P1 = {1,1};
//	Eigen::Vector2f P2 = {4,1};
//	Eigen::Vector2f P3 = {1,4};

//	double ar = geometry::area(P1,P2,P3);
//	cgogn_log_info("AR = " ) << ar;

	EigenVec3d P1={1,1,1};
	StdArrayd Q1={1,1,1};

	geometry::normalize_safe(P1);
	geometry::normalize_safe(Q1);

	std::cout << P1 << std::endl;
	std::cout << Q1 << std::endl;


	CMap2 cmap;
	VertexAttribute<EigenVec3d> vertex_position1 = cmap.add_attribute<EigenVec3d, Vertex>("eigen_double");
	VertexAttribute<StdArrayd> vertex_position2 = cmap.add_attribute<StdArrayd, Vertex>("array_double");

	cgogn::modeling::SquareGrid<CMap2> grid(cmap, 500, 500);
	grid.embed_into_grid(vertex_position1,10.0,10.0,0.0);
	grid.embed_into_grid(vertex_position2,10.0,10.0,0.0);

	double a=0.0;
	double b=0.0;
	float s_a=0.0f;
	float s_b=0.0f;

	for(int i=0;i<100; ++i)
	{
		std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
		cmap.foreach_cell([&] (Face f)
		{
            b += cgogn::geometry::area(cmap, f, vertex_position2);
		});
		std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
		std::chrono::duration<float> elapsed_seconds = end - start;
		s_b += elapsed_seconds.count();

		start = std::chrono::system_clock::now();
		cmap.foreach_cell([&] (Face f)
		{
            a += cgogn::geometry::area(cmap, f, vertex_position1);
		});
		end = std::chrono::system_clock::now();
		elapsed_seconds = end - start;
		s_a += elapsed_seconds.count();
	}

	cgogn_log_info("BENCH PURE EIGEN" ) << s_a << " s";
	cgogn_log_info("BENCH MAP EIGEN" ) << s_b << " s";

	std::cout << a << " == " << b << std::endl;

}

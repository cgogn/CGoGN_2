
#include <chrono>
#include <ctime>

#include <core/map/cmap2.h>

#include <Eigen/Dense>

using namespace cgogn;

typedef Eigen::Vector3d VEC3;

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "USAGE: " << argv[0] << " [filename]" << std::endl;
		return 0;
	}

	cgogn::thread_start();

	CMap2 map;
	map.import(argv[1]);

	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();

	CMap2::VertexAttributeHandler<VEC3> vertex_position = map.get_attribute<VEC3, CMap2::VERTEX>("position");
	CMap2::VertexAttributeHandler<VEC3> vertex_normal = map.add_attribute<VEC3, CMap2::VERTEX>("normal");
	CMap2::FaceAttributeHandler<VEC3> face_normal = map.add_attribute<VEC3, CMap2::FACE>("normal");

	unsigned int nbf = 0;
	map.foreach_cell<CMap2::FACE>([&] (CMap2::Face f)
	{
		++nbf;
		VEC3 v1 = vertex_position[map.phi1(f.dart)] - vertex_position[f.dart];
		VEC3 v2 = vertex_position[map.phi_1(f.dart)] - vertex_position[f.dart];
		face_normal[f] = v1.cross(v2);
	});

	map.foreach_cell<CMap2::FACE>([&] (CMap2::Face f)
	{
		++nbf;
		VEC3 v1 = vertex_position[map.phi1(f.dart)] - vertex_position[f.dart];
		VEC3 v2 = vertex_position[map.phi_1(f.dart)] - vertex_position[f.dart];
		face_normal[f] = v1.cross(v2);
	});

	unsigned int nbv = 0;
	map.foreach_cell<CMap2::VERTEX>([&] (CMap2::Vertex v)
	{
		++nbv;
		VEC3 sum({0, 0, 0});
		unsigned int nb_incident = 0;
		map.foreach_incident_face(v, [&] (CMap2::Face f)
		{
			++nb_incident;
			sum += face_normal[f];
		});
		vertex_normal[v] = sum / nb_incident;
	});

	unsigned int nbe = 0;
//	map.foreach_cell<CMap2::EDGE>([&nbe] (CMap2::Edge)
//	{
//		++nbe;
//	});

	std::cout << "nb vertices -> " << nbv << std::endl;
	std::cout << "nb edges -> " << nbe << std::endl;
	std::cout << "nb faces -> " << nbf << std::endl;

	end = std::chrono::system_clock::now();

	std::chrono::duration<double> elapsed_seconds = end - start;

	std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

	cgogn::thread_stop();
	return 0;
}

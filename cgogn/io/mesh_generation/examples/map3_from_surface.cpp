/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France       *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Web site: http://cgogn.unistra.fr/                                           *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/

#include <chrono>
#include <ctime>

#include <core/cmap/cmap3.h>
#include <io/mesh_generation/tetgen_io.h>
#include <io/map_import.h>

using MapTraits = cgogn::DefaultMapTraits;
using Map3 = cgogn::CMap3<MapTraits>;
using Map2 = cgogn::CMap2<MapTraits>;
using Vec3 = Eigen::Vector3d;


int main(int argc, char** argv)
{
	std::string surface_path;
	std::string tetgen_arg;
	if (argc < 3)
	{
		std::cout << "argc < 3" << std::endl;
		std::exit(EXIT_FAILURE);
	}
	else
	{
		surface_path = std::string(argv[1]);
		tetgen_arg = std::string(argv[2]);
	}

	Map2 map2;
	std::unique_ptr<tetgenio> tetgen_input;
	{
		cgogn::io::import_surface<Vec3>(map2, surface_path);
		Map2::VertexAttributeHandler<Vec3> vertex_position = map2.get_attribute<Vec3, Map2::Vertex::ORBIT>("position");
		tetgen_input = cgogn::io::export_tetgen<Vec3>(map2, vertex_position);
	}

	Map3 map3;
	{
		tetgenio tetgen_output;
		tetrahedralize(tetgen_arg.c_str(), tetgen_input.get(), &tetgen_output) ;
		cgogn::io::TetgenVolumeImport<MapTraits, Vec3> tetgen_import(&tetgen_output);
		tetgen_import.import_file("");
		tetgen_import.create_map(map3);
	}


	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();

	Map3::VertexAttributeHandler<Vec3> vertex_position = map3.get_attribute<Vec3, Map3::Vertex::ORBIT>("position");

	map3.enable_topo_cache<Map3::Volume::ORBIT>();
	map3.enable_topo_cache<Map3::Face::ORBIT>();
	map3.enable_topo_cache<Map3::Vertex::ORBIT>();
	map3.enable_topo_cache<Map3::Edge::ORBIT>();

	unsigned int nbw = 0u;
	map3.foreach_cell([&nbw] (Map3::Volume)
	{
		++nbw;
	});

	unsigned int nbf = 0u;
	map3.foreach_cell([&] (Map3::Face f)
	{
		++nbf;
		Vec3 v1 = vertex_position[Map3::Vertex(map3.phi1(f.dart))] - vertex_position[Map3::Vertex(f.dart)];
		Vec3 v2 = vertex_position[Map3::Vertex(map3.phi_1(f.dart))] - vertex_position[Map3::Vertex(f.dart)];
	});

	unsigned int nbv = 0;
	map3.foreach_cell([&] (Map3::Vertex v)
	{
		++nbv;
		unsigned int nb_incident = 0;
		map3.foreach_incident_face(v, [&] (Map3::Face /*f*/)
		{
			++nb_incident;
		});
	});

	unsigned int nbe = 0;
	map3.foreach_cell([&nbe] (Map3::Edge)
	{
		++nbe;
	});

	std::cout << "nb vertices -> " << nbv << std::endl;
	std::cout << "nb edges -> " << nbe << std::endl;
	std::cout << "nb faces -> " << nbf << std::endl;
	std::cout << "nb volumes -> " << nbw << std::endl;

	end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end - start;
	std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";


	return 0;
}

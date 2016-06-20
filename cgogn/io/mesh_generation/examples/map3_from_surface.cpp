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

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/io/mesh_generation/tetgen_structure_io.h>
#include <cgogn/io/map_import.h>
#include <cgogn/io/map_export.h>
#include <cgogn/geometry/algos/normal.h>

using namespace cgogn::numerics;


using MapTraits = cgogn::DefaultMapTraits;
using Map3 = cgogn::CMap3<MapTraits>;
using Map2 = cgogn::CMap2<MapTraits>;
using Vec3 = Eigen::Vector3d;


int main(int argc, char** argv)
{
	std::string surface_path;
	std::string tetgen_arg;
	std::string output_filename;
	if (argc < 4)
	{
		cgogn_log_info("map3_from_surface") << "USAGE: " << argv[0] << " [surface_mesh_path] [tetgen_args] [output]";
		std::exit(EXIT_FAILURE);
	}
	else
	{
		surface_path = std::string(argv[1]);
		tetgen_arg = std::string(argv[2]);
		output_filename = std::string(argv[3]);
	}

	Map2 map2;
	std::unique_ptr<tetgenio> tetgen_input;
	{
		cgogn::io::import_surface<Vec3>(map2, surface_path);
		Map2::VertexAttribute<Vec3> vertex_position = map2.get_attribute<Vec3, Map2::Vertex::ORBIT>("position");
		tetgen_input = cgogn::io::export_tetgen<Vec3>(map2, vertex_position);
	}

	Map3 map3;
	{
		tetgenio tetgen_output;
		tetrahedralize(tetgen_arg.c_str(), tetgen_input.get(), &tetgen_output) ;
		cgogn::io::TetgenStructureVolumeImport<MapTraits, Vec3> tetgen_import(&tetgen_output);
		tetgen_import.import_file("");
		tetgen_import.create_map(map3);

	}
	Map3::VertexAttribute<Vec3> vertex_position = map3.get_attribute<Vec3, Map3::Vertex::ORBIT>("position");
	cgogn::io::export_volume(map3, cgogn::io::ExportOptions(output_filename,{cgogn::Orbit(Map3::Vertex::ORBIT), "position"}));
	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();



//	map3.enable_topo_cache<Map3::Volume::ORBIT>();
//	map3.enable_topo_cache<Map3::Face::ORBIT>();
//	map3.enable_topo_cache<Map3::Vertex::ORBIT>();
//	map3.enable_topo_cache<Map3::Edge::ORBIT>();

	uint32 nbw = 0u;
	map3.foreach_cell([&nbw] (Map3::Volume)
	{
		++nbw;
	});

	uint32 nbf = 0u;
	map3.foreach_cell([&] (Map3::Face f)
	{
		++nbf;
		Vec3 v1 = vertex_position[Map3::Vertex(map3.phi1(f.dart))] - vertex_position[Map3::Vertex(f.dart)];
		Vec3 v2 = vertex_position[Map3::Vertex(map3.phi_1(f.dart))] - vertex_position[Map3::Vertex(f.dart)];
	});

	uint32 nbv = 0;
	map3.foreach_cell([&] (Map3::Vertex v)
	{
		++nbv;
		uint32 nb_incident = 0;
		map3.foreach_incident_face(v, [&] (Map3::Face /*f*/)
		{
			++nb_incident;
		});
	});

	uint32 nbe = 0;
	map3.foreach_cell([&nbe] (Map3::Edge)
	{
		++nbe;
	});

	cgogn_log_info("map3_from_surface") << "nb vertices -> " << nbv;
	cgogn_log_info("map3_from_surface") << "nb edges -> " << nbe;
	cgogn_log_info("map3_from_surface") << "nb faces -> " << nbf;
	cgogn_log_info("map3_from_surface") << "nb volumes -> " << nbw;

	end = std::chrono::system_clock::now();
	std::chrono::duration<float64> elapsed_seconds = end - start;
	cgogn_log_info("map3_from_surface") << "elapsed time: " << elapsed_seconds.count() << "s";


	return 0;
}

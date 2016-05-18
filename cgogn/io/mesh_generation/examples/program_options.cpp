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

#include "program_options.h"
#include <iostream>
#include <cgogn/core/utils/definitions.h>

#define DEFAULT_IMAGE_PATH CGOGN_STR(CGOGN_TEST_IMAGES_PATH)

namespace cgogn
{

namespace io
{

namespace po = boost::program_options;

ProgramOptions::ProgramOptions(int argc, char **argv)
{
	desc_.add_options()
			("help,h","Produce this help message.")
			("input,i", po::value< std::string >(&input_file_)->default_value(std::string(DEFAULT_IMAGE_PATH)+ std::string("liver.inr.gz")), "The input file : a .vtk StructuredPoints file, or a *.inr.gz file.")
			("fa", po::value<double>(&facet_angle_)->default_value(30.0), "Facet angle, a lower bound for the angles (in degrees) of the surface mesh facets.")
			("fs", po::value<double>(&facet_size_)->default_value(15.0), "Facet size, a scalar field (resp. a constant) describing a space varying (resp. a uniform) upper-bound or for the radii of the surface Delaunay balls.")
			("fd", po::value<double>(&facet_distance_)->default_value(10.0), "Facet distance, scalar field (resp. a constant) describing a space varying (resp. a uniform) upper bound for the same distance.")
			("cr", po::value<double>(&cell_radius_)->default_value(3.0), "Cell radius edge ratio, an upper bound for the radius-edge ratio of the mesh tetrahedra.")
			("cs", po::value<double>(&cell_size_)->default_value(8.0), "Cell size, a scalar field (resp. a constant) describing a space varying (resp. a uniform) upper-bound for the circumradii of the mesh tetrahedra.")
			("lloyd",po::value< bool >(&lloyd_)->default_value(true ),"Optimization parameter : Lloyd smoothing.")
			("lloyd_freeze",po::value< bool >(&lloyd_freeze_)->default_value(true )," Completes the freeze_bound parameter. If it is set to true (default value), frozen vertices will not move anymore in next iterations. Otherwise, at each iteration, any vertex that moves, unfreezes all its incident vertices.")
			("lloyd_freeze_bound",po::value< double >(&lloyd_freeze_bound_)->default_value(0.01 ),"Designed to reduce running time of each optimization iteration. Any vertex that has a displacement less than a given percentage of the length of its shortest incident edge, is frozen (i.e. is not relocated). The parameter freeze_bound gives the threshold ratio. At each iteration, any vertex that moves, unfreezes all its incident vertices.") //Freeze vertices which move less than VALUE*shortest_incident_edge_length
			("lloyd_convergence",po::value< double >(&lloyd_convergence_)->default_value(0.02 ),"Stopping criterion based on convergence: the optimization process is stopped, when at the last iteration, the displacement of any vertex is less than a given percentage of the length of the shortest edge incident to that vertex. The parameter convergence gives the threshold ratio.")
			("lloyd_max_iterations",po::value< std::size_t >(&lloyd_max_iterations_)->default_value(0),"Sets a limit on the number of performed iterations. The default value of 0 means that there is no limit on the number of performed iteration.")
			("odt",po::value< bool >(&odt_)->default_value(true),"Optimization parameter : odt-smoother.")
			("odt_freeze",po::value< bool >(&odt_freeze_)->default_value(true),"Completes the freeze_bound parameter. If it is set to true (default value), frozen vertices will not move anymore in next iterations. Otherwise, at each iteration, any vertex that moves, unfreezes all its incident vertices.")
			("odt_freeze_bound",po::value< double >(&odt_freeze_bound_)->default_value(0.01 ),"Designed to reduce running time of each optimization iteration. Any vertex that has a displacement less than a given percentage of the length of its shortest incident edge, is frozen (i.e. is not relocated). The parameter freeze_bound gives the threshold ratio. At each iteration, any vertex that moves, unfreezes the neighboring vertices.")
			("odt_convergence",po::value< double >(&odt_convergence_)->default_value(0.02 ),"Stopping criterion based on convergence: the optimization process is stopped, when at the last iteration, the displacement of any vertex is less than a given percentage of the length the shortest edge incident to that vertex. The parameter convergence gives the threshold ratio.")
			("odt_max_iterations",po::value< std::size_t >(&odt_max_iterations_)->default_value(0),"Sets a limit on the number of performed iterations. The default value of 0 means that there is no limit on the number of performed iterations.")
			("perturb",po::value< bool >(&perturb_)->default_value(true),"Optimization parameter : perturber.")
			("perturb_sliver_bound",po::value< double >(&perturb_sliver_bound_)->default_value(0 ),"sliver_bound, is designed to give, in degree,  a targeted lower bound on dihedral angles of mesh cells. The function perturb_mesh_3 runs as long as steps are successful and step number sliver_bound (after which the worst tetrahedron in the mesh has a smallest angle larger than sliver_bound degrees) has not been reached. The default value is 0 and means that there is no targeted bound: the perturber then runs as long as steps are successful.")
			("exude",po::value< bool >(&exude_)->default_value(true),"Optimization parameter : exuder.")
			("exude_sliver_bound",po::value< double >(&exude_sliver_bound_)->default_value(0 ),"designed to give, in degree, a targeted lower bound on dihedral angles of mesh cells. The exudation process considers in turn all the mesh cells that have a smallest dihedral angle less than sliver_bound and tries to make them disappear by weighting their vertices. The optimization process stops when every cell in the mesh achieves this quality. The default value is 0 and means that there is no targeted bound : the exuder then runs as long as it can improve the smallest dihedral angles of the set of cells incident to some vertices..")
			;
	po::store(po::parse_command_line(argc,argv,desc_), variable_map_);
	po::notify(variable_map_);
	this->optionsHandler();
}

ProgramOptions::~ProgramOptions()
{}



bool ProgramOptions::optionsHandler()
{

	if (variable_map_.count("help")) {
		std::cout << desc_ << std::endl;
		exit(0);
	}

	std::cout << "Input image: \"" << input_file_ << "\"." <<std::endl;

	if (variable_map_.count("fa")) {
		std::cout << "Facet angle set to : "
			 << variable_map_["fa"].as<double>()
			 << std::endl;
	}
	if (variable_map_.count("fs")) {
		std::cout << "Facet size set to : "
			 << variable_map_["fs"].as<double>()
			 << std::endl;
	}

	if (variable_map_.count("fd")) {
		std::cout << "Facet distance set to : "
			 << variable_map_["fd"].as<double>()
			 << std::endl;
	}

	if (variable_map_.count("cr")) {
		std::cout << "Cell radius edge ratio set to : "
			 << variable_map_["cr"].as<double>()
			 << std::endl;
	}

	if (variable_map_.count("cs")) {
		std::cout << "Cell size set to : "
			 << variable_map_["cs"].as<double>()
			 << std::endl;
	}

	if (variable_map_.count("lloyd")) {
		std::cout << "Lloyed smoother : "
			 << std::boolalpha
			 <<  variable_map_["lloyd"].as<bool>()
			  << std::endl;
	}

	if (variable_map_.count("odt")) {
		std::cout << "Odt smoother : "
			 << std::boolalpha
			 <<  variable_map_["odt"].as<bool>()
			  << std::endl;
	}

	if (variable_map_.count("perturb")) {
		std::cout << "Perturber : "
			 << std::boolalpha
			 <<  variable_map_["perturb"].as<bool>()
			  << std::endl;
	}

	if (variable_map_.count("exude")) {
		std::cout << "Exuder : "
			 << std::boolalpha
			 <<  variable_map_["exude"].as<bool>()
			  << std::endl;
	}

	return true;
}

} // namespace io
} // namespace cgogn

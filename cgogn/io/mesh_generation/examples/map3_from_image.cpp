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
#include "map3_from_image.h"
#include "program_options.h"

using Map3 = cgogn::CMap3<cgogn::DefaultMapTraits>;

using Vec3 = Eigen::Vector3d;

template <typename T>
using VertexAttribute = Map3::VertexAttribute<T>;
template <typename T>
using FaceAttribute = Map3::FaceAttribute<T>;

namespace cp = CGAL::parameters;
namespace cpi = cp::internal;

int main(int argc, char** argv)
{
	Map3 map;
	cgogn::io::ProgramOptions po(argc,argv);

	cpi::Odt_options odt_options = cp::no_odt();
	cpi::Lloyd_options lloyd_options = cp::no_lloyd();
	cpi::Perturb_options perturb_options = cp::no_perturb();
	cpi::Exude_options exude_options = cp::no_exude();

	if (po.odt_)
	{
		odt_options = cp::odt(
					cp::max_iteration_number = po.odt_max_iterations_,
					cp::convergence = po.odt_convergence_,
					cp::freeze_bound = po.odt_freeze_bound_
					);
	}

	if (po.lloyd_)
	{
		lloyd_options = 	cp::lloyd(
					cp::time_limit= 0.,
					cp::max_iteration_number = po.lloyd_max_iterations_,
					cp::convergence = po.lloyd_convergence_,
					cp::freeze_bound = po.lloyd_freeze_bound_
				);

	}

	if (po.perturb_)
		perturb_options = cp::perturb(cp::sliver_bound = po.perturb_sliver_bound_);

	if (po.exude_)
		exude_options = cp::exude(cp::sliver_bound = po.exude_sliver_bound_);

	cgogn::io::VolumeMeshFromImageCGALTraits::Criteria criteria(cp::facet_angle = po.facet_angle_,
																cp::facet_size = po.facet_size_,
																cp::facet_distance = po.facet_distance_,
																cp::cell_radius_edge_ratio = po.cell_radius_,
																cp::cell_size = po.cell_size_
																);
	cgogn::io::create_map3_from_image<Vec3>(map, po.input_file_, criteria, odt_options, lloyd_options, perturb_options,exude_options);

	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();

	VertexAttribute<Vec3> vertex_position = map.get_attribute<Vec3, Map3::Vertex::ORBIT>("position");


	cgogn::CellCache<Map3> cell_cache(map);
	cell_cache.template build<Map3::Vertex>();
	cell_cache.template build<Map3::Edge>();
	cell_cache.template build<Map3::Face>();
	cell_cache.template build<Map3::Volume>();


	unsigned int nbw = 0u;
	map.foreach_cell([&nbw] (Map3::Volume)
	{
		++nbw;
	}, cell_cache);

	unsigned int nbf = 0u;
	map.foreach_cell([&] (Map3::Face f)
	{
		++nbf;
//		Vec3 v1 = vertex_position[Map3::Vertex(map.phi1(f.dart))] - vertex_position[Map3::Vertex(f.dart)];
//		Vec3 v2 = vertex_position[Map3::Vertex(map.phi_1(f.dart))] - vertex_position[Map3::Vertex(f.dart)];
	}, cell_cache);

	unsigned int nbv = 0;
	map.foreach_cell([&] (Map3::Vertex v)
	{
		++nbv;
		unsigned int nb_incident = 0;
		map.foreach_incident_face(v, [&] (Map3::Face /*f*/)
		{
			++nb_incident;
		});
	}, cell_cache);

	unsigned int nbe = 0;
	map.foreach_cell([&nbe] (Map3::Edge)
	{
		++nbe;
	}, cell_cache);

	cgogn_log_info("map3_from_image") << "nb vertices -> " << nbv;
	cgogn_log_info("map3_from_image") << "nb edges -> " << nbe;
	cgogn_log_info("map3_from_image") << "nb faces -> " << nbf;
	cgogn_log_info("map3_from_image") << "nb volumes -> " << nbw;

	end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end - start;
	cgogn_log_info("map3_from_image") << "elapsed time: " << elapsed_seconds.count() << "s";


	return 0;
}

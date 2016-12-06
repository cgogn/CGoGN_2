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

#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/geometry/algos/centroid.h>
#include <cgogn/io/map_import.h>
#include <cgogn/io/map_export.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using Vec3 = Eigen::Vector3d;
//using Vec3 = cgogn::geometry::Vec_T<std::array<double,3>>;

using Vertex = cgogn::CMap2::Vertex;
using Face = cgogn::CMap2::Face;
template <typename T>
using VertexAttribute = cgogn::CMap2::VertexAttribute<T>;
template <typename T>
using FaceAttribute = cgogn::CMap2::FaceAttribute<T>;

int main(int argc, char** argv)
{
	std::string surface_mesh;
	if (argc < 2)
	{
		cgogn_log_info("cmap2_import") << "USAGE: " << argv[0] << " [filename]";
		surface_mesh = std::string(DEFAULT_MESH_PATH) + std::string("off/aneurysm_3D.off");
		cgogn_log_info("cmap2_import") << "Using default mesh : " << surface_mesh;
	}
	else
		surface_mesh = std::string(argv[1]);

	cgogn::CMap2 map;

	cgogn::io::import_surface<Vec3>(map, surface_mesh);

	VertexAttribute<Vec3> position = map.get_attribute<Vec3, Vertex>("position");
	// Face Attribute -> after dual new VertexAttribute
	FaceAttribute<Vec3> positionF = map.add_attribute<Vec3, Face>("position") ;

	// Compute Centroid for the faces
	cgogn::geometry::compute_centroid<Vec3, Face>(map, position, positionF);

	// Compute the dual of the topology
	map.dual();

	cgogn::io::export_surface(map, cgogn::io::ExportOptions::create().filename("dual.off").position_attribute(Vertex::ORBIT, "position").binary(false));
}

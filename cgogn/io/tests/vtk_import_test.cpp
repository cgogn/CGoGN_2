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


#include <gtest/gtest.h>
#include <string>
#include <cgogn/io/map_import.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using namespace cgogn::numerics;
using Vec3 = Eigen::Vector3d;
using Map2 = cgogn::CMap2;
using Map3 = cgogn::CMap3;
using Vertex2 = Map2::Vertex;
using Vertex3 = Map3::Vertex;

const std::string mesh_path(DEFAULT_MESH_PATH);

TEST(ImportTest, vtk_legacy_surface_import)
{
	Map2 map2;
	cgogn::io::import_surface<Vec3>(map2, mesh_path + "vtk/salad_bowl.vtk");

	auto pos = map2.get_attribute<Vec3, Map2::Vertex>("position");
	auto normal = map2.get_attribute<Vec3, Map2::Vertex>("normal");
	const uint32 nbv = map2.nb_cells<Map2::Vertex::ORBIT>();
	const uint32 nbf = map2.nb_cells<Map2::Face::ORBIT>();

	EXPECT_TRUE(pos.is_valid());
	EXPECT_TRUE(normal.is_valid());
	EXPECT_TRUE(map2.check_map_integrity());
	EXPECT_EQ(nbv, 706u);
	EXPECT_EQ(nbf, 1408u);
}

TEST(ImportTest, vtk_legacy_bin_surface_import)
{
	Map2 map2;
	cgogn::io::import_surface<Vec3>(map2, mesh_path + "vtk/salad_bowl_bin.vtk");

	auto pos = map2.get_attribute<Vec3, Map2::Vertex>("position");
	auto normal = map2.get_attribute<Vec3, Map2::Vertex>("normal");
	const uint32 nbv = map2.nb_cells<Map2::Vertex::ORBIT>();
	const uint32 nbf = map2.nb_cells<Map2::Face::ORBIT>();

	EXPECT_TRUE(pos.is_valid());
	EXPECT_TRUE(normal.is_valid());
	EXPECT_TRUE(map2.check_map_integrity());
	EXPECT_EQ(nbv, 706u);
	EXPECT_EQ(nbf, 1408u);
}

TEST(ImportTest, vtk_vtp_asci_surface_import)
{
	Map2 map2;
	cgogn::io::import_surface<Vec3>(map2, mesh_path + "vtk/salad_bowl.vtp");

	auto pos = map2.get_attribute<Vec3, Map2::Vertex>("position");
	auto normal = map2.get_attribute<Vec3, Map2::Vertex>("normal");
	const uint32 nbv = map2.nb_cells<Map2::Vertex::ORBIT>();
	const uint32 nbf = map2.nb_cells<Map2::Face::ORBIT>();

	EXPECT_TRUE(pos.is_valid());
	EXPECT_TRUE(normal.is_valid());
	EXPECT_TRUE(map2.check_map_integrity());
	EXPECT_EQ(nbv, 706u);
	EXPECT_EQ(nbf, 1408u);
}

TEST(ImportTest, vtk_vtu_ascii_volume_import)
{
	Map3 map;
	cgogn::io::import_volume<Vec3>(map, mesh_path + "vtk/nine_hexas.vtu");

	auto pos = map.get_attribute<Vec3, Vertex3>("position");
	const uint32 nbv = map.nb_cells<Vertex3::ORBIT>();
	const uint32 nbw = map.nb_cells<Map3::Volume::ORBIT>();

	EXPECT_TRUE(pos.is_valid());
	EXPECT_TRUE(map.check_map_integrity());
	EXPECT_EQ(nbv, 32u);
	EXPECT_EQ(nbw, 9u);
}

TEST(ImportTest, vtk_vtu_binary_volume_import)
{
	Map3 map;
	cgogn::io::import_volume<Vec3>(map, mesh_path + "vtk/nine_hexas_bin.vtu");

	auto pos = map.get_attribute<Vec3, Vertex3>("position");
	const uint32 nbv = map.nb_cells<Vertex3::ORBIT>();
	const uint32 nbw = map.nb_cells<Map3::Volume::ORBIT>();

	EXPECT_TRUE(pos.is_valid());
	EXPECT_TRUE(map.check_map_integrity());
	EXPECT_EQ(nbv, 32u);
	EXPECT_EQ(nbw, 9u);
}

TEST(ImportTest, vtk_vtu_binary_zlib_volume_import)
{
	Map3 map;
	cgogn::io::import_volume<Vec3>(map, mesh_path + "vtk/nine_hexas_zlib.vtu");

	auto pos = map.get_attribute<Vec3, Vertex3>("position");
	const uint32 nbv = map.nb_cells<Vertex3::ORBIT>();
	const uint32 nbw = map.nb_cells<Map3::Volume::ORBIT>();

	EXPECT_TRUE(pos.is_valid());
	EXPECT_TRUE(map.check_map_integrity());
	EXPECT_EQ(nbv, 32u);
	EXPECT_EQ(nbw, 9u);
}

TEST(ImportTest, vtk_legacy_ascii_voxel_import)
{
	Map3 map;
	cgogn::io::import_volume<Vec3>(map, mesh_path + "vtk/vox8_ascii.vtk");

	auto cRamp1 = map.get_attribute<float32, Map3::Volume>("cRamp1");
	auto cRamp2 = map.get_attribute<float32, Map3::Volume>("cRamp2");
	auto cVects = map.get_attribute<Eigen::Vector3f, Map3::Volume>("cVects");
	auto cv2 = map.get_attribute<Eigen::Vector3f, Map3::Volume>("cv2");

	auto pos = map.get_attribute<Vec3, Vertex3>("position");
	auto xramp = map.get_attribute<float32, Vertex3>("xRamp");
	auto yramp = map.get_attribute<float32, Vertex3>("yRamp");
	auto zramp = map.get_attribute<float32, Vertex3>("zRamp");
	auto outVect = map.get_attribute<Eigen::Vector3f, Vertex3>("outVect");
	auto vect2 = map.get_attribute<Eigen::Vector3f, Vertex3>("vect2");
	auto mytest = map.get_attribute<float32, Vertex3>("mytest");


	const uint32 nbv = map.nb_cells<Vertex3::ORBIT>();
	const uint32 nbw = map.nb_cells<Map3::Volume::ORBIT>();

	EXPECT_TRUE(pos.is_valid());
	EXPECT_TRUE(map.check_map_integrity());
	EXPECT_EQ(nbv, 27u);
	EXPECT_EQ(nbw, 8u);

	EXPECT_TRUE(cRamp1.is_valid());
	EXPECT_TRUE(cRamp2.is_valid());
	EXPECT_TRUE(cVects.is_valid());
	EXPECT_TRUE(cv2.is_valid());
	EXPECT_TRUE(xramp.is_valid());
	EXPECT_TRUE(yramp.is_valid());
	EXPECT_TRUE(zramp.is_valid());
	EXPECT_TRUE(outVect.is_valid());
	EXPECT_TRUE(vect2.is_valid());
	EXPECT_TRUE(mytest.is_valid());
}

TEST(ImportTest, vtk_legacy_binary_voxel_import)
{
	Map3 map;
	cgogn::io::import_volume<Vec3>(map, mesh_path + "vtk/vox8_binary.vtk");

	auto cRamp1 = map.get_attribute<float32, Map3::Volume>("cRamp1");
	auto cRamp2 = map.get_attribute<float32, Map3::Volume>("cRamp2");
	auto cVects = map.get_attribute<Eigen::Vector3f, Map3::Volume>("cVects");
	auto cv2 = map.get_attribute<Eigen::Vector3f, Map3::Volume>("cv2");

	auto pos = map.get_attribute<Vec3, Vertex3>("position");
	auto xramp = map.get_attribute<float32, Vertex3>("xRamp");
	auto yramp = map.get_attribute<float32, Vertex3>("yRamp");
	auto zramp = map.get_attribute<float32, Vertex3>("zRamp");
	auto outVect = map.get_attribute<Eigen::Vector3f, Vertex3>("outVect");
	auto vect2 = map.get_attribute<Eigen::Vector3f, Vertex3>("vect2");
	auto mytest = map.get_attribute<float32, Vertex3>("mytest");


	const uint32 nbv = map.nb_cells<Vertex3::ORBIT>();
	const uint32 nbw = map.nb_cells<Map3::Volume::ORBIT>();

	EXPECT_TRUE(pos.is_valid());
	EXPECT_TRUE(map.check_map_integrity());
	EXPECT_EQ(nbv, 27u);
	EXPECT_EQ(nbw, 8u);

	EXPECT_TRUE(cRamp1.is_valid());
	EXPECT_TRUE(cRamp2.is_valid());
	EXPECT_TRUE(cVects.is_valid());
	EXPECT_TRUE(cv2.is_valid());
	EXPECT_TRUE(xramp.is_valid());
	EXPECT_TRUE(yramp.is_valid());
	EXPECT_TRUE(zramp.is_valid());
	EXPECT_TRUE(outVect.is_valid());
	EXPECT_TRUE(vect2.is_valid());
	EXPECT_TRUE(mytest.is_valid());
}

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

const std::string mesh_path(DEFAULT_MESH_PATH);

TEST(ImportTest, tet_volume_import)
{
	Map3 map3;
	cgogn::io::import_volume<Vec3>(map3, mesh_path + "tet/hand.tet");

	auto pos = map3.get_attribute<Vec3, Map3::Vertex>("position");
	const uint32 nbv = map3.nb_cells<Map3::Vertex::ORBIT>();
	const uint32 nbw = map3.nb_cells<Map3::Volume::ORBIT>();

	EXPECT_TRUE(pos.is_valid());
	EXPECT_TRUE(map3.check_map_integrity());
	EXPECT_EQ(nbv, 2774u);
	EXPECT_EQ(nbw, 8343u);
}

TEST(ImportTest, tet_with_connectors_volume_import)
{
	Map3 map3;
	cgogn::io::import_volume<Vec3>(map3, mesh_path + "tet/liver.tet");

	auto pos = map3.get_attribute<Vec3, Map3::Vertex>("position");
//	const uint32 nbv = map3.nb_cells<Map2::Vertex::ORBIT>();
	const uint32 nbw = map3.nb_cells<Map3::Volume::ORBIT>();

	EXPECT_TRUE(pos.is_valid());
	EXPECT_TRUE(map3.check_map_integrity());
	EXPECT_EQ(nbw, 4967u);
}


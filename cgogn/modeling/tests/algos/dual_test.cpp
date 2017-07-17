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
#include <cgogn/modeling/algos/dual.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using namespace cgogn::numerics;
using Vec3 = Eigen::Vector3d;
using Map2 = cgogn::CMap2;
using Map3 = cgogn::CMap3;

const std::string mesh_path(DEFAULT_MESH_PATH);

void dual_test(const std::string& filename)
{
	Map2 map2;
	testing::internal::CaptureStderr();
	cgogn::io::import_surface<Vec3>(map2, mesh_path + filename);
	const std::string expected_empty_error_output = testing::internal::GetCapturedStderr();

	auto pos = map2.get_attribute<Vec3, Map2::Vertex>("position");
	uint32 nbv = map2.nb_cells<Map2::Vertex::ORBIT>();
	uint32 nbf = map2.nb_cells<Map2::Face::ORBIT>();
	uint32 nbe = map2.nb_cells<Map2::Edge::ORBIT>();

	const uint32 HISTO_SZ = 48;
	std::vector<uint32> histo_vertices(HISTO_SZ, 0);
	map2.foreach_cell([&histo_vertices,&map2](Map2::Vertex v)
	{
		histo_vertices[map2.degree(v)]++;
	});

	std::vector<uint32> histo_faces(HISTO_SZ, 0);
	map2.foreach_cell([&histo_faces,&map2](Map2::Face f)
	{
		histo_faces[map2.codegree(f)]++;
	});
	
	Map2 map2_dual;
	cgogn::modeling::dual2_topo(map2, map2_dual,true, true, true);
	cgogn::modeling::compute_dual2_vertices<Vec3>(map2, map2_dual, pos);
	auto pos_dual = map2_dual.get_attribute<Vec3, Map2::Vertex>("position");

	uint32 nbvd = map2_dual.nb_cells<Map2::Vertex::ORBIT>();
	uint32 nbfd = map2_dual.nb_cells<Map2::Face::ORBIT>();
	uint32 nbed = map2_dual.nb_cells<Map2::Edge::ORBIT>();

	std::vector<uint32> dual_histo_vertices(HISTO_SZ, 0);
	map2_dual.foreach_cell([&dual_histo_vertices, &map2_dual](Map2::Vertex v)
	{
		dual_histo_vertices[map2_dual.degree(v)]++;
	});

	std::vector<uint32> dual_histo_faces(HISTO_SZ, 0);
	map2_dual.foreach_cell([&dual_histo_faces,&map2_dual](Map2::Face f)
	{
		dual_histo_faces[map2_dual.codegree(f)]++;
	});


	auto face_of_src = map2_dual.get_attribute<Map2::Face, Map2::Vertex>("FaceOfSrc");
	auto vert_of_src = map2_dual.get_attribute<Map2::Vertex, Map2::Face>("VertexOfSrc");
	auto edge_of_src = map2_dual.get_attribute<Map2::Edge, Map2::Edge>("EdgeOfSrc");

	EXPECT_TRUE(pos_dual.is_valid());
	EXPECT_TRUE(face_of_src.is_valid());
	EXPECT_TRUE(vert_of_src.is_valid());
	EXPECT_TRUE(edge_of_src.is_valid());
	EXPECT_TRUE(map2_dual.check_map_integrity());
	EXPECT_EQ(nbvd, nbf );
	EXPECT_EQ(nbfd, nbv);
	EXPECT_EQ(nbed, nbe);

	EXPECT_EQ(nbvd, face_of_src.size());
	EXPECT_EQ(nbfd, vert_of_src.size());
	EXPECT_EQ(nbed, edge_of_src.size());

	for (uint32 i = 0; i < HISTO_SZ; ++i)
	{
		EXPECT_EQ(histo_vertices[i], dual_histo_faces[i]);
		EXPECT_EQ(histo_faces[i], dual_histo_vertices[i]);
	}
}


TEST(DualTest, salad_bowl)
{
	dual_test("obj/salad_bowl.obj");
}

TEST(DualTest, star_convex)
{
	dual_test("off/star_convex.off");
}

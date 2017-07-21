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
#include <cgogn/geometry/algos/centroid.h>
#include <cgogn/modeling/tiling/square_tore.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using namespace cgogn::numerics;
using Vec3 = Eigen::Vector3d;
using Map2 = cgogn::CMap2;
using Map3 = cgogn::CMap3;

const std::string mesh_path(DEFAULT_MESH_PATH);

void dual_test(const std::string& filename)
{
	Map2 map2;
	cgogn::io::import_surface<Vec3>(map2, mesh_path + filename);

	auto pos = map2.get_attribute<Vec3, Map2::Vertex>("position");
	uint32 nbv = map2.nb_cells<Map2::Vertex::ORBIT>();
	uint32 nbf = map2.nb_cells<Map2::Face::ORBIT>();
	uint32 nbe = map2.nb_cells<Map2::Edge::ORBIT>();

	// compute histo of valences
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
	
	//DUAL

	Map2 map2_dual;

	cgogn::modeling::dual(map2, map2_dual,nullptr,{"position"},
	[&] (Map2::Face f )
	{
		return cgogn::geometry::centroid<Vec3>(map2, f, pos);
	});

	auto pos_dual = map2_dual.get_attribute<Vec3, Map2::Vertex>("position");


	uint32 nbvd = map2_dual.nb_cells<Map2::Vertex::ORBIT>();
	uint32 nbfd = map2_dual.nb_cells<Map2::Face::ORBIT>();
	uint32 nbed = map2_dual.nb_cells<Map2::Edge::ORBIT>();

	// compute histo of valences of dual
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

	// CHECK

	EXPECT_TRUE(pos_dual.is_valid());
	EXPECT_TRUE(map2_dual.check_map_integrity());
	EXPECT_EQ(nbvd, nbf );
	EXPECT_EQ(nbfd, nbv);
	EXPECT_EQ(nbed, nbe);

	for (uint32 i = 0; i < HISTO_SZ; ++i)
	{
		EXPECT_EQ(histo_vertices[i], dual_histo_faces[i]);
		EXPECT_EQ(histo_faces[i], dual_histo_vertices[i]);
	}
}


void dual_test2(const std::string& filename, const std::string& filename2)
{
	Map2 map2;

	Map2::Face f = map2.add_face(12);

	Map2 map_a;
	cgogn::io::import_surface<Vec3>(map_a, mesh_path + filename);

	Map2::DartMarker dm(map2);
	map2.merge(map_a,dm);

	Map2::Face ff = map2.add_face(12);

	Map2 map_b;
	cgogn::io::import_surface<Vec3>(map_b, mesh_path + filename2);
	map2.merge(map_b,dm);

	map2.remove_volume(Map2::Volume(f.dart));
	map2.remove_volume(Map2::Volume(ff.dart));

	// now ...salad_bowl...horse

	auto pos = map2.get_attribute<Vec3, Map2::Vertex>("position");
	uint32 nbv = map2.nb_cells<Map2::Vertex::ORBIT>();
	uint32 nbf = map2.nb_cells<Map2::Face::ORBIT>();
	uint32 nbe = map2.nb_cells<Map2::Edge::ORBIT>();

	// compute histo of valences
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


	// DUAL
	Map2 map2_dual;
	cgogn::modeling::dual(map2, map2_dual,nullptr,{"position"},
	[&] (Map2::Face f )
	{
		return cgogn::geometry::centroid<Vec3>(map2, f, pos);
	});

	auto pos_dual = map2_dual.get_attribute<Vec3, Map2::Vertex>("position");

	uint32 nbvd = map2_dual.nb_cells<Map2::Vertex::ORBIT>();
	uint32 nbfd = map2_dual.nb_cells<Map2::Face::ORBIT>();
	uint32 nbed = map2_dual.nb_cells<Map2::Edge::ORBIT>();

	// compute histo of valences of dual
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

	// CHECK

	EXPECT_TRUE(pos_dual.is_valid());
	EXPECT_TRUE(map2_dual.check_map_integrity());
	EXPECT_EQ(nbvd, nbf );
	EXPECT_EQ(nbfd, nbv);
	EXPECT_EQ(nbed, nbe);

	for (uint32 i = 0; i < HISTO_SZ; ++i)
	{
		EXPECT_EQ(histo_vertices[i], dual_histo_faces[i]);
		EXPECT_EQ(histo_faces[i], dual_histo_vertices[i]);
	}
}


TEST(DualTest, with_boundaries)
{
	Map2 map2;
	auto pos = map2.add_attribute<Vec3, Map2::Vertex>("position");
	cgogn::modeling::SquareTore<Map2> g(map2, 16, 16);
	g.embed_into_tore(pos, 10.0f, 4.0f);

	Map2::Builder mb(map2);

	map2.merge_incident_faces(Map2::Edge(cgogn::Dart(64)));
	map2.merge_incident_faces(Map2::Edge(cgogn::Dart(66)));

	map2.merge_incident_faces(Map2::Edge(cgogn::Dart(136)));
	mb.boundary_mark(Map2::Face(cgogn::Dart(137)));

	uint32 nb_holes = 1;

	for(uint32 j=4; j<7; ++j)
	{
		for(uint32 i=0; i<8; ++i)
		{
			mb.boundary_mark(Map2::Face(cgogn::Dart(i*8+j*128)));
			nb_holes++;
		}
	}

	// now we have a tore with nb_holes holes

	uint32 nbv = map2.nb_cells<Map2::Vertex::ORBIT>();
	uint32 nbf = map2.nb_cells<Map2::Face::ORBIT>();
	uint32 nbe = map2.nb_cells<Map2::Edge::ORBIT>();

	//DUAL
	Map2 map2_dual;
	Map2::CellMarker<Map2::Vertex::ORBIT> cm(map2_dual);
	cgogn::modeling::dual(map2, map2_dual,&cm,{"position"},
	[&] (Map2::Face f )
	{
		return cgogn::geometry::centroid<Vec3>(map2, f, pos);
	});

	auto pos_dual = map2_dual.get_attribute<Vec3, Map2::Vertex>("position");

	uint32 nbvd = map2_dual.nb_cells<Map2::Vertex::ORBIT>();
	uint32 nbfd = map2_dual.nb_cells<Map2::Face::ORBIT>();
	uint32 nbed = map2_dual.nb_cells<Map2::Edge::ORBIT>();

	// CHECK

	EXPECT_TRUE(pos_dual.is_valid());
	EXPECT_TRUE(map2_dual.check_map_integrity());
	EXPECT_EQ(nbvd, nbf + nb_holes );
	EXPECT_EQ(nbfd, nbv);
	EXPECT_EQ(nbed, nbe);

	EXPECT_EQ(cm.nb_marked(), nb_holes);

}



TEST(DualTest, salad_bowl)
{
	dual_test("obj/salad_bowl.obj");
}

TEST(DualTest, star_convex)
{
	dual_test("off/star_convex.off");
}

TEST(DualTest, not_compact)
{
	dual_test2("obj/salad_bowl.obj","off/horse.off");
}

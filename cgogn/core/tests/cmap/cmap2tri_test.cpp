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

#include <cgogn/core/cmap/cmap2_tri.h>

namespace cgogn
{

#define NB_MAX 100

/*!
 * \brief The CMap2TriTest class implements tests on embedded CMap2
 * It contains a CMap2 to which vertex, edge, face and volume attribute
 * are added to enforce the indexation mechanism in cell traversals.
 *
 * Note that pure topological operations have already been tested,
 * in CMap2TopoTest, thus only the indexation mechanism used for the
 * embedding of cells is tested here.
 */
class CMap2TriTest : public ::testing::Test
{
public:

	using MapBuilder = CMap2Tri::Builder;
	using CDart = CMap2Tri::CDart;
	using Vertex = CMap2Tri::Vertex;
	using Edge = CMap2Tri::Edge;
	using Face = CMap2Tri::Face;
	using Volume = CMap2Tri::Volume;

protected:

	CMap2Tri cmap_;

	CMap2TriTest()
	{}

	void embed_map()
	{
		cmap_.add_attribute<int32, CDart>("darts");
		cmap_.add_attribute<int32, Vertex>("vertices");
		cmap_.add_attribute<int32, Edge>("edges");
		cmap_.add_attribute<int32, Face>("faces");
		cmap_.add_attribute<int32, Volume>("volumes");
	}
};

TEST_F(CMap2TriTest,tris)
{
	for (uint32 i=0; i<10; ++i)
	{
		cmap_.add_face(3u);
	}
	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 30u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 30u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 10u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 10u);


	embed_map();

	for (uint32 i=0; i<10; ++i)
	{
		cmap_.add_face(3u);
	}
	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 60u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 60u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 20u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 20u);

}

TEST_F(CMap2TriTest, builder)
{
	MapBuilder builder(cmap_);
	Dart d1 = builder.add_face_topo_fp(3u);
	Dart d2 = builder.add_face_topo_fp(3u);

	builder.phi2_sew(d1,d2);

	builder.close_map();

	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 4u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 5u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 2u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1u);
}


TEST_F(CMap2TriTest, flip)
{
	MapBuilder builder(cmap_);
	Dart d1 = builder.add_face_topo_fp(3u);
	Dart d2 = builder.add_face_topo_fp(3u);
	builder.phi2_sew(d1,d2);
	builder.close_map();

	embed_map();

	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.degree(Vertex(d1)), 3u);
	EXPECT_EQ(cmap_.degree(Vertex(d2)), 3u);
	EXPECT_EQ(cmap_.degree(Vertex(cmap_.phi_1(d1))), 2u);
	EXPECT_EQ(cmap_.degree(Vertex(cmap_.phi_1(d2))), 2u);

	cmap_.flip_edge(Edge(d1));

	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.degree(Vertex(d1)), 3u);
	EXPECT_EQ(cmap_.degree(Vertex(d2)), 3u);
	EXPECT_EQ(cmap_.degree(Vertex(cmap_.phi_1(d1))), 2u);
	EXPECT_EQ(cmap_.degree(Vertex(cmap_.phi_1(d2))), 2u);
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 4u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 5u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 2u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1u);
}


TEST_F(CMap2TriTest, collapse)
{
	MapBuilder builder(cmap_);

	Dart d1 = builder.add_face_topo_fp(3u);
	Dart d2 = builder.add_face_topo_fp(3u);

	builder.phi2_sew(d1,d2);
	builder.close_hole(cmap_.phi1(d1));

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 5u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 9u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 6u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1u);

	embed_map();

	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.degree(Vertex(d1)), 4u);
	EXPECT_EQ(cmap_.degree(Vertex(d2)), 4u);
	EXPECT_EQ(cmap_.degree(Vertex(cmap_.phi_1(d1))), 3u);
	EXPECT_EQ(cmap_.degree(Vertex(cmap_.phi_1(d2))), 3u);


//	Vertex nv = cmap_.collapse_edge(Edge(d1));

//	EXPECT_TRUE(cmap_.check_map_integrity());
//	EXPECT_EQ(cmap_.degree(nv), 4);

//	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 4);
//	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 6);
//	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 4);
//	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1);
}


TEST_F(CMap2TriTest, split_triangle)
{

	embed_map();
	Face f=cmap_.add_face(3u);

	Vertex center = cmap_.split_triangle(f);

	EXPECT_TRUE(cmap_.check_map_integrity());

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 4u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 6u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 3u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1u);
	EXPECT_EQ(cmap_.degree(center), 3u);
}

TEST_F(CMap2TriTest, split_vertex)
{
	MapBuilder builder(cmap_);
	Dart d1 = builder.add_face_topo_fp(3u);
	builder.close_hole(cmap_.phi1(d1));
	embed_map();

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 4u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 6u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 4u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1u);

	EXPECT_TRUE(cmap_.check_map_integrity());

	Dart d2 = cmap_.phi2(cmap_.phi_1(d1));
	Edge e = cmap_.split_vertex(d1,d2);

	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 5u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 9u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 6u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1u);

	auto verts = cmap_.vertices(e);
	EXPECT_EQ(cmap_.degree(verts.first) + cmap_.degree(verts.second) , 7u);
}

TEST_F(CMap2TriTest, cut_edge)
{
	MapBuilder builder(cmap_);

	Dart d1 = builder.add_face_topo_fp(3u);
	Dart d2 = builder.add_face_topo_fp(3u);

	builder.phi2_sew(d1,d2);
	builder.close_map();

	embed_map();

	Vertex nv = cmap_.cut_edge(Edge(d1));

	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.degree(nv), 4u);

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 5u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 8u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 4u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1u);
}


TEST_F(CMap2TriTest, add_tetra)
{
	embed_map();
	Volume vol = cmap_.add_tetra();

	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 4u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 6u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 4u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1u);

	cmap_.foreach_incident_vertex(vol, [&] (Vertex v)
	{
		EXPECT_EQ(cmap_.degree(v), 3u);
	});
}

} // namespace cgogn

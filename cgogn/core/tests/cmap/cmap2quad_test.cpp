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

#include <cgogn/core/cmap/cmap2_quad.h>

namespace cgogn
{

#define NB_MAX 100

/*!
 * \brief The CMap2QuadTest class implements tests on embedded CMap2
 * It contains a CMap2 to which vertex, edge, face and volume attribute
 * are added to enforce the indexation mechanism in cell traversals.
 *
 * Note that pure topological operations have already been tested,
 * in CMap2TopoTest, thus only the indexation mechanism used for the
 * embedding of cells is tested here.
 */
class CMap2QuadTest : public ::testing::Test
{
public:

	using MapBuilder = CMap2Quad::Builder;
	using CDart = CMap2Quad::CDart;
	using Vertex = CMap2Quad::Vertex;
	using Edge = CMap2Quad::Edge;
	using Face = CMap2Quad::Face;
	using Volume = CMap2Quad::Volume;

protected:

	CMap2Quad cmap_;

	CMap2QuadTest()
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

TEST_F(CMap2QuadTest,quads)
{
	for (uint32 i=0; i<10; ++i)
	{
		cmap_.add_face(4);
	}

	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 40u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 40u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 10u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 10u);

	embed_map();

	for (uint32 i=0; i<10; ++i)
	{
		cmap_.add_face(4);
	}

	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 80u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 80u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 20u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 20u);

}

TEST_F(CMap2QuadTest, builder)
{
	MapBuilder builder(cmap_);

	Dart d1 = builder.add_face_topo_fp(4);
	Dart d2 = builder.add_face_topo_fp(4);

	builder.phi2_sew(d1,d2);

	builder.close_map();

	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 6u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 7u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 2u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1u);
}


TEST_F(CMap2QuadTest, add_hexa)
{
	embed_map();
	Volume vol = cmap_.add_hexa();

	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 8u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 12u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 6u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1u);

	cmap_.foreach_incident_vertex(vol, [&] (Vertex v)
	{
		EXPECT_EQ(cmap_.degree(v), 3u);
	});
}


TEST_F(CMap2QuadTest, extrude_quad)
{
	MapBuilder builder(cmap_);
	Dart d1 = builder.add_face_topo_fp(4);
	builder.close_map();
	embed_map();

	EXPECT_TRUE(cmap_.check_map_integrity());

	cmap_.extrude_quad(Face(d1));

	EXPECT_TRUE(cmap_.check_map_integrity());
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 8u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 12u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 5u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1u);
}



} // namespace cgogn

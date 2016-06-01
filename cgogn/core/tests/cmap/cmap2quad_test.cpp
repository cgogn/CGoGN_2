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

#include <cgogn/core/cmap/cmap2_quad_builder.h>

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

	struct MiniMapTraits
	{
		static const uint32 CHUNK_SIZE = 16;
	};

	using testCMap2Quad = CMap2Quad<MiniMapTraits>;
	using MapBuilder = CMap2QuadBuilder_T<MiniMapTraits>;
	using CDart = testCMap2Quad::CDart;
	using Vertex = testCMap2Quad::Vertex;
	using Edge = testCMap2Quad::Edge;
	using Face = testCMap2Quad::Face;
	using Volume = testCMap2Quad::Volume;

protected:

	testCMap2Quad cmap_;
	testCMap2Quad cmap2_;

	CMap2QuadTest()
	{
		cmap_.add_attribute<int32, CDart::ORBIT>("darts");
		cmap_.add_attribute<int32, Vertex::ORBIT>("vertices");
		cmap_.add_attribute<int32, Edge::ORBIT>("edges");
		cmap_.add_attribute<int32, Face::ORBIT>("faces");
		cmap_.add_attribute<int32, Volume::ORBIT>("volumes");
	}
};

TEST_F(CMap2QuadTest,quads)
{
	for (uint32 i=0; i<10; ++i)
	{
		cmap2_.add_face(4);
	}
	EXPECT_EQ(cmap2_.nb_cells<Vertex::ORBIT>(), 40);
	EXPECT_EQ(cmap2_.nb_cells<Edge::ORBIT>(), 40);
	EXPECT_EQ(cmap2_.nb_cells<Face::ORBIT>(), 10);
	EXPECT_EQ(cmap2_.nb_cells<Volume::ORBIT>(), 10);

	for (uint32 i=0; i<10; ++i)
	{
		cmap_.add_face(4);
	}
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 40);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 40);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 10);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 10);

}

TEST_F(CMap2QuadTest, builder)
{
	MapBuilder builder(cmap2_);

	Dart d1 = builder.add_face_topo_parent(4);
	Dart d2 = builder.add_face_topo_parent(4);

	builder.phi2_sew(d1,d2);

	builder.close_map();

	EXPECT_EQ(cmap2_.nb_cells<Vertex::ORBIT>(), 6);
	EXPECT_EQ(cmap2_.nb_cells<Edge::ORBIT>(), 7);
	EXPECT_EQ(cmap2_.nb_cells<Face::ORBIT>(), 2);
	EXPECT_EQ(cmap2_.nb_cells<Volume::ORBIT>(), 1);


}

} // namespace cgogn

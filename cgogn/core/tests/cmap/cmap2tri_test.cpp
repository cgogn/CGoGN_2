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

#include <cgogn/core/cmap/cmap2_tri_builder.h>

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

	struct MiniMapTraits
	{
		static const uint32 CHUNK_SIZE = 16;
	};

	using testCMap2Tri = CMap2Tri<MiniMapTraits>;
	using MapBuilder = CMap2TriBuilder_T<MiniMapTraits>;
	using CDart = testCMap2Tri::CDart;
	using Vertex = testCMap2Tri::Vertex;
	using Edge = testCMap2Tri::Edge;
	using Face = testCMap2Tri::Face;
	using Volume = testCMap2Tri::Volume;

protected:

	testCMap2Tri cmap_;
	testCMap2Tri cmap2_;

	CMap2TriTest()
	{
		cmap_.add_attribute<int32, CDart::ORBIT>("darts");
		cmap_.add_attribute<int32, Vertex::ORBIT>("vertices");
		cmap_.add_attribute<int32, Edge::ORBIT>("edges");
		cmap_.add_attribute<int32, Face::ORBIT>("faces");
		cmap_.add_attribute<int32, Volume::ORBIT>("volumes");
	}
};

TEST_F(CMap2TriTest,tris)
{
	for (uint32 i=0; i<10; ++i)
	{
		cmap2_.add_face(3);
	}
	EXPECT_EQ(cmap2_.nb_cells<Vertex::ORBIT>(), 30);
	EXPECT_EQ(cmap2_.nb_cells<Edge::ORBIT>(), 30);
	EXPECT_EQ(cmap2_.nb_cells<Face::ORBIT>(), 10);
	EXPECT_EQ(cmap2_.nb_cells<Volume::ORBIT>(), 10);

	for (uint32 i=0; i<10; ++i)
	{
		cmap_.add_face(3);
	}
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 30);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 30);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 10);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 10);

}

TEST_F(CMap2TriTest, builder)
{
	MapBuilder builder(cmap2_);

	Dart d1 = builder.add_face_topo_parent(3);
	Dart d2 = builder.add_face_topo_parent(3);

	builder.phi2_sew(d1,d2);

	builder.close_map();

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 4);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 5);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 2);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1);


}

} // namespace cgogn

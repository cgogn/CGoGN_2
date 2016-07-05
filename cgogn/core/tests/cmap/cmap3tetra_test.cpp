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

#include <cgogn/core/cmap/cmap3_tetra.h>

namespace cgogn
{

/**
 * \brief The CMap3TetraTest class implements tests on embedded CMap3Tetra
 * It contains a CMap3Tetra to which vertex, edge, face and volume attribute
 * are added to enforce the indexation mechanism in cell traversals.
 *
 */
class CMap3TetraTest : public ::testing::Test
{
public:

	struct MiniMapTraits
	{
		static const uint32 CHUNK_SIZE = 16;
	};

	using testCMap3 = CMap3Tetra<MiniMapTraits>;
	using MapBuilder = testCMap3::Builder;
	using CDart = testCMap3::CDart;
	using Vertex2 = testCMap3::Vertex2;
	using Vertex = testCMap3::Vertex;
	using Edge2 = testCMap3::Edge2;
	using Edge = testCMap3::Edge;
	using Face2 = testCMap3::Face2;
	using Face = testCMap3::Face;
	using Volume = testCMap3::Volume;
	using ConnectedComponent = testCMap3::ConnectedComponent;

protected:

	testCMap3 cmap_;

	/**
	 * \brief A vector of darts on which the methods are tested.
	 */
//	std::vector<Dart> darts_;

	CMap3TetraTest()
	{
//		darts_.reserve(NB_MAX);
//		std::srand(uint32(std::time(0)));

//		cmap_.add_attribute<int32, CDart::ORBIT>("darts");
//		cmap_.add_attribute<int32, Vertex2::ORBIT>("vertices2");
//		cmap_.add_attribute<int32, Vertex::ORBIT>("vertices");
//		cmap_.add_attribute<int32, Edge2::ORBIT>("edges2");
//		cmap_.add_attribute<int32, Edge::ORBIT>("edges");
//		cmap_.add_attribute<int32, Face2::ORBIT>("faces2");
//		cmap_.add_attribute<int32, Face::ORBIT>("faces");
//		cmap_.add_attribute<int32, Volume::ORBIT>("volumes");
	}
};

/**
 * @brief Cutting edges preserves the cell indexation
 */
TEST_F(CMap3TetraTest, topo_1)
{
	MapBuilder mbuild(cmap_);

	Dart p1 = mbuild.add_pyramid_topo(3u);

	mbuild.close_map();

	EXPECT_TRUE(cmap_.check_map_integrity());

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 4);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 6);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 4);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1);
	EXPECT_EQ(cmap_.nb_cells<ConnectedComponent::ORBIT>(), 1);
}

/**
 * @brief Cutting edges preserves the cell indexation
 */
TEST_F(CMap3TetraTest, topo_4)
{
	MapBuilder mbuild(cmap_);

	Dart p1 = mbuild.add_pyramid_topo(3u);
	Dart p2 = mbuild.add_pyramid_topo(3u);
	mbuild.sew_volumes(p1, p2);

	Dart p3 = mbuild.add_pyramid_topo(3u);
	Dart p4 = mbuild.add_pyramid_topo(3u);
	mbuild.sew_volumes(p3, p4);

	// Close the map (remove remaining boundary)
	mbuild.close_map();

	EXPECT_TRUE(cmap_.check_map_integrity());

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 10);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 18);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 14);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 4);
	EXPECT_EQ(cmap_.nb_cells<ConnectedComponent::ORBIT>(), 2);
}


/**
 * @brief Cutting edges preserves the cell indexation
 */
TEST_F(CMap3TetraTest, embedded)
{
	cmap_.add_attribute<int32, CDart::ORBIT>("darts");
	cmap_.add_attribute<int32, Vertex2::ORBIT>("vertices2");
	cmap_.add_attribute<int32, Vertex::ORBIT>("vertices");
	cmap_.add_attribute<int32, Edge2::ORBIT>("edges2");
	cmap_.add_attribute<int32, Edge::ORBIT>("edges");
	cmap_.add_attribute<int32, Face2::ORBIT>("faces2");
	cmap_.add_attribute<int32, Face::ORBIT>("faces");
	cmap_.add_attribute<int32, Volume::ORBIT>("volumes");

	MapBuilder mbuild(cmap_);

	Dart p1 = mbuild.add_pyramid_topo(3u);
	Dart p2 = mbuild.add_pyramid_topo(3u);
	mbuild.sew_volumes(p1, p2);

	Dart p3 = mbuild.add_pyramid_topo(3u);
	Dart p4 = mbuild.add_pyramid_topo(3u);
	Dart p5 = mbuild.add_pyramid_topo(3u);
	mbuild.sew_volumes(p3, cmap_.phi2(p4));
	mbuild.sew_volumes(p4, cmap_.phi2(p5));
	mbuild.sew_volumes(p5, cmap_.phi2(p3));

	// Close the map (remove remaining boundary)
	cmap_.foreach_dart([&] (Dart d)
	{
		if (cmap_.phi3(d) == d) mbuild.close_hole_topo(d,true);
	});

	// Embed the map
	cmap_.foreach_dart([&] (Dart d)
	{
		if (!cmap_.is_boundary(d))
			mbuild.new_orbit_embedding(CDart(d));
	});
	cmap_.foreach_cell<FORCE_DART_MARKING>([&] (Vertex2 v)
	{
		mbuild.new_orbit_embedding(v);
	});
	cmap_.foreach_cell<FORCE_DART_MARKING>([&] (Vertex v)
	{
		mbuild.new_orbit_embedding(v);
	});
	cmap_.foreach_cell<FORCE_DART_MARKING>([&] (Edge2 e)
	{
		mbuild.new_orbit_embedding(e);
	});
	cmap_.foreach_cell<FORCE_DART_MARKING>([&] (Edge e)
	{
		mbuild.new_orbit_embedding(e);
	});
	cmap_.foreach_cell<FORCE_DART_MARKING>([&] (Face2 f)
	{
		mbuild.new_orbit_embedding(f);
	});
	cmap_.foreach_cell<FORCE_DART_MARKING>([&] (Face f)
	{
		mbuild.new_orbit_embedding(f);
	});
	cmap_.foreach_cell<FORCE_DART_MARKING>([&] (Volume w)
	{
		mbuild.new_orbit_embedding(w);
	});

	EXPECT_TRUE(cmap_.check_map_integrity());

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 5+5);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 10+9);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 7+9);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 2+3);
	EXPECT_EQ(cmap_.nb_cells<ConnectedComponent::ORBIT>(), 2);

	int nb=0;
	cmap_.foreach_incident_volume(Edge(p3),[&] (Volume) { ++nb; });
	EXPECT_EQ(nb,3);
	nb=0;
	cmap_.foreach_incident_face(Edge(p3),[&] (Face){ ++nb; });
	EXPECT_EQ(nb,3);

	nb=0;
	cmap_.foreach_incident_face(Volume(p3),[&] (Face){ ++nb; });
	EXPECT_EQ(nb,4);
	nb=0;
	cmap_.foreach_incident_edge(Volume(p5),[&] (Edge){ ++nb; });
	EXPECT_EQ(nb,6);
	nb=0;
	cmap_.foreach_incident_vertex(Volume(p4),[&] (Vertex){ ++nb; });
	EXPECT_EQ(nb,4);


	nb=0;
	cmap_.foreach_adjacent_edge_through_vertex(Edge(p3),[&] (Edge){ ++nb; });
	EXPECT_EQ(nb,6);
	nb=0;
	cmap_.foreach_adjacent_edge_through_volume(Edge(p3),[&] (Edge){ ++nb; });
	EXPECT_EQ(nb,9);

	Dart p6 = cmap_.phi<211>(p3);
	nb=0;
	cmap_.foreach_incident_face(Vertex(p6),[&] (Face){ ++nb; });
	EXPECT_EQ(nb,5);
	nb=0;
	cmap_.foreach_incident_volume(Vertex(p6),[&] (Volume){ ++nb; });
	EXPECT_EQ(nb,2);

	nb=0;
	cmap_.foreach_adjacent_vertex_through_edge(Vertex(p6),[&] (Vertex){ ++nb; });
	EXPECT_EQ(nb,4);
	nb=0;
	cmap_.foreach_adjacent_vertex_through_face(Vertex(p6),[&] (Vertex){ ++nb; });
	EXPECT_EQ(nb,4);
	nb=0;
	cmap_.foreach_adjacent_vertex_through_volume(Vertex(p6),[&] (Vertex){ ++nb; });
	EXPECT_EQ(nb,4);

}


} // namespace cgogn

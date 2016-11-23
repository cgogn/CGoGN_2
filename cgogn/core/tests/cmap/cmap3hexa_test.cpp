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

#include <cgogn/core/cmap/cmap3_hexa.h>

namespace cgogn
{

/**
 * \brief The CMap3HexaTest class implements tests on embedded CMap3HexaHexa
 * It contains a CMap3HexaHexa to which vertex, edge, face and volume attribute
 * are added to enforce the indexation mechanism in cell traversals.
 *
 */
class CMap3HexaTest : public ::testing::Test
{
public:

	using MapBuilder = CMap3Hexa::Builder;
	using CDart = CMap3Hexa::CDart;
	using Vertex2 = CMap3Hexa::Vertex2;
	using Vertex = CMap3Hexa::Vertex;
	using Edge2 = CMap3Hexa::Edge2;
	using Edge = CMap3Hexa::Edge;
	using Face2 = CMap3Hexa::Face2;
	using Face = CMap3Hexa::Face;
	using Volume = CMap3Hexa::Volume;
	using ConnectedComponent = CMap3Hexa::ConnectedComponent;

protected:

	CMap3Hexa cmap_;

	/**
	 * \brief A vector of darts on which the methods are tested.
	 */
//	std::vector<Dart> darts_;

	CMap3HexaTest()
	{}
};

/**
 * @brief Cutting edges preserves the cell indexation
 */
TEST_F(CMap3HexaTest, topo_1)
{
	MapBuilder mbuild(cmap_);

	mbuild.add_prism_topo_fp(4u);

	mbuild.close_map();

	EXPECT_TRUE(cmap_.check_map_integrity());

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 8u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 12u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 6u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 1u);
	EXPECT_EQ(cmap_.nb_cells<ConnectedComponent::ORBIT>(), 1u);
}

/**
 * @brief Cutting edges preserves the cell indexation
 */
TEST_F(CMap3HexaTest, topo_4)
{
	MapBuilder mbuild(cmap_);

	Dart p1 = mbuild.add_prism_topo_fp(4u);
	Dart p2 = mbuild.add_prism_topo_fp(4u);
	mbuild.sew_volumes_fp(p1, p2);

	Dart p3 = mbuild.add_prism_topo_fp(4u);
	Dart p4 = mbuild.add_prism_topo_fp(4u);
	mbuild.sew_volumes_fp(p3, p4);

	// Close the map (remove remaining boundary)
	mbuild.close_map();

	EXPECT_TRUE(cmap_.check_map_integrity());

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 24u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 40u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 22u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 4u);
	EXPECT_EQ(cmap_.nb_cells<ConnectedComponent::ORBIT>(), 2u);
}

/**
 * @brief Cutting edges preserves the cell indexation
 */
TEST_F(CMap3HexaTest, embedded)
{
	MapBuilder mbuild(cmap_);

	Dart p1 = mbuild.add_prism_topo_fp(4u);
	Dart p2 = mbuild.add_prism_topo_fp(4u);
	mbuild.sew_volumes_fp(p1, p2);

	Dart p3 = mbuild.add_prism_topo_fp(4u);
	Dart p4 = mbuild.add_prism_topo_fp(4u);
	Dart p5 = mbuild.add_prism_topo_fp(4u);
	mbuild.sew_volumes_fp(p3, cmap_.phi2(p4));
	mbuild.sew_volumes_fp(p4, cmap_.phi2(p5));
	mbuild.sew_volumes_fp(p5, cmap_.phi2(p3));

	mbuild.close_map();

	cmap_.add_attribute<int32, CDart>("darts");
	cmap_.add_attribute<int32, Vertex2>("vertices2");
	cmap_.add_attribute<int32, Vertex>("vertices");
	cmap_.add_attribute<int32, Edge2>("edges2");
	cmap_.add_attribute<int32, Edge>("edges");
	cmap_.add_attribute<int32, Face2>("faces2");
	cmap_.add_attribute<int32, Face>("faces");
	cmap_.add_attribute<int32, Volume>("volumes");

	EXPECT_TRUE(cmap_.check_map_integrity());

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 12u+14u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 20u+25u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 11u+15u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 2u+3u);
	EXPECT_EQ(cmap_.nb_cells<ConnectedComponent::ORBIT>(), 2u);

	int nb=0;
	cmap_.foreach_incident_volume(Edge(p3),[&] (Volume) { ++nb; });
	EXPECT_EQ(nb,3);
	nb=0;
	cmap_.foreach_incident_face(Edge(p3),[&] (Face){ ++nb; });
	EXPECT_EQ(nb,3);

	nb=0;
	cmap_.foreach_incident_face(Volume(p3),[&] (Face){ ++nb; });
	EXPECT_EQ(nb,6);
	nb=0;
	cmap_.foreach_incident_edge(Volume(p5),[&] (Edge){ ++nb; });
	EXPECT_EQ(nb,12);
	nb=0;
	cmap_.foreach_incident_vertex(Volume(p4),[&] (Vertex){ ++nb; });
	EXPECT_EQ(nb,8);


	nb=0;
	cmap_.foreach_adjacent_edge_through_vertex(Edge(p3),[&] (Edge){ ++nb; });
	EXPECT_EQ(nb,6);
	nb=0;
	cmap_.foreach_adjacent_edge_through_volume(Edge(p3),[&] (Edge){ ++nb; });
	EXPECT_EQ(nb,24);

	Dart p6 = cmap_.phi<211>(p3);
	nb=0;
	cmap_.foreach_incident_face(Vertex(p6),[&] (Face){ ++nb; });
	EXPECT_EQ(nb,5);
	nb=0;
	cmap_.foreach_incident_face(Edge(p6),[&] (Face){ ++nb; });
	EXPECT_EQ(nb,3);
	nb=0;
	cmap_.foreach_incident_volume(Vertex(p6),[&] (Volume){ ++nb; });
	EXPECT_EQ(nb,2);



	p6 = cmap_.phi<211211>(p3);
	nb=0;
	cmap_.foreach_incident_face(Vertex(p6),[&] (Face){ ++nb; });
	EXPECT_EQ(nb,3);
	nb=0;
	cmap_.foreach_incident_volume(Vertex(p6),[&] (Volume){ ++nb; });
	EXPECT_EQ(nb,1);
	nb=0;
	cmap_.foreach_incident_face(Edge(p6),[&] (Face){ ++nb; });
	EXPECT_EQ(nb,2);

	nb=0;
	cmap_.foreach_adjacent_vertex_through_edge(Vertex(p6),[&] (Vertex){ ++nb; });
	EXPECT_EQ(nb,3);
	nb=0;
	cmap_.foreach_adjacent_vertex_through_face(Vertex(p6),[&] (Vertex){ ++nb; });
	EXPECT_EQ(nb,6);
	nb=0;
	cmap_.foreach_adjacent_vertex_through_volume(Vertex(p6),[&] (Vertex){ ++nb; });
	EXPECT_EQ(nb,7);

}


} // namespace cgogn

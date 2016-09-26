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

#include <cgogn/core/cmap/cmap3_builder.h>

namespace cgogn
{

#define NB_MAX 100

/**
 * \brief The CMap3Test class implements tests on embedded CMap3
 * It contains a CMap3 to which vertex, edge, face and volume attribute
 * are added to enforce the indexation mechanism in cell traversals.
 *
 * Note that pure topological operations have already been tested,
 * in CMap3TopoTest, thus only the indexation mechanism used for the
 * embedding of cells is tested here.
 */
class CMap3Test : public ::testing::Test
{
public:

	struct MiniMapTraits
	{
		static const uint32 CHUNK_SIZE = 16;
	};

	using testCMap3 = CMap3<MiniMapTraits>;
	using MapBuilder = CMap3Builder_T<MiniMapTraits>;
	using CDart = testCMap3::CDart;
	using Vertex2 = testCMap3::Vertex2;
	using Vertex = testCMap3::Vertex;
	using Edge2 = testCMap3::Edge2;
	using Edge = testCMap3::Edge;
	using Face2 = testCMap3::Face2;
	using Face = testCMap3::Face;
	using Volume = testCMap3::Volume;

protected:

	testCMap3 cmap_;

	/**
	 * \brief A vector of darts on which the methods are tested.
	 */
	std::vector<Dart> darts_;

	CMap3Test()
	{
		darts_.reserve(NB_MAX);
		std::srand(uint32(std::time(0)));

		cmap_.add_attribute<int32, CDart::ORBIT>("darts");
		cmap_.add_attribute<int32, Vertex2::ORBIT>("vertices2");
		cmap_.add_attribute<int32, Vertex::ORBIT>("vertices");
		cmap_.add_attribute<int32, Edge2::ORBIT>("edges2");
		cmap_.add_attribute<int32, Edge::ORBIT>("edges");
		cmap_.add_attribute<int32, Face2::ORBIT>("faces2");
		cmap_.add_attribute<int32, Face::ORBIT>("faces");
		cmap_.add_attribute<int32, Volume::ORBIT>("volumes");
	}
};

/**
 * @brief Cutting edges preserves the cell indexation
 */
TEST_F(CMap3Test, cut_edge)
{
	MapBuilder mbuild(cmap_);

	Dart p1 = mbuild.add_prism_topo(3u);
	Dart p2 = mbuild.add_prism_topo(3u);
	mbuild.sew_volumes(p1, p2);

	Dart p3 = mbuild.add_pyramid_topo(4u);
	Dart p4 = mbuild.add_pyramid_topo(4u);
	mbuild.sew_volumes(p3, p4);

	// Close the map (remove remaining boundary)
	cmap_.foreach_dart([&] (Dart d)
	{
		if (cmap_.phi3(d) == d) mbuild.close_hole_topo(d);
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

	cmap_.cut_edge(Edge(p1));
	cmap_.cut_edge(Edge(cmap_.phi1(p1)));
	cmap_.cut_edge(Edge(cmap_.phi<21>(p1)));

	cmap_.cut_edge(Edge(p3));
	cmap_.cut_edge(Edge(cmap_.phi1(p3)));
	cmap_.cut_edge(Edge(cmap_.phi<21>(p3)));

	EXPECT_TRUE(cmap_.check_map_integrity());
}

/**
 * @brief Cutting faces preserves the cell indexation
 */
TEST_F(CMap3Test, cut_face)
{
	MapBuilder mbuild(cmap_);

	Dart p1 = mbuild.add_prism_topo(3u);
	Dart p2 = mbuild.add_prism_topo(3u);
	mbuild.sew_volumes(p1, p2);

	Dart p3 = mbuild.add_pyramid_topo(4u);
	Dart p4 = mbuild.add_pyramid_topo(4u);
	mbuild.sew_volumes(p3, p4);

	// Close the map (remove remaining boundary)
	cmap_.foreach_dart([&] (Dart d)
	{
		if (cmap_.phi3(d) == d) mbuild.close_hole_topo(d);
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

	cmap_.cut_face(cmap_.phi2(p1), cmap_.phi<211>(p1));
	cmap_.cut_face(p3, cmap_.phi<11>(p3));

	EXPECT_TRUE(cmap_.check_map_integrity());
}

/**
 * @brief Cutting volumes preserves the cell indexation
 */
TEST_F(CMap3Test, cut_volume)
{
	MapBuilder mbuild(cmap_);

	Dart p1 = mbuild.add_prism_topo(6u);

	// Close the map (remove remaining boundary)
	cmap_.foreach_dart([&] (Dart d)
	{
		if (cmap_.phi3(d) == d) mbuild.close_hole_topo(d);
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

	std::vector<Dart> path;
	Dart d = p1; path.push_back(d);
	d = cmap_.phi<121>(d); path.push_back(d);
	d = cmap_.phi<121>(d); path.push_back(d);
	d = cmap_.phi1(d); path.push_back(d);
	d = cmap_.phi<121>(d); path.push_back(d);
	d = cmap_.phi<121>(d); path.push_back(d);

	cmap_.cut_volume(path);

	EXPECT_TRUE(cmap_.check_map_integrity());
}

/**
 * @brief Merging volumes incident to a face preserves the cell indexation
 */
TEST_F(CMap3Test, merge_incident_volumes)
{
	MapBuilder mbuild(cmap_);

	Dart p1 = mbuild.add_prism_topo(3u);
	Dart p2 = mbuild.add_prism_topo(3u);
	mbuild.sew_volumes(p1, p2);

	Dart p3 = mbuild.add_pyramid_topo(4u);
	Dart p4 = mbuild.add_pyramid_topo(4u);
	mbuild.sew_volumes(p3, p4);

	// Close the map (remove remaining boundary)
	cmap_.foreach_dart([&] (Dart d)
	{
		if (cmap_.phi3(d) == d) mbuild.close_hole_topo(d);
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

	cmap_.merge_incident_volumes(Face(p1));
	cmap_.merge_incident_volumes(Face(p3));

	EXPECT_TRUE(cmap_.check_map_integrity());
}

#undef NB_MAX

} // namespace cgogn

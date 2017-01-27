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

#include <cgogn/core/cmap/cmap3.h>

namespace cgogn
{

#define NB_MAX 1000

/**
 * \brief The CMap3TopoTest class implements topological tests on CMap3
 * It derives from CMap3 to allow the test of protected methods
 *
 * Note that these tests, check that the topological operators perform as wanted
 * but do neither tests the containers (refs_, used_, etc.) or the iterators.
 * These last tests are implemented in another test suite.
 */
class CMap3TopoTest : public CMap3, public ::testing::Test
{
public:

	using Inherit = CMap3;

	using Vertex2 = Inherit::Vertex2;
	using Vertex = Inherit::Vertex;
	using Edge2 = Inherit::Edge2;
	using Edge = Inherit::Edge;
	using Face2 = Inherit::Face2;
	using Face = Inherit::Face;
	using Volume = Inherit::Volume;

	using VertexMarker = Inherit::CellMarker<Vertex::ORBIT>;

protected:

	/**
	 * \brief A vector of darts on which the methods are tested.
	 */
	std::vector<Dart> darts_;

	/**
	 * \brief Generate a random set of faces.
	*/
	CMap3TopoTest()
	{
		darts_.reserve(NB_MAX);
		std::srand(static_cast<uint32>(std::time(0)));
	}

	/**
	 * \brief Tests if the open vertex of d contains a specified dart e.
	 * The method supposes that the given dart d is the first dart
	 * of the open PHI21 orbit (i.e. phi2(d) == d)
	 */
	bool same_open_vertex(Dart d, Dart e)
	{
		cgogn_assert(phi2(d) == d);
		Dart it = d;
		Dart it1 = phi_1(it);

		while (it != e && phi2(it1) != it1)
		{
			it = phi2(it1);
			it1 = phi_1(it);
		}
		if (it == e) return true;
		return false;
	}

	/**
	 * \brief Tests if the volume of d contains a specified dart e.
	 * The method does not exploit the indexing information
	 */
	bool same_volume(Dart d, Dart e)
	{
		bool result = false;

		foreach_dart_of_orbit(Volume(d), [&](Dart vit) -> bool
		{
			if (vit == e) result = true;
			return !result;
		});

		return result;
	}

	/**
	 * \brief Embed an open vertex d on a new attribute.
	 * The method supposes that the given dart d is the first dart
	 * of the open PHI21 orbit (i.e. phi2(d) == d)
	 */
	void new_open_vertex_embedding(Dart d)
	{
		cgogn_assert(phi2(d) == d);
		const uint32 emb = add_attribute_element<Vertex::ORBIT>();

		Dart it = d;
		Dart it1 = phi_1(it);

		set_embedding<Vertex>(it, emb);
		while (phi2(it1) != it1)
		{
			it = phi2(it1);
			it1 = phi_1(it);
			set_embedding<Vertex>(it, emb);
		}
	}

	/**
	 * \brief Generate a set of closed surfaces.
	 */
	void add_closed_surfaces()
	{
		darts_.clear();

		// Generate NB_MAX random 2-surfaces (without boundary)
		for (uint32 i = 0u; i < NB_MAX; ++i)
		{
			uint32 n = 3u + std::rand() % 10;
			uint32 p = std::rand() % 2;
			switch (p)
			{
				case 0:
					darts_.push_back(add_pyramid_topo_fp(n));
					break;
				case 1:
					darts_.push_back(add_prism_topo_fp(n));
					break;
				default:
					break;
			}
		}

		// Close the map
		close_map();
	}
};

/**
 * \brief The random generated maps used in the tests are sound.
 */
TEST_F(CMap3TopoTest, random_map_generators)
{
	EXPECT_EQ(nb_darts(), 0u);
	add_closed_surfaces();
	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Test attribute management
 *
 */
TEST_F(CMap3TopoTest, add_attribute)
{
	add_closed_surfaces();

	add_attribute<int32, CDart>("darts");
	EXPECT_TRUE(check_map_integrity());

	add_attribute<int32, Vertex2>("vertices2");
	EXPECT_TRUE(check_map_integrity());

	add_attribute<int32, Vertex>("vertices");
	EXPECT_TRUE(check_map_integrity());

	add_attribute<int32, Edge2>("edges2");
	EXPECT_TRUE(check_map_integrity());

	add_attribute<int32, Edge>("edges");
	EXPECT_TRUE(check_map_integrity());

	add_attribute<int32, Face2>("faces2");
	EXPECT_TRUE(check_map_integrity());

	add_attribute<int32, Face>("faces");
	EXPECT_TRUE(check_map_integrity());

	add_attribute<int32, Volume>("Volumes");
	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Sewing and unsewing darts correctly changes the topological relations.
 * The test performs NB_MAX sewing and unsewing on randomly chosen dart of darts_.
 * The map integrity is not preserved (this test creates fixed points for PHI3).
 */
TEST_F(CMap3TopoTest, phi3_sew_unsew)
{
	add_closed_surfaces();

	for (uint32 i = 0u; i < NB_MAX; ++i)
	{
		Dart d0 = darts_[std::rand() % NB_MAX];
		Dart d3 = phi3(d0);
		phi3_unsew(d0);
		EXPECT_TRUE(phi3(d0) == d0);
		EXPECT_TRUE(phi3(d3) == d3);
		Dart e0 = d0;
		while (e0 == d0) e0 = darts_[std::rand() % NB_MAX];
		phi3_unsew(e0);

		phi3_sew(d0, e0);
		EXPECT_TRUE(phi3(d0) == e0);
		EXPECT_TRUE(phi3(e0) == d0);
	}
}

/**
 * \brief Cutting an edge increases the size of both incident faces and adds a vertex of degree 2.
 * The number of generated cells is correct and the map integrity is preserved.
 * The test performs NB_MAX edge cutting on edges of randomly generated volumes.
 */
TEST_F(CMap3TopoTest, cut_edge_topo)
{
	add_closed_surfaces();

	uint32 count_vertices = nb_cells<Vertex::ORBIT>();
	uint32 count_edges = nb_cells<Edge::ORBIT>();
	uint32 count_faces = nb_cells<Face::ORBIT>();
	uint32 count_volumes = nb_cells<Volume::ORBIT>();

	for (Dart d : darts_)
	{
		uint32 k1 = codegree(Face(d));
		uint32 k2 = codegree(Face(phi2(d)));
		cut_edge_topo(d);
		if (same_cell(Face(d), Face(phi2(d))))
		{
			EXPECT_EQ(codegree(Face(d)), k1 + 2u);
		}
		else
		{
			EXPECT_EQ(codegree(Face(d)), k1 + 1u);
			EXPECT_EQ(codegree(Face(phi2(d))), k2 + 1u);
		}
	}

	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count_vertices + NB_MAX);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), count_edges + NB_MAX);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), count_faces);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), count_volumes);
	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Flipping an edge decreases by 1 the degree of original incident vertices and increases
 * by 1 the degree of the new incident vertices
 * The codegree of the 2 incident faces is not modified
 * The number of generated cells is correct and the map integrity is preserved.
 * The test performs NB_MAX edge flips on edges of randomly generated volumes.
 */
TEST_F(CMap3TopoTest, flip_edge_topo)
{
	add_closed_surfaces();

	uint32 count_vertices = nb_cells<Vertex::ORBIT>();
	uint32 count_edges = nb_cells<Edge::ORBIT>();
	uint32 count_faces = nb_cells<Face::ORBIT>();
	uint32 count_volumes = nb_cells<Volume::ORBIT>();

	for (Dart d : darts_)
	{
		uint32 fk1 = codegree(Face(d));
		uint32 fk2 = codegree(Face(phi2(d)));
		uint32 vk1 = degree(Vertex(d));
		uint32 vk2 = degree(Vertex(phi2(d)));
		uint32 vk1n = degree(Vertex(phi<211>(d)));
		uint32 vk2n = degree(Vertex(phi<11>(d)));
		if (flip_edge_topo(d))
		{
			EXPECT_EQ(codegree(Face(d)), fk1);
			EXPECT_EQ(codegree(Face(phi2(d))), fk2);
			EXPECT_EQ(degree(Vertex(phi_1(d))), vk1 - 1u);
			EXPECT_EQ(degree(Vertex(phi_1(phi2(d)))), vk2 - 1u);
			EXPECT_EQ(degree(Vertex(d)), vk1n + 1u);
			EXPECT_EQ(degree(Vertex(phi2(d))), vk2n + 1u);
		}
	}

	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), count_edges);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), count_faces);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), count_volumes);
	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Flipping back an edge decreases by 1 the degree of original incident vertices and increases
 * by 1 the degree of the new incident vertices
 * The codegree of the 2 incident faces is not modified
 * The number of generated cells is correct and the map integrity is preserved.
 * The test performs NB_MAX edge back flips on edges of randomly generated volumes.
 */
TEST_F(CMap3TopoTest, flip_back_edge_topo)
{
	add_closed_surfaces();

	uint32 count_vertices = nb_cells<Vertex::ORBIT>();
	uint32 count_edges = nb_cells<Edge::ORBIT>();
	uint32 count_faces = nb_cells<Face::ORBIT>();
	uint32 count_volumes = nb_cells<Volume::ORBIT>();

	for (Dart d : darts_)
	{
		uint32 fk1 = codegree(Face(d));
		uint32 fk2 = codegree(Face(phi2(d)));
		uint32 vk1 = degree(Vertex(d));
		uint32 vk2 = degree(Vertex(phi2(d)));
		uint32 vk1n = degree(Vertex(phi_1(d)));
		uint32 vk2n = degree(Vertex(phi_1(phi2(d))));
		if (flip_back_edge_topo(d))
		{
			EXPECT_EQ(codegree(Face(d)), fk1);
			EXPECT_EQ(codegree(Face(phi2(d))), fk2);
			EXPECT_EQ(degree(Vertex(phi<211>(d))), vk1 - 1u);
			EXPECT_EQ(degree(Vertex(phi<11>(d))), vk2 - 1u);
			EXPECT_EQ(degree(Vertex(d)), vk1n + 1u);
			EXPECT_EQ(degree(Vertex(phi2(d))), vk2n + 1u);
		}
	}

	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), count_edges);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), count_faces);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), count_volumes);
	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Cutting a face adds an edge and replaces a face of codegree K,
 * with two subfaces whose codegrees K1 and K2 verify K1+K2 = K+2.
 * The test performs NB_MAX face cuts between vertices of a randomly generated surface.
 * The number of generated cells is correct and the map integrity is preserved.
 */
TEST_F(CMap3TopoTest, cut_face_topo)
{
	add_closed_surfaces();

	uint32 count_vertices = nb_cells<Vertex::ORBIT>();
	uint32 count_edges = nb_cells<Edge::ORBIT>();
	uint32 count_faces = nb_cells<Face::ORBIT>();
	uint32 count_volumes = nb_cells<Volume::ORBIT>();

	for (Dart d : darts_)
	{
		uint32 k = codegree(Face(d));
		if (k > 1u)
		{
			Dart e = d; // find a second dart in the face of d (distinct from d)
			uint32 i = std::rand() % 10u;
			while (i-- > 0u) e = phi1(e);
			if (e == d) e = phi1(e);

			cut_face_topo(d, e);
			++count_edges;
			++count_faces;
			EXPECT_EQ(codegree(Face(d)) + codegree(Face(e)), k + 2);
		}
	}

	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), count_edges);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), count_faces);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), count_volumes);
	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Merging the faces incident to an edge removes an edge and a face.
 * The codegree of the resulting face is K1 + K2 - 2 (K1 and K2 being the codegrees fo the original faces)
 * The number of generated cells is correct and the map integrity is preserved.
 */
TEST_F(CMap3TopoTest, merge_incident_faces)
{
	add_closed_surfaces();

	uint32 count_vertices = nb_cells<Vertex::ORBIT>();
	uint32 count_edges = nb_cells<Edge::ORBIT>();
	uint32 count_faces = nb_cells<Face::ORBIT>();
	uint32 count_volumes = nb_cells<Volume::ORBIT>();

	for (Dart d : darts_)
	{
		Dart d1 = phi1(d);
		uint32 k1 = codegree(Face(d));
		uint32 k2 = codegree(Face(phi2(d)));
		if (merge_incident_faces_topo(d))
		{
			--count_edges;
			--count_faces;
			EXPECT_EQ(codegree(Face(d1)), k1 + k2 - 2);
		}
	}

	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), count_edges);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), count_faces);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), count_volumes);
	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Merging the volumes incident to an edge removes an edge, K faces and K-1 volumes (K being the degree of the edge).
 * The codegree of the resulting volume is the sum of the codgrees of the original volumes minus 2*K
 * The number of generated cells is correct and the map integrity is preserved.
 */
TEST_F(CMap3TopoTest, merge_incident_volumes_of_edge_topo)
{
	Dart p1 = add_prism_topo_fp(6u);
	Dart p2 = add_prism_topo_fp(6u);
	sew_volumes_fp(phi2(p1), phi2(p2));
	sew_volumes_fp(phi2(phi_1(p1)), phi2(phi1(p2)));
	Dart e1 = phi<21>(p1);

	Dart c1 = add_prism_topo_fp(4u);
	Dart c2 = add_prism_topo_fp(4u);
	Dart c3 = add_prism_topo_fp(4u);
	sew_volumes_fp(phi<12>(c1), phi2(c2));
	sew_volumes_fp(phi<12>(c3), phi2(c1));
	sew_volumes_fp(phi<12>(c2), phi2(c3));
	Dart e2 = phi_1(phi2(c1));

	close_map();

	uint32 count_vertices = nb_cells<Vertex::ORBIT>();
	uint32 count_edges = nb_cells<Edge::ORBIT>();
	uint32 count_faces = nb_cells<Face::ORBIT>();
	uint32 count_volumes = nb_cells<Volume::ORBIT>();

	uint32 ek_1 = degree(Edge(e1));
	uint32 vk1_1 = codegree(Volume(e1));
	uint32 vk2_1 = codegree(Volume(phi3(e1)));
	merge_incident_volumes_of_edge_topo(e1);
	--count_edges;
	count_faces -= ek_1;
	count_volumes -= ek_1 - 1u;
	EXPECT_EQ(codegree(Volume(p1)), vk1_1 + vk2_1 - 2u*ek_1);

	uint32 ek_2 = degree(Edge(e2));
	uint32 vk_2 = 0;
	foreach_incident_volume(Edge(e2), [&] (Volume v)
	{
		vk_2 += codegree(v);
	});
	merge_incident_volumes_of_edge_topo(e2);
	--count_edges;
	count_faces -= ek_2;
	count_volumes -= ek_2 - 1u;
	EXPECT_EQ(codegree(Volume(c1)), vk_2 - 2u*ek_2);

	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), count_edges);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), count_faces);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), count_volumes);
	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Merging the volumes incident to a face removes a face and a volume.
 * The codegree of the resulting volume is K1 + K2 - 2 (K1 and K2 being the codegrees of the original volumes)
 * The number of generated cells is correct and the map integrity is preserved.
 */
TEST_F(CMap3TopoTest, merge_incident_volumes_of_face_topo)
{
	Dart p1 = add_prism_topo_fp(3u);
	Dart p2 = add_prism_topo_fp(3u);
	sew_volumes_fp(p1, p2);

	Dart p3 = add_pyramid_topo_fp(4u);
	Dart p4 = add_pyramid_topo_fp(4u);
	sew_volumes_fp(p3, p4);

	close_map();

	uint32 count_vertices = nb_cells<Vertex::ORBIT>();
	uint32 count_edges = nb_cells<Edge::ORBIT>();
	uint32 count_faces = nb_cells<Face::ORBIT>();
	uint32 count_volumes = nb_cells<Volume::ORBIT>();

	uint32 k1 = codegree(Volume(p1));
	uint32 k2 = codegree(Volume(p2));
	Dart p12 = phi2(p1);
	merge_incident_volumes_of_face_topo(p1);
	--count_faces;
	--count_volumes;
	EXPECT_EQ(codegree(Volume(p12)), k1 + k2 - 2);

	uint32 k3 = codegree(Volume(p3));
	uint32 k4 = codegree(Volume(p4));
	Dart p32 = phi2(p3);
	merge_incident_volumes_of_face_topo(p3);
	--count_faces;
	--count_volumes;
	EXPECT_EQ(codegree(Volume(p32)), k3 + k4 - 2);

	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), count_edges);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), count_faces);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), count_volumes);
	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Cutting a volume adds a face and replaces a volume of codegree K,
 * with two subvolumes whose codegrees K1 and K2 verify K1+K2 = K+2.
 * The test performs NB_MAX face cuts between vertices of a randomly generated surface.
 * The number of generated cells is correct and the map integrity is preserved.
 */
TEST_F(CMap3TopoTest, cut_volume_topo)
{
	Dart p1 = add_prism_topo_fp(6u);
	close_map();

	uint32 count_vertices = nb_cells<Vertex::ORBIT>();
	uint32 count_edges = nb_cells<Edge::ORBIT>();
	uint32 count_faces = nb_cells<Face::ORBIT>();
	uint32 count_volumes = nb_cells<Volume::ORBIT>();

	uint32 k = codegree(Volume(p1));

	std::vector<Dart> path;
	Dart d = p1; path.push_back(d);
	d = phi<121>(d); path.push_back(d);
	d = phi<121>(d); path.push_back(d);
	d = phi1(d); path.push_back(d);
	d = phi<121>(d); path.push_back(d);
	d = phi<121>(d); path.push_back(d);

	Dart f = cut_volume_topo(path);
	++count_faces;
	++count_volumes;
	EXPECT_EQ(codegree(Volume(f)) + codegree(Volume(phi3(f))), k + 2);

	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), count_edges);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), count_faces);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), count_volumes);
	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Sew two volumes removes a face, K vertices and K edges (K being the codegree of the sewn faces)
 * The number of generated cells is correct and the map integrity is preserved.
 */
TEST_F(CMap3TopoTest, sew_volumes_topo)
{
	Dart p1 = add_prism_topo_fp(4u);
	Dart p2 = add_prism_topo_fp(4u);

	close_map();

	uint32 count_vertices = nb_cells<Vertex::ORBIT>();
	uint32 count_edges = nb_cells<Edge::ORBIT>();
	uint32 count_faces = nb_cells<Face::ORBIT>();
	uint32 count_volumes = nb_cells<Volume::ORBIT>();

	if (sew_volumes_topo(p1, p2))
	{
		uint32 k = codegree(Face(p1));
		count_vertices -= k;
		count_edges -= k;
		--count_faces;

		EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count_vertices);
		EXPECT_EQ(nb_cells<Edge::ORBIT>(), count_edges);
		EXPECT_EQ(nb_cells<Face::ORBIT>(), count_faces);
		EXPECT_EQ(nb_cells<Volume::ORBIT>(), count_volumes);
	}

	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Closing a map add one face per hole.
 * The test closes the holes of a randomly generated open surface.
 * The map integrity is preserved and the cell indexation is soundly completed
 */
TEST_F(CMap3TopoTest, close_map)
{
//	add_closed_surfaces();

//	// add attributes to initialize the indexation
//	add_attribute<int, CDart>("darts");
//	add_attribute<int, Vertex>("vertices");
//	add_attribute<int, Edge>("edges");
//	add_attribute<int, Face>("faces");
//	add_attribute<int, Volume>("volumes");
//	EXPECT_TRUE(check_map_integrity());

//	// create some random holes (full removal or partial unsewing of faces)
//	for (Dart d : darts_)
//	{
//		if (std::rand() % 2 == 1)
//		{
//			uint32 n = std::rand() % 10u;
//			uint32 k = degree(Face(d));

//			foreach_dart_of_orbit(Face(d), [&] (Dart e) -> bool
//			{
//				Dart e2 = phi2(e);
//				phi2_unsew(e);
//				// correct indexation of vertices
//				if (!same_open_vertex(e2, phi1(e))) new_open_vertex_embedding(e2);
//				if (!same_open_vertex(e, phi1(e2))) new_open_vertex_embedding(e);
//				// correct indexation of edges
//				new_orbit_embedding(Edge(e2));
//				// correct indexation of volumes
//				if (!same_volume(e2, e)) new_orbit_embedding(Volume(e));
//				// interrupt the face unsewing after n steps
//				if (n-- <= 0) return false;
//				// control if a partial or full face unsewing has been done
//				--k;
//				return true;
//			});
//			// if the face is completely unsewn randomly removes it
//			if (k == 0u && std::rand() % 2 == 1)
//			{
//				Dart e = d;
//				Dart it = phi1(e);
//				while (it != e)
//				{
//					Dart next = phi1(it);
//					this->remove_topology_element(it);
//					it = next;
//				}
//				this->remove_topology_element(e);
//			}
//		}
//	}

//	MapBuilder mbuild(*this);
//	mbuild.close_map();
//	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief The number of connected components is correctly counted
 */
TEST_F(CMap3TopoTest, nb_connected_components)
{
	Dart p1 = add_prism_topo_fp(3u);
	Dart p2 = add_prism_topo_fp(3u);
	sew_volumes_fp(p1, p2);

	Dart p3 = add_pyramid_topo_fp(4u);
	Dart p4 = add_pyramid_topo_fp(4u);
	sew_volumes_fp(p3, p4);

	add_prism_topo_fp(5u);

	close_map();

	EXPECT_EQ(nb_connected_components(), 3u);
}

#undef NB_MAX

} // namespace cgogn

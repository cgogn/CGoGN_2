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

#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/core/cmap/cmap2_builder.h>

namespace cgogn
{

#define NB_MAX 1000

/**
 * \brief The CMap2TopoTest class implements topological tests on CMap2
 * It derives from CMap2 to allow the test of protected methods
 *
 * Note that these tests check that the topological operators perform as wanted
 * but do neither test the containers (refs_, used_, etc.) nor the iterators.
 * These last tests are implemented in another test suite.
 */
class CMap2TopoTest : public CMap2, public ::testing::Test
{
public:

	using Inherit = CMap2;

	using MapBuilder = Inherit::Builder;
	using CDart = Inherit::CDart;
	using Vertex = Inherit::Vertex;
	using Edge = Inherit::Edge;
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
	CMap2TopoTest()
	{
		darts_.reserve(NB_MAX);
		std::srand(uint32(std::time(0)));
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

		foreach_dart_of_orbit_until(Volume(d), [&] (Dart vit)
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
	 * \brief Generate a random set of faces and put them in darts_
	 * \return The total number of added vertices.
	 * The face size ranges from 1 to 10.
	 * A random dart of each face is put in the darts_ array.
	 */
	uint32 add_faces(uint32 n)
	{
		darts_.clear();
		uint32 count = 0u;
		for (uint32 i = 0u; i < n; ++i)
		{
			uint32 m = 1u + std::rand() % 10u;
			Dart d = add_face_topo(m);
			count += m;

			m = std::rand() % 10u;
			while (m-- > 0u) d = phi1(d);

			darts_.push_back(d);
		}
		return count;
	}

	/**
	 * \brief Generate a set of closed surfaces with arbitrary genus.
	 */
	void add_closed_surfaces()
	{
		darts_.clear();

		// Generate NB_MAX random 1-faces (without boundary)
		for (uint32 i = 0u; i < NB_MAX; ++i)
		{
			uint32 n = 3u + std::rand() % 10u;
			Dart d = Inherit::Inherit::add_face_topo(n);
			darts_.push_back(d);
		}
		// Sew some pairs of 1-edges
		for (uint32 i = 0u; i < 3u * NB_MAX; ++i)
		{
			Dart e1 = darts_[std::rand() % NB_MAX];
			uint32 n = std::rand() % 10u;
			while (n-- > 0u) e1 = phi1(e1);

			Dart e2 = darts_[std::rand() % NB_MAX];
			n = std::rand() % 10u;
			while (n-- > 0u) e2 = phi1(e2);

			n = 1 + std::rand() % 3u;
			while (n-- > 0u && phi2(e1) == e1 && phi2(e2) == e2 && e2 != e1)
			{
				phi2_sew(e2, e1);
				e1 = phi1(e1);
				e2 = phi_1(e2);
			}
		}
		// Close the map
		MapBuilder mbuild(*this);
		mbuild.close_map();
	}
};

/**
 * \brief The random generated maps used in the tests are sound.
 */
TEST_F(CMap2TopoTest, random_map_generators)
{
	EXPECT_EQ(nb_darts(), 0u);

	add_faces(NB_MAX);
	EXPECT_TRUE(check_map_integrity());

	add_closed_surfaces();
	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Test attribute management
 *
 */
TEST_F(CMap2TopoTest, add_attribute)
{
	add_faces(NB_MAX);
	add_closed_surfaces();

	add_attribute<int32, CDart>("darts");
	EXPECT_TRUE(check_map_integrity());

	add_attribute<int32, Vertex>("vertices");
	EXPECT_TRUE(check_map_integrity());

	add_attribute<int32, Edge>("edges");
	EXPECT_TRUE(check_map_integrity());

	add_attribute<int32, Face>("faces");
	EXPECT_TRUE(check_map_integrity());

	add_attribute<int32, Volume>("Volumes");
	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Sewing and unsewing darts correctly changes the topological relations.
 * The test performs NB_MAX sewing and unsewing on randomly chosen dart of darts_.
 * The map integrity is not preserved (this test creates fixed points for PHI2).
 */
TEST_F(CMap2TopoTest, phi2_sew_unsew)
{
	add_faces(NB_MAX);

	for (uint32 i = 0u; i < NB_MAX; ++i)
	{
		Dart d0 = darts_[std::rand() % NB_MAX];
		Dart d2 = phi2(d0);
		phi2_unsew(d0);
		EXPECT_TRUE(phi2(d0) == d0);
		EXPECT_TRUE(phi2(d2) == d2);
		Dart e0 = d0;
		while (e0 == d0) e0 = darts_[std::rand() % NB_MAX];
		phi2_unsew(e0);

		phi2_sew(d0, e0);
		EXPECT_TRUE(phi2(d0) == e0);
		EXPECT_TRUE(phi2(e0) == d0);
	}
}

/**
 * \brief Adding a 2-face of size n adds 2*n darts, n vertices and edges, 2 1-faces and 1 volume.
 * The test adds some faces and check that the number of generated cells is correct
 * and that the map integrity is preserved.
 */
TEST_F(CMap2TopoTest, add_face_topo)
{
	add_face_topo(1u);
	EXPECT_EQ(nb_darts(), 2u);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), 1u);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), 1u);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), 1u);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), 1u);

	add_face_topo(10u);
	EXPECT_EQ(nb_darts(), 22u);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), 11u);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), 11u);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), 2u);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), 2u);

	uint32 count_vertices = 11u + add_faces(NB_MAX);

	for (Dart d : darts_)
		EXPECT_TRUE(is_boundary(phi2(d)));

	EXPECT_EQ(nb_darts(), 2u * count_vertices);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), count_vertices);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), NB_MAX + 2u);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), NB_MAX + 2u);

	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Adding a pyramid whose base has n sides build a surface with
 * 4*n darts, n+1 vertices, 2*n edges, n+1 faces and 1 volume.
 * The test adds some pyramides and check that the number of generated cells is correct
 * and that the map integrity is preserved.
 */
TEST_F(CMap2TopoTest, add_pyramid_topo)
{
	add_pyramid_topo(3u);
	EXPECT_EQ(nb_darts(), 12u);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), 4u);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), 6u);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), 4u);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), 1u);

	add_pyramid_topo(10u);
	EXPECT_EQ(nb_darts(), 40u+12u);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), 11+4u);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), 20u+6u);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), 11u+4u);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), 1u+1u);

	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Adding a prism whose base has n sides build a surface with
 * 4*n darts, n+1 vertices, 2*n edges, n+1 faces and 1 volume.
 * The test adds some prims and check that the number of generated cells is correct
 * and that the map integrity is preserved.
 */
TEST_F(CMap2TopoTest, add_prism_topo)
{
	add_prism_topo(3u);
	EXPECT_EQ(nb_darts(), 18u);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), 6u);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), 9u);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), 5u);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), 1u);

	add_prism_topo(10u);
	EXPECT_EQ(nb_darts(), 60u+18u);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), 20u+6u);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), 30u+9u);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), 12u+5u);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), 1u+1u);

	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Cutting an edge increases the size of both incident faces and add a vertex of degree 2.
 * The test performs NB_MAX edge cutting on edges of randomly generated faces.
 * The number of generated cells is correct and the map integrity is preserved.
 */
TEST_F(CMap2TopoTest, cut_edge_topo)
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
 * \brief Fliping an edge changes the degree of its vertices.
 * The test performs NB_MAX edge flips on randomly generated faces.
 * The expected cells are modified and the map integrity is preserved.
 */
TEST_F(CMap2TopoTest, flip_edge_topo)
{
	add_closed_surfaces();

	uint32 count_vertices = nb_cells<Vertex::ORBIT>();
	uint32 count_edges = nb_cells<Edge::ORBIT>();
	uint32 count_faces = nb_cells<Face::ORBIT>();
	uint32 count_volumes = nb_cells<Volume::ORBIT>();

	for (Dart d : darts_)
	{
		Dart e1 = d;	// choose a random edge in the face
		uint32 i = std::rand() % 10u;
		while (i-- > 0u) e1 = phi1(e1);
		Dart e2 = phi2(e1);

		uint32 k1 = codegree(Face(e1));
		uint32 k2 = codegree(Face(e2));

		Vertex k11_vertex = Vertex(e1);
		Vertex k12_vertex = Vertex(phi1(phi1(e2)));
		Vertex k21_vertex = Vertex(e2);
		Vertex k22_vertex = Vertex(phi1(phi1(e1)));

		uint32 k11 = degree(k11_vertex);
		uint32 k12 = degree(k12_vertex);
		uint32 k21 = degree(k21_vertex);
		uint32 k22 = degree(k22_vertex);
		uint32* k11_ptr = &k11;
		uint32* k12_ptr = &k12;
		uint32* k21_ptr = &k21;
		uint32* k22_ptr = &k22;

		// Handle special vertex configurations
		if (same_cell(k11_vertex, k12_vertex)) k12_ptr = k11_ptr;
		if (same_cell(k11_vertex, k21_vertex)) k21_ptr = k11_ptr;
		if (same_cell(k11_vertex, k22_vertex)) k22_ptr = k11_ptr;
		if (same_cell(k12_vertex, k21_vertex)) k21_ptr = k12_ptr;
		if (same_cell(k12_vertex, k22_vertex)) k22_ptr = k12_ptr;
		if (same_cell(k21_vertex, k22_vertex)) k22_ptr = k21_ptr;

		// Vertices with degree 1 do not move during an edge flip
		bool k11_move = (degree(k11_vertex) > 1);
		bool k21_move = (degree(k21_vertex) > 1);

		if (k11_move)
		{
			*k11_ptr -= 1u;
			*k12_ptr += 1u;
		}
		if (k21_move)
		{
			*k21_ptr -= 1u;
			*k22_ptr += 1u;
		}

		if (flip_edge_topo(e1))
		{
			EXPECT_EQ(codegree(Face(e1)), k1);
			EXPECT_EQ(codegree(Face(e2)), k2);

			if (k11_move)
			{
				EXPECT_EQ(degree(Vertex(phi_1(e1))), *k11_ptr);
				EXPECT_EQ(degree(Vertex(e1)), *k12_ptr);
			}
			else
			{
				EXPECT_EQ(degree(k11_vertex), *k11_ptr);
				EXPECT_EQ(degree(k12_vertex), *k12_ptr);
			}
			if (k21_move)
			{
				EXPECT_EQ(degree(Vertex(phi_1(e2))), *k21_ptr);
				EXPECT_EQ(degree(Vertex(e2)), *k22_ptr);
			}
			else
			{
				EXPECT_EQ(degree(k21_vertex), *k21_ptr);
				EXPECT_EQ(degree(k22_vertex), *k22_ptr);
			}
		}
	}

	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), count_edges);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), count_faces);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), count_volumes);

	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Cutting a face add an edge and replace a face of degree K,
 * with two subfaces whose degrees K1 and K2 verify K1+K2 = K+2.
 * The test performs NB_MAX face cuts between vertices of a randomly generated surface.
 * The number of generated cells is correct and the map integrity is preserved.
 */
TEST_F(CMap2TopoTest, cut_face_topo)
{
	add_closed_surfaces();

	uint32 count_vertices = nb_cells<Vertex::ORBIT>();
	uint32 count_edges = nb_cells<Edge::ORBIT>();
	uint32 count_faces = nb_cells<Face::ORBIT>();
	uint32 count_volumes = nb_cells<Volume::ORBIT>();

	for (Dart d : darts_)
	{
		Dart dd = d;

		uint32 k = codegree(Face(dd));
		if (k > 1u)
		{
			Dart e = dd; // find a second dart in the face of d (distinct from d)
			uint32 i = std::rand() % 10u;
			while (i-- > 0u) e = phi1(e);
			if (e == dd) e = phi1(e);

			cut_face_topo(dd, e);
			++count_edges;
			++count_faces;

			EXPECT_EQ(codegree(Face(dd)) + codegree(Face(e)), k + 2);
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
TEST_F(CMap2TopoTest, merge_incident_faces_topo)
{
//	add_closed_surfaces();

	MapBuilder mbuild(*this);

	Dart f1 = Inherit::Inherit::add_face_topo(5u);
	Dart f2 = Inherit::Inherit::add_face_topo(3u);
	phi2_sew(f1, f2);

	mbuild.close_map();

	uint32 count_vertices = nb_cells<Vertex::ORBIT>();
	uint32 count_edges = nb_cells<Edge::ORBIT>();
	uint32 count_faces = nb_cells<Face::ORBIT>();
	uint32 count_volumes = nb_cells<Volume::ORBIT>();

	uint32 k1 = codegree(Face(f1));
	uint32 k2 = codegree(Face(f2));
	Dart f11 = phi1(f1);
	merge_incident_faces_topo(f1);
	--count_edges;
	--count_faces;
	EXPECT_EQ(codegree(Face(f11)), k1 + k2 - 2);

//	const ChunkArrayContainer<uint8>& topo_container = topology_container();

//	for (Dart d : darts_)
//	{
//		// check if the dart has not been removed by a previous merge
//		if (topo_container.used(d.index))
//		{
//			Dart d1 = phi1(d);
//			Dart d2 = phi2(d);

//			uint32 k1 = codegree(Face(d));
//			uint32 k2 = codegree(Face(d2));

//			if (merge_incident_faces_topo(d))
//			{
//				--count_edges;
//				--count_faces;
//				EXPECT_EQ(codegree(Face(d1)), k1 + k2 - 2);
//			}
//		}
//	}

	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), count_edges);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), count_faces);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), count_volumes);

	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief Closing a map add one face per holes.
 * The test closes the holes of a randomly generated open surface.
 * The map integrity is preserved and the cell indexation is soundly completed
 */
TEST_F(CMap2TopoTest, close_map)
{
	add_closed_surfaces();

	// add attributes to initialize the indexation
	add_attribute<int32, CDart>("darts");
	add_attribute<int32, Vertex>("vertices");
	add_attribute<int32, Edge>("edges");
	add_attribute<int32, Face>("faces");
	add_attribute<int32, Volume>("volumes");

	EXPECT_TRUE(check_map_integrity());

	// create some random holes (full removal or partial unsewing of faces)
	for (Dart d : darts_)
	{
		if (std::rand() % 2 == 1)
		{
			uint32 n = std::rand() % 10u;
			uint32 k = codegree(Face(d));

			foreach_dart_of_orbit_until(Face(d), [&] (Dart e)
			{
				Dart e2 = phi2(e);
				if (!this->is_boundary(e) && !this->is_boundary(e2))
				{
					phi2_unsew(e);
					// correct indexation of vertices
					if (!same_open_vertex(e2, phi1(e))) new_open_vertex_embedding(e2);
					if (!same_open_vertex(e, phi1(e2))) new_open_vertex_embedding(e);
					// correct indexation of edges
					new_orbit_embedding(Edge(e2));
					// correct indexation of volumes
					if (!same_volume(e2, e)) new_orbit_embedding(Volume(e));
					// interrupt the face unsewing after n steps
					if (n-- <= 0) return false;
					// control if a partial or full face unsewing has been done
					--k;
				}
				return true;
			});
			// if the face is completely unsewn randomly removes it
			if (k == 0u && std::rand() % 2 == 1)
			{
				Dart e = d;
				Dart it = phi1(e);
				while (it != e)
				{
					Dart next = phi1(it);
					this->remove_topology_element(it);
					it = next;
				}
				this->remove_topology_element(e);
			}
		}
	}

	MapBuilder mbuild(*this);
	mbuild.close_map();

	EXPECT_TRUE(check_map_integrity());
}

/**
 * \brief The degree & codegree of cells are correctly computed
 */
TEST_F(CMap2TopoTest, degree)
{
	Face f1(this->add_face_topo(1u));
	EXPECT_EQ(codegree(Edge(f1.dart)), 1u);
	EXPECT_EQ(degree(Edge(f1.dart)), 1u);
	EXPECT_EQ(degree(f1), 1u);

	Face f2(this->add_face_topo(10u));
	EXPECT_EQ(codegree(Edge(f2.dart)), 2u);
	EXPECT_EQ(degree(Edge(f2.dart)), 1u);

	phi2_unsew(f1.dart);
	phi2_unsew(f2.dart);
	phi2_sew(f1.dart, f2.dart);
	EXPECT_EQ(degree(Edge(f1.dart)), 2u);
	EXPECT_EQ(degree(Edge(f2.dart)), 2u);
}

/**
 * \brief The multi_phi are correctly applied
 */
TEST_F(CMap2TopoTest, multi_phi)
{
	Face f(this->add_face_topo(10u));

	EXPECT_EQ(f.dart, this->phi<22>(f.dart));
	EXPECT_EQ(f.dart, this->phi<1111111111>(f.dart));
	EXPECT_EQ(f.dart, this->phi<211111111112>(f.dart));
	EXPECT_EQ(f.dart, this->phi<11122111221111>(f.dart));
}

/**
 * \brief The number of connected components is correctly counted
 */
TEST_F(CMap2TopoTest, nb_connected_components)
{
	add_faces(10u);
	add_prism_topo(3u);
	add_prism_topo(5u);
	add_pyramid_topo(4u);

	EXPECT_EQ(nb_connected_components(), 13u);
}

#undef NB_MAX

} // namespace cgogn

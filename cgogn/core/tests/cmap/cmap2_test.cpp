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

#include <core/cmap/cmap2_builder.h>

namespace cgogn
{

#define NB_MAX 100

/*!
 * \brief The CMap2Test class implements tests on embedded CMap2
 * It contains a CMap2 to which vertex, edge, face and volume attribute
 * are added to enforce the indexation mechanism in cell traversals.
 *
 * Note that pure topological operations have already been tested,
 * in CMap2TopoTest, thus only the indexation mechanism used for the
 * embedding of cells is tested here.
 */
class CMap2Test : public ::testing::Test
{

public:

	using testCMap2 = CMap2<DefaultMapTraits>;
	using MapBuilder = CMap2Builder_T<DefaultMapTraits>;
	using CDart = testCMap2::CDart;
	using Vertex = testCMap2::Vertex;
	using Edge = testCMap2::Edge;
	using Face = testCMap2::Face;
	using Volume = testCMap2::Volume;

protected:

	testCMap2 cmap_;

	/*!
	 * \brief A vector of darts on which the methods are tested.
	 */
	std::vector<Dart> darts_;

	CMap2Test()
	{
		darts_.reserve(NB_MAX);
		std::srand(static_cast<unsigned int>(std::time(0)));

		cmap_.add_attribute<int, CDart::ORBIT>("darts");
		cmap_.add_attribute<int, Vertex::ORBIT>("vertices");
		cmap_.add_attribute<int, Edge::ORBIT>("edges");
		cmap_.add_attribute<int, Face::ORBIT>("faces");
		cmap_.add_attribute<int, Volume::ORBIT>("volumes");
	}

	/*!
	 * \brief Generate a random set of faces and put them in darts_
	 * \return The total number of added vertices.
	 * The face size ranges from 1 to 10.
	 * A random dart of each face is put in the darts_ array.
	 */
	unsigned int add_faces(unsigned int n)
	{
		darts_.clear();
		unsigned int count = 0u;
		for (unsigned int i = 0u; i < n; ++i)
		{
			unsigned int m = 1u + std::rand() % 10u;
			Dart d = cmap_.add_face(m);
			count += m;

			m = std::rand() % 10u;
			while (m-- > 0u) d = cmap_.phi1(d);

			darts_.push_back(d);
		}
		return count;
	}

	/*!
	 * \brief Generate a closed surface from the set of faces in darts_
	 */
	void add_closed_surfaces()
	{
		darts_.clear();
		MapBuilder mbuild(cmap_);
		unsigned int n;

		// Generate NB_MAX random 1-faces (without boundary)
		for (unsigned int i = 0u; i < NB_MAX; ++i)
		{
			n = 1u + std::rand() % 10u;
			Dart d = mbuild.add_face_topo_parent(n);
			darts_.push_back(d);
		}
		// Sew some pairs off edges
		for (unsigned int i = 0u; i < 3u * NB_MAX; ++i)
		{
			Dart e1 = darts_[std::rand() % NB_MAX];
			n = std::rand() % 10u;
			while (n-- > 0u) e1 = cmap_.phi1(e1);

			Dart e2 = darts_[std::rand() % NB_MAX];
			n = std::rand() % 10u;
			while (n-- > 0u) e2 = cmap_.phi1(e2);

			n = 1 + std::rand() % 3u;
			while (n-- > 0u && cmap_.phi2(e1) == e1 && cmap_.phi2(e2) == e2 && e2 != e1)
			{
				mbuild.phi2_sew(e2, e1);
				e1 = cmap_.phi1(e1);
				e2 = cmap_.phi_1(e2);
			}
		}
		// Close the map (remove remaining boundary)
		cmap_.foreach_dart(
			[&] (Dart d)
			{
				if (cmap_.phi2(d) == d) mbuild.close_hole_topo(d);
			},
			[this] (Dart d) { return true; }
		);
		// Embed the map
		cmap_.foreach_dart(
			[&] (Dart d)
			{
				mbuild.new_orbit_embedding(CDart(d));
			},
			[this] (Dart d) { return true; }
		);
		cmap_.foreach_cell<FORCE_DART_MARKING>(
			[&] (Vertex v)
			{
				mbuild.new_orbit_embedding(v);
			},
			[this] (Dart d) { return true; }
		);
		cmap_.foreach_cell<FORCE_DART_MARKING>(
			[&] (Edge e)
			{
				mbuild.new_orbit_embedding(e);
			},
			[this] (Dart d) { return true; }
		);
		cmap_.foreach_cell<FORCE_DART_MARKING>(
			[&] (Face f)
			{
				mbuild.new_orbit_embedding(f);
			},
			[this] (Dart d) { return true; }
		);
		cmap_.foreach_cell<FORCE_DART_MARKING>(
			[&] (Volume w)
			{
				mbuild.new_orbit_embedding(w);
			},
			[this] (Dart d) { return true; }
		);
	}
};

/*!
 * \brief The random generated maps used in the tests are sound.
 */
TEST_F(CMap2Test, random_map_generators)
{
	add_faces(NB_MAX);
	EXPECT_TRUE(cmap_.check_map_integrity());

	add_closed_surfaces();
	EXPECT_TRUE(cmap_.check_map_integrity());
}

/*!
 * \brief Adding faces preserves the cell indexation
 */
TEST_F(CMap2Test, add_face)
{
	unsigned int count_vertices = add_faces(NB_MAX);

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), count_vertices);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 2 * NB_MAX);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), NB_MAX);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

/*!
 * \brief Cutting edges preserves the cell indexation
 */
TEST_F(CMap2Test, cut_edge)
{
	add_closed_surfaces();

	for (Dart d : darts_)
		cmap_.cut_edge(Edge(d));

	EXPECT_TRUE(cmap_.check_map_integrity());
}

/*!
 * \brief Cutting faces preserves the cell indexation
 */
TEST_F(CMap2Test, cut_face)
{
	add_closed_surfaces();

	for (Dart d : darts_)
	{
		if (cmap_.degree(Face(d)) > 1u)
		{
			Dart e = d; // find a second dart in the face of d (distinct from d)
			unsigned int i = std::rand() % 10u;
			while (i-- > 0u) e = cmap_.phi1(e);
			if (e == d) e = cmap_.phi1(e);

			cmap_.cut_face(Vertex(d), Vertex(e));
		}
	}
	EXPECT_TRUE(cmap_.check_map_integrity());
}

#undef NB_MAX

} // namespace cgogn

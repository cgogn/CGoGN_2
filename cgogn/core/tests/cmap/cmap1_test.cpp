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

#include <cgogn/core/cmap/cmap1.h>

namespace cgogn
{

#define NB_MAX 100u

/**
 * \brief The CMap1Test class implements tests on embedded CMap1
 * It contains a CMap1 to which vertex and face attribute are added
 * to enforce the indexation mecanism in cell traversals.
 *
 * Note that pure topological operations have already been tested,
 * in CMap1TopoTest, thus only the indexation mecanism used for the
 * embedding of cells is tested here.
 */
class CMap1Test : public ::testing::Test
{
public:

	using Vertex = CMap1::Vertex;
	using Face = CMap1::Face;

protected:

	CMap1 cmap_;

	/**
	 * \brief A vector of darts on which the methods are tested.
	 */
	std::vector<Dart> darts_;

	CMap1Test()
	{
		darts_.reserve(NB_MAX);
		std::srand(uint32(std::time(0)));

		cmap_.add_attribute<int32, Vertex>("vertices");
		cmap_.add_attribute<int32, Face>("faces");
	}

	/**
	 * \brief Generate a random set of faces and put them in darts_
	 * \return The total number of added darts or vertices.
	 * The face size ranges from 1 to 10.
	 * A random dart of each face is put in the darts_ array.
	 */
	uint32 add_faces(uint32 n)
	{
		darts_.clear();
		uint32 count = 0u;
		for (uint32 i = 0u; i < n; ++i)
		{
			uint32 m = 1u + uint32(std::rand()) % 10;
			Dart d = cmap_.add_face(m).dart;
			count += m;

			m = uint32(std::rand()) % 10u;
			while (m-- > 0u) d = cmap_.phi1(d);

			darts_.push_back(d);
		}
		return count;
	}
};

/**
 * \brief The random generated maps used in the tests are sound.
 */
TEST_F(CMap1Test, random_map_generators)
{
	add_faces(NB_MAX);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

/**
 * \brief Adding faces preserves the cell indexation
 */
TEST_F(CMap1Test, add_face)
{
	uint32 count_vertices = add_faces(NB_MAX);

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), NB_MAX);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

/**
 * \brief Removing faces preserves the cell indexation
 */
TEST_F(CMap1Test, remove_face)
{
	uint32 count_vertices = add_faces(NB_MAX);
	uint32 count_faces = NB_MAX;

	for (Dart d : darts_)
	{
		if (std::rand() % 3 == 1)
		{
			Face f(d);
			uint32 k = cmap_.codegree(f);
			cmap_.remove_face(f);
			count_vertices -= k;
			--count_faces;
		}
	}

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), count_faces);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

/**
 * \brief Splitting vertices preserves the cell indexation
 */
TEST_F(CMap1Test, split_vertex)
{
	uint32 count_vertices = add_faces(NB_MAX);

	for (Dart d : darts_)
	{
		cmap_.split_vertex(Vertex(d));
		++count_vertices;
	}

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), NB_MAX);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

/**
 * \brief Removing vertices preserves the cell indexation
 */
TEST_F(CMap1Test, remove_vertex)
{
	uint32 count_vertices = add_faces(NB_MAX);
	uint32 count_faces = NB_MAX;

	for (Dart d: darts_)
	{
		uint32 k = cmap_.codegree(Face(d));
		cmap_.remove_vertex(Vertex(d));
		--count_vertices;
		if (k == 1u) --count_faces;
	}

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), count_vertices);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), count_faces);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

#undef NB_MAX

} // namespace cgogn

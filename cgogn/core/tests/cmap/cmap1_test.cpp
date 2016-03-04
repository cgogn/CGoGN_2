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

#include <core/cmap/cmap1.h>

namespace cgogn
{

#define NB_MAX 100

/*!
 * \brief The CMap1Test class implements tests on embedded CMap1
 * It contains a CMap1 to which vertex and face attribute are added
 * to enforce the indexation mecanism in cell traversals.
 *
 * Note that pure topological operations have already been tested,
 * in CMap1TopoTest, thus only the indexation mecanism used for the
 * embedding of cells is tested here.
 */
class CMap1Test: public ::testing::Test
{

public:

	using testCMap1 = CMap1<DefaultMapTraits>;
	using Vertex = testCMap1::Vertex;
	using Face = testCMap1::Face;

protected:

	testCMap1 cmap_;

	/*!
	 * \brief A vector of darts on which the methods are tested.
	 */
	std::vector<Dart> darts_;

	CMap1Test()
	{
		std::srand(static_cast<unsigned int>(std::time(0)));

		cmap_.add_attribute<int, Vertex::ORBIT>("vertices");
		cmap_.add_attribute<int, Face::ORBIT>("faces");
	}

	/*!
	 * \brief Generate a random set of faces and put them in darts_
	 * \return The total number of added darts or vertices.
	 * The face size ranges from 1 to 10.
	 * A random dart of each face is put in the darts_ array.
	 */
	unsigned int addFaces(unsigned int n)
	{
		unsigned int count = 0;
		for (unsigned int i = 0; i < n; ++i)
		{
			unsigned int n = 1 + std::rand() % 10;
			Dart d = cmap_.add_face(n);
			count += n;

			while (std::rand()%10 != 1)
				d = cmap_.phi1(d);

			darts_.push_back(d);
		}
		return count;
	}
};

/*!
 * \brief Adding faces preserves the cell indexation
 */
TEST_F(CMap1Test, add_face)
{
	unsigned int countVertices = addFaces(NB_MAX);

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), countVertices);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), NB_MAX);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

/*!
 * \brief Removing faces preserves the cell indexation
 */
TEST_F(CMap1Test, remove_face)
{
	unsigned int countVertices = addFaces(NB_MAX);
	int countFaces = NB_MAX;

	for (Dart d: darts_)
	{
		if (std::rand()%3 == 1) {
			unsigned int k = cmap_.degree(d);
			cmap_.remove_face(d);
			countVertices -= k;
			--countFaces;
		}
	}

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), countVertices);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), countFaces);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

/*!
 * \brief Splitting vertices preserves the cell indexation
 */
TEST_F(CMap1Test, split_vertex)
{
	unsigned int countVertices = addFaces(NB_MAX);

	for (Dart d: darts_)
	{
		cmap_.split_vertex(d);
		++countVertices;
	}

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), countVertices);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), NB_MAX);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

/*!
 * \brief Removing vertices preserves the cell indexation
 */
TEST_F(CMap1Test, remove_vertex)
{
	unsigned int countVertices = addFaces(NB_MAX);
	unsigned int countFaces = NB_MAX;

	for (Dart d: darts_)
	{
		unsigned int k = cmap_.degree(Face(d));
		cmap_.remove_vertex(Vertex(d));
		--countVertices;
		if (k == 1) --countFaces;
	}

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), countVertices);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), countFaces);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

#undef NB_MAX

} // namespace cgogn

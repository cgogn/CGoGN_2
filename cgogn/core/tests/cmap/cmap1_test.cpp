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
	using VertexAttributeHandler = testCMap1::VertexAttributeHandler<int>;
	using Vertex = testCMap1::Vertex;
	using FaceAttributeHandler = testCMap1::FaceAttributeHandler<int>;
	using Face = testCMap1::Face;

protected:

	testCMap1 cmap_;
	VertexAttributeHandler vertices_;
	FaceAttributeHandler faces_;

	CMap1Test()
	{
		std::srand(static_cast<unsigned int>(std::time(0)));

		vertices_ = cmap_.add_attribute<int, Vertex::ORBIT>("vertices");
		faces_ = cmap_.add_attribute<int, Face::ORBIT>("faces");
	}

	int randomFaces() {
		int count = 0;
		for (int i = 0; i < NB_MAX; ++i) {
			int n = 1 + std::rand() % 100;
			Dart d = cmap_.add_face(n);
			count += n;

			while (std::rand()%10 != 1)
				d = cmap_.phi1(d);

			tdarts_[i] = d;
		}
		return count;
	}

	std::array<Dart, NB_MAX> tdarts_;
};

/*!
 * \brief Adding vertices preserves the cell indexation
 */
TEST_F(CMap1Test, add_face)
{
	int n = randomFaces();

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), n);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), NB_MAX);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

TEST_F(CMap1Test, remove_face)
{
	int n = randomFaces();

	int countVertex = n;
	int countFace = NB_MAX;
	for (int i = 0; i < NB_MAX; ++i) {
		Face d = tdarts_[i];
		unsigned int k = cmap_.degree(d);
		cmap_.remove_face(d);
		countVertex -= k;
		--countFace;
	}

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), countVertex);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), countFace);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

TEST_F(CMap1Test, split_vertex)
{
	int n = randomFaces();

	for (int i = 0; i < NB_MAX; ++i) {
		Face d = tdarts_[i];
		unsigned int k = cmap_.degree(Face(d));
		cmap_.split_vertex(Vertex(d));
	}

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), n+NB_MAX);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), NB_MAX);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

TEST_F(CMap1Test, remove_vertex)
{
	int n = randomFaces();

	int countVertex = n;
	int countFace = NB_MAX;
	for (int i = 0; i < NB_MAX; ++i) {
		Face d = tdarts_[i];
		unsigned int k = cmap_.degree(d);
		cmap_.remove_vertex(Vertex(d));
		--countVertex;
		if (k == 1u) --countFace;
	}

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), countVertex);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), countFace);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

#undef NB_MAX

} // namespace cgogn

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

#include <cstdlib>
#include <ctime>

#include <gtest/gtest.h>

#include <core/cmap/cmap1.h>
#include <core/cmap/sanity_check.h>

namespace cgogn
{

#define NB_MAX 1000

class CMap1Test: public ::testing::Test
{

public:

	using testCMap1 = CMap1<DefaultMapTraits>;
	using Vertex = testCMap1::Vertex;
	using Face = testCMap1::Face;

protected:

	testCMap1 cmap_;

	CMap1Test()
	{
		std::srand(static_cast<unsigned int>(std::time(0)));

		cmap_.add_attribute<int, Vertex::ORBIT>("vertices");
		cmap_.add_attribute<int, Face::ORBIT>("faces");
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

TEST_F(CMap1Test, testCMap1Constructor)
{
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 0u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 0u);
}

TEST_F(CMap1Test, addFace)
{
	int n = randomFaces();

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), n);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), NB_MAX);
	EXPECT_TRUE(is_well_embedded<Vertex::ORBIT>(cmap_));
	EXPECT_TRUE(is_well_embedded<Face::ORBIT>(cmap_));
	EXPECT_TRUE(is_orbit_embedding_unique<Vertex::ORBIT>(cmap_));
	EXPECT_TRUE(is_orbit_embedding_unique<Face::ORBIT>(cmap_));
}

TEST_F(CMap1Test, testSplitVertex)
{
	int n = randomFaces();

	for (int i = 0; i < NB_MAX; ++i) {
		Face d = tdarts_[i];
		unsigned int k = cmap_.degree(Face(d));
		EXPECT_TRUE(k > 0); // avoid warning unused var k
		cmap_.split_vertex(Vertex(d));
	}

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), n+NB_MAX);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), NB_MAX);
	EXPECT_TRUE(is_well_embedded<Vertex::ORBIT>(cmap_));
	EXPECT_TRUE(is_well_embedded<Face::ORBIT>(cmap_));
	EXPECT_TRUE(is_orbit_embedding_unique<Vertex::ORBIT>(cmap_));
	EXPECT_TRUE(is_orbit_embedding_unique<Face::ORBIT>(cmap_));
}

TEST_F(CMap1Test, testRemoveVertex)
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
	EXPECT_TRUE(is_well_embedded<Vertex::ORBIT>(cmap_));
	EXPECT_TRUE(is_well_embedded<Face::ORBIT>(cmap_));
	EXPECT_TRUE(is_orbit_embedding_unique<Vertex::ORBIT>(cmap_));
	EXPECT_TRUE(is_orbit_embedding_unique<Face::ORBIT>(cmap_));
}

TEST_F(CMap1Test, testRemoveFace)
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
	EXPECT_TRUE(is_well_embedded<Vertex::ORBIT>(cmap_));
	EXPECT_TRUE(is_well_embedded<Face::ORBIT>(cmap_));
	EXPECT_TRUE(is_orbit_embedding_unique<Vertex::ORBIT>(cmap_));
	EXPECT_TRUE(is_orbit_embedding_unique<Face::ORBIT>(cmap_));
}

} // namespace cgogn

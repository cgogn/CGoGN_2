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

#include <core/cmap/cmap0.h>

namespace cgogn
{

#define NB_MAX 100

class CMap0TopoTest: public ::testing::Test
{

public:

	using testCMap0 = CMap0<DefaultMapTraits>;
	using Vertex = testCMap0::Vertex;

protected:

	testCMap0 cmap_;

	CMap0TopoTest()
	{
	}

	std::array<Dart, NB_MAX> tdarts_;

	int addVertices() {
		for (int i = 0; i < NB_MAX; ++i)
			tdarts_[i] = cmap_.add_vertex();

		return NB_MAX;
	}
};

TEST_F(CMap0TopoTest, testCMap0Constructor)
{
	EXPECT_EQ(cmap_.nb_darts(), 0u);
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 0u);
}

TEST_F(CMap0TopoTest, testAddVertex)
{
	for (int i = 1; i< NB_MAX; ++i) {
		cmap_.add_vertex();
		EXPECT_EQ(cmap_.nb_darts(), i);
		EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), i);
	}
	EXPECT_TRUE(cmap_.check_map_integrity());
}

TEST_F(CMap0TopoTest, testRemoveVertex)
{
	int n = addVertices();

	int countVertices = n;
	for (int i = 0; i < n; ++i) {
		Vertex d = tdarts_[i];
		if (i%2 == 1) {
			cmap_.remove_vertex(Vertex(d));
			--countVertices;
			EXPECT_EQ(cmap_.nb_darts(), countVertices);
			EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), countVertices);
		}
	}
	EXPECT_TRUE(cmap_.check_map_integrity());
}

} // namespace cgogn

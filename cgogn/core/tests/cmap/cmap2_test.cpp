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

#include <core/cmap/cmap2.h>

namespace cgogn
{

#define NB_MAX 100

/*!
 * \brief The CMap2Test class implements tests on embedded CMap2
 * It contains a CMap2 to which vertex, edge, face and volume attribute
 * are added to enforce the indexation mecanism in cell traversals.
 *
 * Note that pure topological operations have already been tested,
 * in CMap2TopoTest, thus only the indexation mecanism used for the
 * embedding of cells is tested here.
 */
class CMap2Test: public ::testing::Test
{

public:

	using testCMap2 = CMap2<DefaultMapTraits>;
	using Vertex = testCMap2::Vertex;
	using Edge = testCMap2::Edge;
	using Face = testCMap2::Face;
	using Volume = testCMap2::Volume;

protected:

	testCMap2 cmap_;

	CMap2Test()
	{
		std::srand(static_cast<unsigned int>(std::time(0)));

		cmap_.add_attribute<int, Vertex::ORBIT>("vertices");
		cmap_.add_attribute<int, Edge::ORBIT>("edges");
		cmap_.add_attribute<int, Face::ORBIT>("faces");
		cmap_.add_attribute<int, Volume::ORBIT>("volumes");
	}

	int randomFaces() {
		int count = 0;
		for (int i = 0; i < NB_MAX; ++i) {
			int n = 1 + std::rand() % 100;
			Dart d = cmap_.add_face(n);
			count += n;

			n = std::rand() % 10;
			while (n-- > 0)	d = cmap_.phi1(d);

			tdarts_[i] = d;
		}
		return count;
	}

	std::array<Dart, NB_MAX> tdarts_;
};

TEST_F(CMap2Test, testCMap2Constructor)
{
	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), 0u);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), 0u);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 0u);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), 0u);
}

TEST_F(CMap2Test, addFace)
{
	int n = randomFaces();

	EXPECT_EQ(cmap_.nb_cells<Vertex::ORBIT>(), n);
	EXPECT_EQ(cmap_.nb_cells<Edge::ORBIT>(), n);
	EXPECT_EQ(cmap_.nb_cells<Face::ORBIT>(), 2*NB_MAX);
	EXPECT_EQ(cmap_.nb_cells<Volume::ORBIT>(), NB_MAX);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

#undef NB_MAX

} // namespace cgogn

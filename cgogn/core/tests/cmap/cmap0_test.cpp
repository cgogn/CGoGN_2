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
#include <core/cmap/sanity_check.h>

namespace cgogn
{

#define NB_MAX 100

/*!
 * \brief The CMap0Test class implements tests on embedded CMap0
 * It contains a CMap0 to which a vertex attribute is added
 * to enforce the indexation mecanism in cell traversals.
 * Note that pure topological operations have already been tested,
 * thus only the indexation mecanism used for the embedding of cells
 * is tested here.
 */
class CMap0Test: public ::testing::Test
{

public:

	using testCMap0 = CMap0<DefaultMapTraits>;
	using VertexAttributeHandler = testCMap0::VertexAttributeHandler<int>;
	using Vertex = testCMap0::Vertex;

protected:

	testCMap0 cmap_;
	VertexAttributeHandler vertices_;

	/*!
	 * \brief A vector of darts on which the methods are tested.
	 */
	std::vector<Dart> darts_;

	/*!
	 * \brief Add a vertex attribute to the testing configuration
	 */
	CMap0Test()
	{
		std::srand(static_cast<unsigned int>(std::time(0)));
		vertices_ = cmap_.add_attribute<int, Vertex::ORBIT>("vertices");
	}

	/*!
	 * \brief Initialize the darts in darts_ with added vertices
	 * \param n : the number of added darts or vertices
	 */
	void addVertices(unsigned int n)
	{
		for (unsigned int i = 0; i < n; ++i)
			darts_.push_back(cmap_.add_vertex());
	}
};

/*!
 * \brief Adding vertices preserves the cell indexation
 */
TEST_F(CMap0Test, testAddVertex)
{
	addVertices(NB_MAX);
	EXPECT_TRUE(cmap_.check_map_integrity());
}

/*!
 * \brief Removing vertices preserves the cell indexation
 */
TEST_F(CMap0Test, testRemoveVertex)
{
	addVertices(NB_MAX);

	for (Dart d: darts_)
		if (std::rand()%3 == 1) cmap_.remove_vertex(Vertex(d));

	EXPECT_TRUE(cmap_.check_map_integrity());
}

} // namespace cgogn

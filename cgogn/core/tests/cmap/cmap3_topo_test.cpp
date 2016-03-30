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

#include <core/cmap/cmap3.h>
#include <core/cmap/cmap3_builder.h>

namespace cgogn
{

/*!
 * \brief The CMap3TopoTest class implements topological tests on CMap3
 * It derives from CMap3 to allow the test of protected methods
 *
 * Note that these tests, check that the topological operators perform as wanted
 * but do neither tests the containers (refs_, used_, etc.) or the iterators.
 * These last tests are implemented in another test suite.
 */
class CMap3TopoTest : public CMap3<DefaultMapTraits>, public ::testing::Test
{

public:

	using Inherit = CMap3<DefaultMapTraits>;
	using MapBuilder = CMap3Builder_T<DefaultMapTraits>;
	using Vertex = CMap3TopoTest::Vertex;
	using Edge   = CMap3TopoTest::Edge;
	using Face   = CMap3TopoTest::Face;
	using Volume   = CMap3TopoTest::Volume;
	using VertexMarker = CMap3TopoTest::CellMarker<Vertex::ORBIT>;

protected:


};

/*!
 * \brief Test pyramid creation.
 */
TEST_F(CMap3TopoTest, pyramid_test)
{
	Dart d1 = this->add_pyramid_topo(4);
	CMap3TopoTest::MapBuilder mbuild(*this);
	mbuild.close_map();

	EXPECT_TRUE(check_map_integrity());
	EXPECT_EQ(nb_darts(), 32u);

	Dart d2 = this->phi<32121213>(d1);
	EXPECT_EQ(d1,d2);
}


/*!
 * \brief Test pyramid creation.
 */
TEST_F(CMap3TopoTest, prism_test)
{
	Dart d1 = this->add_prism_topo(3);
	CMap3TopoTest::MapBuilder mbuild(*this);
	mbuild.close_map();

	EXPECT_TRUE(check_map_integrity());
	EXPECT_EQ(nb_darts(), 36u);

	d1 = this->phi<321>(d1);
	Dart d2 = this->phi<112112112>(d1);
	EXPECT_EQ(d1,d2);
}



} // namespace cgogn

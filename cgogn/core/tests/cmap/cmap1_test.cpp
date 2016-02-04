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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <core/cmap/cmap1.h>

namespace cgogn
{

// class CMap1TopoMock : public CMap1<DefaultMapTraits> {
// public:
//     MOCK_METHOD0( add_dart, Dart() );
//     MOCK_METHOD1( cut_edge_topo, Dart(Dart d) );
//     MOCK_METHOD1( add_face_topo, Dart(unsigned int nb_edges) );
// };

class CMap1TopoTest: public CMap1<DefaultMapTraits>, public ::testing::Test
{

	using CMap1 = cgogn::CMap1<DefaultMapTraits>;

public:

	CMap1 cmap_;
	CMap1::Face d_;

protected:

	CMap1TopoTest()
	{
		d_ = this->add_face_topo(10);
	}
};

TEST_F(CMap1TopoTest, testFaceDegree)
{
	EXPECT_EQ(10, this->degree(d_));
}

TEST_F(CMap1TopoTest, testCutEdge)
{
	Dart d1 = this->phi1(d_);
	Dart e = this->cut_edge_topo(d_);

	EXPECT_EQ(d1.index, this->phi1(e).index);
	EXPECT_EQ(d_.dart.index, this->phi_1(e).index);
	EXPECT_EQ(11, this->degree(d_));
}

TEST_F(CMap1TopoTest, testUncutEdge)
{
	Dart e = this->phi1(d_);
	Dart d1 = this->phi1(e);
	this->uncut_edge_topo(d_);

	EXPECT_EQ(d1.index, this->phi1(d_).index);
	EXPECT_EQ(9, this->degree(d_));
}

TEST_F(CMap1TopoTest, testCollapseEdge)
{
	Dart e = this->phi1(d_);
	Dart d1 = this->phi1(e);
	this->collapse_edge_topo(e);

	EXPECT_EQ(d1.index, this->phi1(d_).index);
	EXPECT_EQ(9, this->degree(d_));
}

TEST_F(CMap1TopoTest, testSplitFace)
{
	Dart e = this->phi1(this->phi1(this->phi1(d_)));
	this->split_face_topo(d_, e);

	EXPECT_EQ(3, this->degree(d_));
	EXPECT_EQ(7, this->degree(e));
}

// TEST_F(CMap1TopoTest, testDeleteFace)
// {
// 	this->delete_face_topo(d_);
//  	EXPECT_EQ(0, this->degree(d_));
// }

} // namespace cgogn

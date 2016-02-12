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


class CMap1TopoTest: public CMap1<DefaultMapTraits>, public ::testing::Test
{
public:
	typedef CMap1TopoTest::Face Face;

protected:

	CMap1TopoTest()
	{}
};


TEST_F(CMap1TopoTest, testAddDart)
{

}

TEST_F(CMap1TopoTest, testDeleteDart)
{

}

TEST_F(CMap1TopoTest, testFaceDegree)
{
	Dart d = this->add_face_topo(10);
	EXPECT_EQ(10, this->degree(Face(d)));
}

TEST_F(CMap1TopoTest, testCutEdge)
{
	Dart d = this->add_face_topo(10);
	Dart d1 = this->phi1(d);

	Dart e = this->cut_edge_topo(d);

	EXPECT_EQ(d1.index, this->phi1(e).index);
	EXPECT_EQ(d.index, this->phi_1(e).index);
	EXPECT_EQ(11, this->degree(Face(d)));
}

TEST_F(CMap1TopoTest, testCollapseEdge)
{
	Dart d = this->add_face_topo(10);
	Dart d_1 = this->phi_1(d);
	Dart d1 = this->phi1(d);

	this->collapse_edge_topo(d);

	EXPECT_EQ(d1.index, this->phi1(d_1).index);
	EXPECT_EQ(9, this->degree(Face(d_1)));
}

TEST_F(CMap1TopoTest, testReverseFace)
{
	Dart d = this->add_face_topo(10);
	std::vector<Dart> successors;

	{
		Dart dit = d;
		do
		{
			successors.push_back(this->phi1(dit));
			dit = this->phi1(dit);
		}
		while(dit != d);
	}

	this->reverse_face_topo(d);

	{
		Dart dit = d;
		unsigned i = 0;
		do
		{
			EXPECT_EQ(this->phi_1(dit).index, successors[i].index);
			dit = this->phi_1(dit);
			++i;
		}
		while(dit != d);
	}
}

TEST_F(CMap1TopoTest, testForEachDartOfVertex)
{

}

TEST_F(CMap1TopoTest, testForEachDartOfEdge)
{

}

TEST_F(CMap1TopoTest, testForEachDartOfFace)
{

}

} // namespace cgogn

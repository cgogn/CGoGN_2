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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <core/cmap/cmap1.h>

namespace cgogn
{


class CMap1TopoTest: public CMap1<DefaultMapTraits>, public ::testing::Test
{

public:

	static const int NB_MAX = 1000;

	using Vertex = CMap1TopoTest::Vertex;
	using Face   = CMap1TopoTest::Face;

protected:

	/*!
	 * \brief Generate a random set of faces.
	*/
	CMap1TopoTest() : nb_darts_(0)
	{
		std::srand(static_cast<unsigned int>(std::time(0)));
	}

	bool dartIntegrity(Dart d) {
		return (phi1(phi_1(d)) == d && phi_1(phi1(d)) == d);
	}

	bool mapIntegrity() {
		bool result = true;
		foreach_dart( [&] (Dart d) {
			if (!dartIntegrity(d)) {
				result = false;
				return ;
			}
		});
		return result;
	}

	int randomDarts() {
		int n = std::rand() % NB_MAX;
		for (int i = 0; i < n; ++i)
			tdarts_[i] = add_dart();

		return n;
	}

	int randomFaces() {
		int count = 0;
		for (int i = 0; i < NB_MAX; ++i) {
			int n = std::rand() % 100;
			Dart d = add_face_topo(n);
			count += n;

			for (int k = std::rand() % n; k > 0; ++k)
				d = phi1(d);

			tdarts_[i] = d;
		}
		return count;
	}

	std::array<Dart, NB_MAX> tdarts_;
	int nb_darts_;
};


TEST_F(CMap1TopoTest, testAddDart)
{
	int n = randomDarts();
	EXPECT_EQ(nb_darts(), n);
	EXPECT_TRUE(mapIntegrity());
}

TEST_F(CMap1TopoTest, testDeleteDart)
{
	int n = randomDarts();
	int count = n;
	for (int i = 0; i < n; ++i) {
		if (std::rand() % 3 == 1) {
			remove_dart(tdarts_[i]);
			--count;
		}
	}
	EXPECT_EQ(nb_darts(), count);
	EXPECT_TRUE(mapIntegrity());
}

TEST_F(CMap1TopoTest, testPhi1SewUnSew)
{
	int n = randomDarts();
	for (int i = 0; i < 1000; ++i) {
		Dart d = tdarts_[std::rand() % n];
		Dart e = tdarts_[std::rand() % n];
		Dart f = tdarts_[std::rand() % n];
		Dart nd = phi1(d);
		Dart ne = phi1(e);
		phi1_sew(d, e);
		EXPECT_TRUE(phi1(d) == ne);
		EXPECT_TRUE(phi1(e) == nd);
		Dart nf1 = phi1(f);
		Dart nf2 = phi1(nf1);
		phi1_unsew(f);
		EXPECT_TRUE(phi1(nf1) == nf1);
		EXPECT_TRUE(phi1(f) == nf2);
	}
	EXPECT_EQ(nb_darts(), n);
	EXPECT_TRUE(mapIntegrity());
}

TEST_F(CMap1TopoTest, testFaceDegree)
{
	Dart d = this->add_face_topo(10);
	EXPECT_EQ(10, this->degree(Face(d)));
}

TEST_F(CMap1TopoTest, testSplitVertex)
{
	Dart d = this->add_face_topo(10);
	Dart d1 = this->phi1(d);

	Dart e = this->split_vertex_topo(d);

	EXPECT_EQ(d1.index, this->phi1(e).index);
	EXPECT_EQ(d.index, this->phi_1(e).index);
	EXPECT_EQ(11, this->degree(Face(d)));
}

TEST_F(CMap1TopoTest, testRemoveVertex)
{
	Dart d = this->add_face_topo(10);
	Dart d_1 = this->phi_1(d);
	Dart d1 = this->phi1(d);

	this->remove_vertex(d);

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

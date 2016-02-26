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

#include <core/cmap/cmap2.h>

namespace cgogn
{

#define NB_MAX 1000

class CMap2TopoTest: public CMap2<DefaultMapTraits>, public ::testing::Test
{

public:

	using Vertex = CMap2TopoTest::Vertex;
	using Face   = CMap2TopoTest::Face;

protected:

	/*!
	 * \brief An array of randomly genrated darts on which the methods are tested.
	 */
	std::array<Dart, NB_MAX> tdarts_;

	/*!
	 * \brief Generate a random set of faces.
	*/
	CMap2TopoTest()
	{
		std::srand(static_cast<unsigned int>(std::time(0)));
	}

	int randomDarts() {
		int n = 1 + std::rand() % (NB_MAX-1);
		for (int i = 0; i < n; ++i)
			tdarts_[i] = add_dart();

		return n;
	}

	int randomFaces() {
		int count = 0;
		for (int i = 0; i < NB_MAX; ++i) {
			int n = 1 + std::rand() % 100;
			Dart d = add_face_topo(n);
			count += n;

			while (std::rand()%10 != 1)
				d = phi1(d);

			tdarts_[i] = d;
		}
		return count;
	}
};

TEST_F(CMap2TopoTest, testCMap2Constructor)
{
	EXPECT_EQ(nb_darts(), 0u);
	EXPECT_EQ(this->template nb_cells<Vertex::ORBIT>(), 0u);
	EXPECT_EQ(this->template nb_cells<Edge::ORBIT>(), 0u);
	EXPECT_EQ(this->template nb_cells<Face::ORBIT>(), 0u);
	EXPECT_EQ(this->template nb_cells<Volume::ORBIT>(), 0u);
}

TEST_F(CMap2TopoTest, testAddDart)
{
	int n = randomDarts();

	EXPECT_EQ(nb_darts(), n);
	EXPECT_TRUE(check_map_integrity());
}

TEST_F(CMap2TopoTest, testDeleteDart)
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
	EXPECT_TRUE(check_map_integrity());
}

TEST_F(CMap2TopoTest, testPhi2SewUnSew)
{
	int n = randomDarts();

	for (int i = 0; i < 1000; ++i) {
		Dart d0 = tdarts_[std::rand() % n];
		Dart d2 = phi2(d0);
		if (d0 != d2) {
			phi2_unsew(d0);
			EXPECT_FALSE(phi2(d0) == d2);
			EXPECT_TRUE(phi2(d0) == d0);
			EXPECT_TRUE(phi2(d2) == d2);
		}
		Dart e0 = d0;
		while (e0 == d0) e0 = tdarts_[std::rand() % n];
		EXPECT_FALSE(d0 == e0);
		phi2_unsew(e0);
		EXPECT_TRUE(phi2(e0) == e0);

		phi2_sew(d0,e0);
		EXPECT_TRUE(phi2(d0) == e0);
		EXPECT_TRUE(phi2(e0) == d0);
	}

	EXPECT_EQ(nb_darts(), n);
	EXPECT_TRUE(check_map_integrity());
}

TEST_F(CMap2TopoTest, testAddFace)
{
	int n = randomFaces();

	EXPECT_EQ(this->template nb_cells<Vertex::ORBIT>(), n);
	EXPECT_EQ(this->template nb_cells<Edge::ORBIT>(), n);
	EXPECT_EQ(this->template nb_cells<Face::ORBIT>(), 2*NB_MAX);
	EXPECT_EQ(this->template nb_cells<Volume::ORBIT>(), NB_MAX);
	EXPECT_TRUE(check_map_integrity());
}

TEST_F(CMap2TopoTest, testCutEdge)
{
	int n = randomFaces();

	for (int i = 0; i < NB_MAX; ++i) {
		Dart d = tdarts_[i];
		unsigned int k = degree(Face(d));
		cut_edge_topo(d);
		EXPECT_EQ(degree(Face(d)), k+1);
	}

	EXPECT_EQ(this->template nb_cells<Vertex::ORBIT>(), n+NB_MAX);
	EXPECT_EQ(this->template nb_cells<Edge::ORBIT>(), n+NB_MAX);
	EXPECT_EQ(this->template nb_cells<Face::ORBIT>(), 2*NB_MAX);
	EXPECT_EQ(this->template nb_cells<Volume::ORBIT>(), NB_MAX);
	EXPECT_TRUE(check_map_integrity());
}

TEST_F(CMap2TopoTest, testCutFace)
{
	int n = randomFaces();

	int countEdges = n;
	int countFaces = 2*NB_MAX;

	for (int i = 0; i < NB_MAX; ++i) {
		Dart d = tdarts_[i];
		Dart e = d;
		while (std::rand()%10 != 1) e = phi1(e);
		if (e == d) e = phi1(e);

		unsigned int k = degree(Face(d));
		if (k>1) {
			cut_face_topo(d, e);
			++countEdges;
			++countFaces;
			EXPECT_EQ(degree(Face(d))+degree(Face(e)), k+2);
		}
	}

	EXPECT_EQ(this->template nb_cells<Vertex::ORBIT>(), n);
	EXPECT_EQ(this->template nb_cells<Edge::ORBIT>(), countEdges);
	EXPECT_EQ(this->template nb_cells<Face::ORBIT>(), countFaces);
	EXPECT_EQ(this->template nb_cells<Volume::ORBIT>(), NB_MAX);
	EXPECT_TRUE(check_map_integrity());
}

TEST_F(CMap2TopoTest, testFaceDegree)
{
	Face f = this->add_face_topo(10);

	EXPECT_EQ(degree(f),10);
}

// The traversal methods are tested through the nb_cells calls and wihtin other methods

} // namespace cgogn

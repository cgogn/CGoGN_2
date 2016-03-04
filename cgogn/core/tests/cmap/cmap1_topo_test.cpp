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

namespace cgogn
{

#define NB_MAX 1000

class CMap1TopoTest: public CMap1<DefaultMapTraits>, public ::testing::Test
{

public:

	using Vertex = CMap1TopoTest::Vertex;
	using Face   = CMap1TopoTest::Face;

protected:

	/*!
	 * \brief An array of randomly genrated darts on which the methods are tested.
	 */
	std::array<Dart, NB_MAX> tdarts_;

	/*!
	 * \brief Generate a random set of faces.
	*/
	CMap1TopoTest()
	{
		std::srand(static_cast<unsigned int>(std::time(0)));
	}

	bool dartIntegrity(Dart d) {
		return (phi1(phi_1(d)) == d &&
				phi_1(phi1(d)) == d);
	}

	bool mapIntegrity() {
		bool result = true;
		foreach_dart_until( [&] (Dart d) {
			if (!dartIntegrity(d)) {
				result = false;
			}
			return result;
		});
		return result;
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

TEST_F(CMap1TopoTest, testCMap1Constructor)
{
	EXPECT_EQ(nb_darts(), 0u);
	EXPECT_EQ(this->template nb_cells<Vertex::ORBIT>(), 0u);
	EXPECT_EQ(this->template nb_cells<Face::ORBIT>(), 0u);
}

TEST_F(CMap1TopoTest, testAddDart)
{
	int n = randomDarts();
	(*phi1_)[tdarts_[n-1].index] = Dart(1024);

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

TEST_F(CMap1TopoTest, testAddFace)
{
	int n = randomFaces();

	EXPECT_EQ(this->template nb_cells<Vertex::ORBIT>(), n);
	EXPECT_EQ(this->template nb_cells<Face::ORBIT>(), NB_MAX);
	EXPECT_TRUE(mapIntegrity());
}

TEST_F(CMap1TopoTest, testSplitVertex)
{
	int n = randomFaces();

	for (int i = 0; i < NB_MAX; ++i) {
		Face d = tdarts_[i];
		unsigned int k = degree(Face(d));
		split_vertex_topo(d);
		EXPECT_EQ(degree(Face(d)), k+1);
	}

	EXPECT_EQ(this->template nb_cells<Vertex::ORBIT>(), n+NB_MAX);
	EXPECT_EQ(this->template nb_cells<Face::ORBIT>(), NB_MAX);
	EXPECT_TRUE(mapIntegrity());
}

TEST_F(CMap1TopoTest, testRemoveVertex)
{
	int n = randomFaces();

	int countVertex = n;
	int countFace = NB_MAX;
	for (int i = 0; i < NB_MAX; ++i) {
		Face d = tdarts_[i];
		unsigned int k = degree(Face(d));
		if (k > 1) {
			Dart e = phi1(d);
			remove_vertex(Vertex(d));
			--countVertex;
			EXPECT_EQ(degree(Face(e)), k-1);
		}
		else {
			remove_vertex(Vertex(d));
			--countFace;
			--countVertex;
		}
	}

	EXPECT_EQ(this->template nb_cells<Vertex::ORBIT>(), countVertex);
	EXPECT_EQ(this->template nb_cells<Face::ORBIT>(), countFace);
	EXPECT_TRUE(mapIntegrity());
}

TEST_F(CMap1TopoTest, testRemoveFace)
{
	int n = randomFaces();

	int countVertex = n;
	int countFace = NB_MAX;
	for (int i = 0; i < NB_MAX; ++i) {
		Face d = tdarts_[i];
		unsigned int k = degree(Face(d));
		remove_face(d);
		countVertex -= k;
		--countFace;
	}

	EXPECT_EQ(this->template nb_cells<Vertex::ORBIT>(), countVertex);
	EXPECT_EQ(this->template nb_cells<Face::ORBIT>(), countFace);
	EXPECT_TRUE(mapIntegrity());
}

TEST_F(CMap1TopoTest, testReverseFace)
{
	int n = randomFaces();
	EXPECT_TRUE(n > 0); // avoid warning unused var n

	for (int i = 0; i < NB_MAX; ++i) {
		Face f = tdarts_[i];
		unsigned int k = degree(f);

		std::vector<Dart> face_darts;
		foreach_dart_of_orbit(f, [&] (Dart d) {
			face_darts.push_back(d);
		});

		reverse_face_topo(tdarts_[i]);
		EXPECT_EQ(degree(Face(tdarts_[i])), k);

		f = phi1(f);
		foreach_dart_of_orbit(f, [&] (Dart d) {
			EXPECT_TRUE(face_darts.back() == d);
			face_darts.pop_back();
		});
		EXPECT_TRUE(face_darts.empty());
	}
}

TEST_F(CMap1TopoTest, testDegree)
{
	Face f = this->add_face_topo(10);

	EXPECT_EQ(degree(f),10);
}

TEST_F(CMap1TopoTest, testHasDegree)
{
	Face f = this->add_face_topo(10);

	EXPECT_TRUE(has_degree(f,10));
	EXPECT_FALSE(has_degree(f,0));
	EXPECT_FALSE(has_degree(f,9));
	EXPECT_FALSE(has_degree(f,11));
}

// The traversal methods are tested through the nb_cells calls and wihtin other methods

} // namespace cgogn

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

#include <core/cmap/cmap1.h>

namespace cgogn
{

#define NB_MAX 1000

/*!
* \brief The CMap1TopoTest class implements topological tests on CMap1
* It derives from CMap1 to allow the test of protected methods
*/
class CMap1TopoTest : public CMap1<DefaultMapTraits>, public ::testing::Test
{

public:

	using Vertex = CMap1TopoTest::Vertex;
	using Face   = CMap1TopoTest::Face;

protected:

	/*!
	 * \brief An array of darts on which the methods are tested.
	 */
	std::array<Dart, NB_MAX> tdarts_;

	CMap1TopoTest()
	{
		std::srand(static_cast<unsigned int>(std::time(0)));
	}

	/*!
	 * \brief Initialize the darts in tdarts_
	 * \return The number of added darts or vertices
	 */
	unsigned int addVertices() {
		for (unsigned int i = 0; i < NB_MAX; ++i)
			tdarts_[i] = add_dart();

		return NB_MAX;
	}

	/*!
	 * \brief Generate a random set of faces and put them in tdarts_
	 * \return The number of added darts or vertices in all faces.
	 * The face size ranges from 1 to 100.
	 * A random dart of each face is put in the tdarts_ array.
	 */
	unsigned int addFaces() {
		unsigned int count = 0;
		for (unsigned int i = 0; i < NB_MAX; ++i) {
			unsigned int n = 1 + std::rand() % 100;
			Dart d = add_face_topo(n);
			count += n;

			while (std::rand()%10 != 1)
				d = phi1(d);

			tdarts_[i] = d;
		}
		return count;
	}
};

/*!
 * \brief An empty CMap1 contains no dart, no vertex and no face.
 */
TEST_F(CMap1TopoTest, testCMap1Constructor)
{
	EXPECT_EQ(nb_darts(), 0u);
	EXPECT_EQ(this->template nb_cells<Vertex::ORBIT>(), 0u);
	EXPECT_EQ(this->template nb_cells<Face::ORBIT>(), 0u);
}

/*!
 * \brief Adding darts adds one vertex and one face per dart.
 * The test adds NB_MAX darts.
 * The number of cells correctly increases and the map integrity is preserved.
 */
TEST_F(CMap1TopoTest, testAddDart)
{
	for (unsigned int i = 1; i< NB_MAX; ++i) {
		add_dart();
		EXPECT_EQ(nb_darts(), i);
		EXPECT_EQ(nb_cells<Vertex::ORBIT>(), i);
		EXPECT_EQ(nb_cells<Face::ORBIT>(), i);
	}
	EXPECT_TRUE(check_map_integrity());
}

/*!
 * \brief Removing unsewn darts removes one vertex and on face per dart.
 * The test randomly removes 1/3 of the initial vertices.
 * The number of cells correctly decreases and the map integrity is preserved.
 */
TEST_F(CMap1TopoTest, testRemoveDart)
{
	unsigned int n = addVertices();

	unsigned int count = n;
	for (unsigned int i = 0; i < n; ++i) {
		if (std::rand() % 3 == 1) {
			remove_dart(tdarts_[i]);
			--count;
			EXPECT_EQ(nb_darts(), count);
			EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count);
			EXPECT_EQ(nb_cells<Face::ORBIT>(), count);
		}
	}
	EXPECT_TRUE(check_map_integrity());
}

/*!
 * \brief Sewing and unsewing darts correctly changes the topological relations.
 * The test perfoms NB_MAX sewing and unsewing on randomly chosen dart of tdarts_.
 * The number of vertices is unchanged and the map integrity is preserved.
 */
TEST_F(CMap1TopoTest, testPhi1SewUnSew)
{
	unsigned int n = addVertices();

	for (unsigned int i = 0; i < NB_MAX; ++i) {
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
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), n);
	EXPECT_TRUE(check_map_integrity());
}

/*!
 * \brief Adding a face of size n adds n darts, n vertices and 1 face.
 * The test performs NB_MAX additions of randomly sized faces.
 * The number of generated cells is correct and the map integrity is preserved.
 */
TEST_F(CMap1TopoTest, testAddFace)
{
	unsigned int count = 0;

	for (unsigned int i = 0; i < NB_MAX; ++i) {
		unsigned int n = 1 + std::rand() % 100;
		Dart d = add_face_topo(n);
		count += n;
		EXPECT_EQ(nb_darts(), count);
		EXPECT_EQ(nb_cells<Vertex::ORBIT>(), count);
		EXPECT_EQ(nb_cells<Face::ORBIT>(), i+1);
	}
	EXPECT_TRUE(check_map_integrity());
}

/*! \brief Spliting a vertex increases the size of its face.
 * The test performs NB_MAX vertex spliting on vertices of randomly generated faces.
 * The number of generated cells is correct and the map integrity is preserved.
 */
TEST_F(CMap1TopoTest, testSplitVertex)
{
	unsigned int n = addFaces();

	for (unsigned int i = 0; i < NB_MAX; ++i) {
		Face d = tdarts_[i];
		unsigned int k = degree(Face(d));
		split_vertex_topo(d);
		EXPECT_EQ(degree(Face(d)), k+1);
		EXPECT_EQ(nb_cells<Vertex::ORBIT>(), n+i+1);
		EXPECT_EQ(nb_cells<Face::ORBIT>(), NB_MAX);
	}
	EXPECT_TRUE(check_map_integrity());
}

/*! \brief Removing a vertex decreases the size of its face and removes the face if its size is 1.
* The test performs NB_MAX vertex spliting on vertices of randomly generated faces.
* The number of generated cells is correct and the map integrity is preserved.
*/
TEST_F(CMap1TopoTest, testRemoveVertex)
{
	unsigned int n = addFaces();

	unsigned int countVertices = n;
	unsigned int countFaces = NB_MAX;
	for (unsigned int i = 0; i < NB_MAX; ++i) {
		Face d = tdarts_[i];
		unsigned int k = degree(Face(d));
		if (k > 1) {
			Dart e = phi1(d);
			remove_vertex(Vertex(d));
			--countVertices;
			EXPECT_EQ(degree(Face(e)), k-1);
		}
		else {
			remove_vertex(Vertex(d));
			--countFaces;
			--countVertices;
		}
	}

	EXPECT_EQ(this->template nb_cells<Vertex::ORBIT>(), countVertices);
	EXPECT_EQ(this->template nb_cells<Face::ORBIT>(), countFaces);
	EXPECT_TRUE(check_map_integrity());
}

TEST_F(CMap1TopoTest, testRemoveFace)
{
	int n = addFaces();

	int countVertices = n;
	int countFaces = NB_MAX;
	for (int i = 0; i < NB_MAX; ++i) {
		Face d = tdarts_[i];
		unsigned int k = degree(Face(d));
		remove_face(d);
		countVertices -= k;
		--countFaces;
	}

	EXPECT_EQ(this->template nb_cells<Vertex::ORBIT>(), countVertices);
	EXPECT_EQ(this->template nb_cells<Face::ORBIT>(), countFaces);
	EXPECT_TRUE(check_map_integrity());
}

TEST_F(CMap1TopoTest, testReverseFace)
{
	int n = addFaces();

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
	EXPECT_TRUE(check_map_integrity());
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

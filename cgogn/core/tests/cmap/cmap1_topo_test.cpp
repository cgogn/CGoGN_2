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

#define NB_MAX 100

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
	 * \brief A vector of darts on which the methods are tested.
	 */
	std::vector<Dart> darts_;

	CMap1TopoTest()
	{
		std::srand(static_cast<unsigned int>(std::time(0)));
	}

	/*!
	 * \brief Initialize the darts in darts_ with added vertices
	 * \param n : the number of added darts or vertices
	 */
	void addVertices(unsigned int n)
	{
		for (unsigned int i = 0; i < n; ++i)
			darts_.push_back(add_dart());
	}

	/*!
	 * \brief Generate a random set of faces and put them in darts_
	 * \return The total number of added darts or vertices.
	 * The face size ranges from 1 to 10.
	 * A random dart of each face is put in the darts_ array.
	 */
	unsigned int addFaces(unsigned int n)
	{
		unsigned int count = 0;
		for (unsigned int i = 0; i < n; ++i)
		{
			unsigned int n = 1 + std::rand() % 10;
			Dart d = add_face_topo(n);
			count += n;

			while (std::rand()%10 != 1)
				d = phi1(d);

			darts_.push_back(d);
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
 * The test adds darts and check that the number of cells correctly
 * increases and that the map integrity is preserved.
 */
TEST_F(CMap1TopoTest, testAddDart)
{
	add_dart();
	EXPECT_EQ(nb_darts(), 1);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), 1);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), 1);

	addVertices(NB_MAX);
	EXPECT_EQ(nb_darts(), NB_MAX+1);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), NB_MAX+1);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), NB_MAX+1);

	EXPECT_TRUE(check_map_integrity());
}

/*!
 * \brief Removing unsewn darts removes one vertex and on face per dart.
 * The test randomly removes 1/3 of the initial vertices.
 * The number of cells correctly decreases and the map integrity is preserved.
 */
TEST_F(CMap1TopoTest, testRemoveDart)
{
	addVertices(NB_MAX);
	int countVertices = NB_MAX;

	remove_dart(darts_.back());
	--countVertices;
	EXPECT_EQ(nb_darts(), countVertices);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), countVertices);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), countVertices);

	darts_.pop_back();
	for (Dart d: darts_)
	{
		if (std::rand()%3 == 1)
		{
			remove_dart(d);
			--countVertices;
		}
	}
	EXPECT_EQ(nb_darts(), countVertices);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), countVertices);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), countVertices);
	EXPECT_TRUE(check_map_integrity());
}

/*!
 * \brief Sewing and unsewing darts correctly changes the topological relations.
 * The test perfoms NB_MAX sewing and unsewing on randomly chosen dart of tdarts_.
 * The number of vertices is unchanged, the number of faces changes correctly
 * and the map integrity is preserved.
 */
TEST_F(CMap1TopoTest, testPhi1SewUnSew)
{
	addVertices(NB_MAX);
	int countFaces = NB_MAX;

	for (unsigned int i = 0; i < NB_MAX; ++i) {
		Dart d = darts_[std::rand() % NB_MAX];
		Dart e = darts_[std::rand() % NB_MAX];
		Dart f = darts_[std::rand() % NB_MAX];
		Dart nd = phi1(d);
		Dart ne = phi1(e);
		if (d != e) {
			if (same_cell(Face(d),Face(e)))
				++countFaces;
			else
				--countFaces;
		}
		phi1_sew(d, e);
		EXPECT_TRUE(phi1(d) == ne);
		EXPECT_TRUE(phi1(e) == nd);
		Dart nf1 = phi1(f);
		Dart nf2 = phi1(nf1);
		phi1_unsew(f);
		if (f != nf1) ++countFaces;
		EXPECT_TRUE(phi1(nf1) == nf1);
		EXPECT_TRUE(phi1(f) == nf2);
	}

	EXPECT_EQ(nb_darts(), NB_MAX);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), NB_MAX);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), countFaces);
	EXPECT_TRUE(check_map_integrity());
}

/*!
 * \brief Adding a face of size n adds n darts, n vertices and 1 face.
 * The test adds some faces and check that the number of generated cells is correct
 * and that the map integrity is preserved.
 */
TEST_F(CMap1TopoTest, testAddFace)
{
	add_face_topo(1);
	EXPECT_EQ(nb_darts(), 1);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), 1);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), 1);

	add_face_topo(10);
	EXPECT_EQ(nb_darts(), 11);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), 11);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), 2);

	unsigned int countVertices = 11 + addFaces(NB_MAX);

	EXPECT_EQ(nb_darts(), countVertices);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), countVertices);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), NB_MAX+2);
	EXPECT_TRUE(check_map_integrity());
}

/*! \brief Spliting a vertex increases the size of its face.
 * The test performs NB_MAX vertex spliting on vertices of randomly generated faces.
 * The number of generated cells is correct and the map integrity is preserved.
 */
TEST_F(CMap1TopoTest, testSplitVertex)
{
	unsigned int countVertices = addFaces(NB_MAX);

	for (Dart d: darts_)
	{
		unsigned int k = degree(Face(d));
		split_vertex_topo(d);
		EXPECT_EQ(degree(Face(d)), k+1);
	}
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), countVertices+NB_MAX);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), NB_MAX);
	EXPECT_TRUE(check_map_integrity());
}

/*! \brief Removing a vertex decreases the size of its face and removes the face if its size is 1.
* The test performs NB_MAX vertex spliting on vertices of randomly generated faces.
* The number of generated cells is correct and the map integrity is preserved.
*/
TEST_F(CMap1TopoTest, testRemoveVertex)
{
	unsigned int countVertices = addFaces(NB_MAX);
	unsigned int countFaces = NB_MAX;

	for (Dart d: darts_)
	{
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

	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), countVertices);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), countFaces);
	EXPECT_TRUE(check_map_integrity());
}

/*! \brief Removing a face removes all its vertices.
 * The test randomly removes 1/3 of the initial faces.
 * The number of cells correctly decreases and the map integrity is preserved.
 */
TEST_F(CMap1TopoTest, testRemoveFace)
{
	unsigned int countVertices = addFaces(NB_MAX);
	unsigned int countFaces = NB_MAX;

	for (Dart d: darts_)
	{
		unsigned int k = degree(Face(d));
		remove_face(d);
		countVertices -= k;
		--countFaces;
	}

	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), countVertices);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), countFaces);
	EXPECT_TRUE(check_map_integrity());
}

/*! \brief Reversing a face reverses the order of its vertices.
 * The test reverses randomly generated faces.
 * The number of faces and their degrees do not change and the map integrity is preserved.
 */
TEST_F(CMap1TopoTest, testReverseFace)
{
	unsigned int countVertices = addFaces(NB_MAX);

	for (Dart d: darts_)
	{
		unsigned int k = degree(Face(d));

		std::vector<Dart> face_darts;
		foreach_dart_of_orbit(Face(d), [&] (Dart e) {
			face_darts.push_back(e);
		});

		reverse_face_topo(d);
		EXPECT_EQ(degree(Face(d)), k);

		d = phi1(d);
		foreach_dart_of_orbit(Face(d), [&] (Dart e) {
			EXPECT_TRUE(face_darts.back() == e);
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

} // namespace cgogn

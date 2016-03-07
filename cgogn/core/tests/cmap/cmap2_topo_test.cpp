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

#define NB_MAX 1000

/*!
 * \brief The CMap2TopoTest class implements topological tests on CMap2
 * It derives from CMap2 to allow the test of protected methods
 *
 * Note that these tests, check that the topological operators perform as wanted
 * but do neither tests the containers (refs_, used_, etc.) or the iterators.
 * These last tests are implemented in another test suite.
 */
class CMap2TopoTest: public CMap2<DefaultMapTraits>, public ::testing::Test
{

public:

	using Inherit = CMap2<DefaultMapTraits>;
	using Vertex = CMap2TopoTest::Vertex;
	using Edge   = CMap2TopoTest::Edge;
	using Face   = CMap2TopoTest::Face;
	using Volume   = CMap2TopoTest::Volume;

protected:

	/*!
	 * \brief A vector of darts on which the methods are tested.
	 */
	std::vector<Dart> darts_;

	/*!
	 * \brief Generate a random set of faces.
	*/
	CMap2TopoTest()
	{
		std::srand(static_cast<unsigned int>(std::time(0)));
	}

	/*!
	 * \brief Generate a random set of faces and put them in darts_
	 * \return The total number of added vertices.
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

			n = std::rand() % 10;
			while (n-- > 0)	d = phi1(d);

			darts_.push_back(d);
		}
		return count;
	}

	/*!
	 * \brief Generate a closed surface from the set of faces in darts_
	 */
	void makeSurface()
	{
		unsigned int n = 0;

		// Generate NB_MAX random 1-faces (with no boundary)
		for (unsigned int i = 0; i < NB_MAX; ++i)
		{
			n = 1 + std::rand() % 10;
			Dart d = Inherit::Inherit::add_face_topo(n);
			darts_.push_back(d);
		}
		// Sew some pairs off 1-egdes
		for (unsigned int i = 0; i < 3*NB_MAX; ++i) {
			Dart e1 = darts_[std::rand() % NB_MAX];
			n = std::rand() % 10;
			while (n-- > 0)	e1 = phi1(e1);
			Dart e2 = darts_[std::rand() % NB_MAX];
			n = std::rand() % 10;
			while (n-- > 0)	e2 = phi1(e2);

			foreach_dart_of_orbit_until(Face(e1), [&] (Dart d) {
				if (phi2(d) == d) {
					if (phi2(e2) == e2 && e2 != d) {
						phi2_sew(e2, d);
						return (std::rand()%3 == 1);
					}
					else
						return false;
				}
				else
					return false;
			});
		}
		close_map();
	}
};

/*!
 * \brief An empty CMap2 contains no dart and no cells.
 */
TEST_F(CMap2TopoTest, Constructor)
{
	EXPECT_EQ(nb_darts(), 0u);
	EXPECT_EQ(this->template nb_cells<Vertex::ORBIT>(), 0u);
	EXPECT_EQ(this->template nb_cells<Edge::ORBIT>(), 0u);
	EXPECT_EQ(this->template nb_cells<Face::ORBIT>(), 0u);
	EXPECT_EQ(this->template nb_cells<Volume::ORBIT>(), 0u);
}

/*!
 * \brief Sewing and unsewing darts correctly changes the topological relations.
 * The test perfoms NB_MAX sewing and unsewing on randomly chosen dart of darts_.
 * The map integrity is not preserved (this test creates fixed points for PHI2).
 */
TEST_F(CMap2TopoTest, phi2_sew_unsew)
{
	unsigned int countVertices = addFaces(NB_MAX);
	unsigned int countFaces = NB_MAX;
	unsigned int countVolumes = NB_MAX;

	for (int i = 0; i < NB_MAX; ++i) {
		Dart d0 = darts_[std::rand() % NB_MAX];
		Dart d2 = phi2(d0);
		phi2_unsew(d0);
		EXPECT_TRUE(phi2(d0) == d0);
		EXPECT_TRUE(phi2(d2) == d2);
		Dart e0 = d0;
		while (e0 == d0) e0 = darts_[std::rand() % NB_MAX];
		phi2_unsew(e0);

		phi2_sew(d0,e0);
		EXPECT_TRUE(phi2(d0) == e0);
		EXPECT_TRUE(phi2(e0) == d0);
	}
}

/*!
 * \brief Adding a 2-face of size n adds 2*n darts, n vertices and edges, 2 1-faces and 1 volume.
 * The test adds some faces and check that the number of generated cells is correct
 * and that the map integrity is preserved.
 */
TEST_F(CMap2TopoTest, add_face_topo)
{
	add_face_topo(1);
	EXPECT_EQ(nb_darts(), 2);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), 1);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), 1);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), 2);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), 1);

	add_face_topo(10);
	EXPECT_EQ(nb_darts(), 22);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), 11);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), 11);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), 4);
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), 2);

	unsigned int countVertices = 11 + addFaces(NB_MAX);

	EXPECT_EQ(nb_darts(), 2*countVertices);
	EXPECT_EQ(nb_cells<Vertex::ORBIT>(), countVertices);
	EXPECT_EQ(nb_cells<Edge::ORBIT>(), countVertices);
	EXPECT_EQ(nb_cells<Face::ORBIT>(), 2*(NB_MAX+2));
	EXPECT_EQ(nb_cells<Volume::ORBIT>(), NB_MAX+2);
	EXPECT_TRUE(check_map_integrity());
}

/*! \brief Cutting an edge increases the size of both incident faces and add a vertex of degree 2.
 * The test performs NB_MAX edge cutting on edges of randomly generated faces.
 * The number of generated cells is correct and the map integrity is preserved.
 */
TEST_F(CMap2TopoTest, testCutEdge)
{
	int n = addFaces(NB_MAX);

	for (int i = 0; i < NB_MAX; ++i) {
		Dart d = darts_[i];
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
	int n = addFaces(NB_MAX);

	int countEdges = n;
	int countFaces = 2*NB_MAX;

	for (int i = 0; i < NB_MAX; ++i) {
		Dart d = darts_[i];
		Dart e = d;
		unsigned int j = std::rand() % 10;
		while (j-- > 0)	e = phi1(e);
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

#undef NB_MAX

} // namespace cgogn

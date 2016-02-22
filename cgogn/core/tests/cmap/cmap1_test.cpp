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
#include <core/cmap/map_traits.h>
#include <core/cmap/sanity_check.h>

namespace cgogn
{

class CMap1Test: public ::testing::Test
{

public:
	typedef CMap1<DefaultMapTraits> myCMap1;
	typedef myCMap1::Vertex Vertex;
	typedef myCMap1::Face Face;

protected:
	myCMap1 cmap_;

	CMap1Test()
	{}


};

TEST_F(CMap1Test, addFace)
{
	Face f = cmap_.add_face(10);

	cmap_.split_vertex(Vertex(f.dart));

	EXPECT_TRUE(is_well_embedded<Face::SELF_ORBIT>(cmap_));
	EXPECT_TRUE(is_orbit_embedding_unique<Face::SELF_ORBIT>(cmap_));
	EXPECT_TRUE(is_container_well_referenced<Face::SELF_ORBIT>(cmap_));

}

} // namespace cgogn

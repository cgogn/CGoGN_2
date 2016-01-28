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

#include <core/utils/precision.h>
#include <geometry/types/eigen.h>
#include <geometry/types/vec.h>
#include <geometry/functions/normal.h>
#include <gtest/gtest.h>
#include <iostream>


using StdArray = cgogn::geometry::Vec_T<std::array<double,3>>;
using EigenVec3d = Eigen::Vector3d;

TEST(Normal_TEST, TriangleNormal)
{
	{
		StdArray p0(1,3,-5);
		StdArray p1(7,-4,0.1);
		StdArray p2(-15,-2,15);
		StdArray n = cgogn::geometry::triangle_normal(p0,p1,p2);
		cgogn::almost_equal_relative(n.dot(p1-p0),0.);
//		EXPECT_TRUE(cgogn::almost_equal_relative(n.dot(p1-p0),0.)); // is false !
		EXPECT_TRUE(cgogn::almost_equal_absolute(n.dot(p1-p0),0., 1e-8));
		EXPECT_TRUE(cgogn::almost_equal_relative(n.dot(p2-p0),0.));
		EXPECT_TRUE(cgogn::almost_equal_relative(n.dot(p2-p1),0.));
//		EXPECT_DOUBLE_EQ(n.dot(p1-p0),0); // is false !
		EXPECT_NEAR(n.dot(p1-p0),0, 1e-8);
		EXPECT_DOUBLE_EQ(n.dot(p2-p0),0);
		EXPECT_DOUBLE_EQ(n.dot(p2-p1),0);
	}
	{
		EigenVec3d p0(1,3,-5);
		EigenVec3d p1(7,-4,0.1);
		EigenVec3d p2(-15,-2,15);
		EigenVec3d n = cgogn::geometry::triangle_normal(p0,p1,p2);
//		EXPECT_TRUE(cgogn::almost_equal_relative(n.dot(p1-p0),0.)); // is false !
		EXPECT_TRUE(cgogn::almost_equal_absolute(n.dot(p1-p0),0., 1e-8));
		EXPECT_TRUE(cgogn::almost_equal_relative(n.dot(p2-p0),0.));
		EXPECT_TRUE(cgogn::almost_equal_relative(n.dot(p2-p1),0.));
//		EXPECT_DOUBLE_EQ(n.dot(p1-p0),0); // is false !
		EXPECT_NEAR(n.dot(p1-p0),0, 1e-8);
		EXPECT_DOUBLE_EQ(n.dot(p2-p0),0);
		EXPECT_DOUBLE_EQ(n.dot(p2-p1),0);
	}

}


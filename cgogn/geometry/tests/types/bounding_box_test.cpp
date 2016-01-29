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

#include <geometry/types/bounding_box.h>
#include <gtest/gtest.h>
#include <iostream>
#include <cmath>

using StdArray = cgogn::geometry::Vec_T<std::array<double,3>>;
using EigenVec3d = Eigen::Vector3d;
using BoundingBox_Array = cgogn::geometry::BoundingBox<StdArray>;
using BoundingBox_Eigen = cgogn::geometry::BoundingBox<EigenVec3d>;

TEST(BoundingBox_TEST, NameOfType)
{
	EXPECT_EQ(cgogn::name_of_type(BoundingBox_Array()), "cgogn::geometry::BoundingBox<cgogn::geometry::Vec_T<std::array<double,3>>>");
	EXPECT_EQ(cgogn::name_of_type(BoundingBox_Eigen()), "cgogn::geometry::BoundingBox<Eigen::Matrix<double,3,1,0,3,1>>");
}

TEST(BoundingBox_TEST, Basics)
{
	double epsilon=0.0000001;
	{
		BoundingBox_Array bb;
		bb.addPoint(StdArray({0.5,0.4,0.3}));
		bb.addPoint(StdArray({-1,-2,-3}));
		bb.addPoint(StdArray({1,2,3}));

		EXPECT_EQ(bb.min(), StdArray({-1,-2,-3}));
		EXPECT_EQ(bb.max(), StdArray({1,2,3}));
		EXPECT_EQ(bb.maxSize(), 6);
		EXPECT_EQ(bb.minSize(), 2);
		EXPECT_NEAR(bb.diagSize(),std::sqrt(2.0*2+4*4+6*6),epsilon);
	std::cout << bb.center()[0] <<","<<bb.center()[1]<<","<<bb.center()[2]<< std::endl;
		EXPECT_EQ(bb.center(), StdArray({0,0,0}));

	}
	{
		BoundingBox_Eigen bb;
		bb.addPoint(EigenVec3d(0.5,0.4,0.3));
		bb.addPoint(EigenVec3d(-1,2,-3));
		bb.addPoint(EigenVec3d(1,-2,3));

		EXPECT_EQ(bb.min(), EigenVec3d(-1,-2,-3));
		EXPECT_EQ(bb.max(), EigenVec3d(1,2,3));
		EXPECT_EQ(bb.maxSize(), 6);
		EXPECT_EQ(bb.minSize(), 2);
		EXPECT_NEAR(bb.diagSize(),std::sqrt(2.0*2+4*4+6*6),epsilon);
		EXPECT_EQ(bb.center(), EigenVec3d(0,0,0));

	}
}

TEST(BoundingBox_TEST, testing)
{
	double epsilon=0.0000001;
	{
		BoundingBox_Array bb;
		bb.addPoint(StdArray({0.5,0.4,0.3}));
		bb.addPoint(StdArray({-1,-2,-3}));
		bb.addPoint(StdArray({1,2,3}));

		EXPECT_EQ(bb.contains(StdArray({1,1,1})),true);

		BoundingBox_Array bb2;
		bb2.addPoint(StdArray({0,0,0}));
		bb2.addPoint(StdArray({4,5,2}));

		EXPECT_EQ(bb.intersects(bb2),true);

		BoundingBox_Array bb3;
		bb3.addPoint(StdArray({0,0,0}));
		bb3.addPoint(StdArray({1,1,1}));

		EXPECT_EQ(bb.contains(bb3),true);

		EXPECT_EQ(bb.rayIntersect(StdArray({-9,-9,-9}),StdArray({1,1,1})),true);
		EXPECT_EQ(bb.rayIntersect(StdArray({-9,-9,-9}),StdArray({1,-1,0})),false);

	}
	{
		BoundingBox_Eigen bb;
		bb.addPoint(EigenVec3d(0.5,0.4,0.3));
		bb.addPoint(EigenVec3d(-1,2,-3));
		bb.addPoint(EigenVec3d(1,-2,3));

		EXPECT_EQ(bb.contains(EigenVec3d(1,1,1)),true);

		BoundingBox_Eigen bb2;
		bb2.addPoint(EigenVec3d(0,0,0));
		bb2.addPoint(EigenVec3d(4,5,2));

		EXPECT_EQ(bb.intersects(bb2),true);

		BoundingBox_Eigen bb3;
		bb3.addPoint(EigenVec3d(0,0,0));
		bb3.addPoint(EigenVec3d(1,1,1));

		EXPECT_EQ(bb.contains(bb3),true);

		EXPECT_EQ(bb.rayIntersect(EigenVec3d(-9,-9,-9),EigenVec3d(1,1,1)),true);
		EXPECT_EQ(bb.rayIntersect(EigenVec3d(-9,-9,-9),EigenVec3d(1,-1,0)),false);

	}
}


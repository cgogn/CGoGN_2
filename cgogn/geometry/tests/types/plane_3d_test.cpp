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

#include <geometry/types/plane_3d.h>
#include <gtest/gtest.h>
#include <iostream>

using StdArray = cgogn::geometry::Vec_T<std::array<double,3>>;
using EigenVec3d = Eigen::Vector3d;
using Plane3D_Array = cgogn::geometry::Plane3D<StdArray>;
using Plane3D_Eigen = cgogn::geometry::Plane3D<EigenVec3d>;

TEST(Plane3D_TEST, NameOfType)
{
	EXPECT_EQ(cgogn::name_of_type(Plane3D_Array(StdArray(),0.)), "cgogn::geometry::Plane3D<cgogn::geometry::Vec_T<std::array<double,3>>>");
	EXPECT_EQ(cgogn::name_of_type(Plane3D_Eigen(EigenVec3d(),0.)), "cgogn::geometry::Plane3D<Eigen::Matrix<double,3,1,0,3,1>>");
}

TEST(Plane3D_TEST, Project)
{
	{
		Plane3D_Array plane(StdArray{4.,0.,0.}, StdArray{0.,0.,0.});
		StdArray p{5.,8.,12.};
		plane.project(p);
		EXPECT_EQ(p[0], 0.);
		EXPECT_EQ(p[1], 8.);
		EXPECT_EQ(p[2], 12.);
	}
	{
		Plane3D_Eigen plane(EigenVec3d{4,0,0}, EigenVec3d{0,0,0});
		EigenVec3d p{5,8,12};
		plane.project(p);
		EXPECT_EQ(p[0], 0.);
		EXPECT_EQ(p[1], 8.);
		EXPECT_EQ(p[2], 12.);
	}
}

TEST(Plane3D_TEST, Orient)
{
	{
		Plane3D_Array plane(StdArray{0.,0.,15.}, StdArray{0.,0.,0.});
		StdArray p1{546854.,864.,12.};
		StdArray p2{-5.,886486.,-12.};
		StdArray p3{44552.,7.,0.};
		EXPECT_EQ(plane.orient(p1), cgogn::geometry::Orientation3D::OVER);
		EXPECT_EQ(plane.orient(p2), cgogn::geometry::Orientation3D::UNDER);
		EXPECT_EQ(plane.orient(p3), cgogn::geometry::Orientation3D::ON);
	}
	{
		Plane3D_Eigen plane(EigenVec3d{0,0,15}, EigenVec3d{0,0,0});
		EigenVec3d p1{546854,864,12};
		EigenVec3d p2{-5,886486,-12};
		EigenVec3d p3{44552,7,0};
		EXPECT_EQ(plane.orient(p1), cgogn::geometry::Orientation3D::OVER);
		EXPECT_EQ(plane.orient(p2), cgogn::geometry::Orientation3D::UNDER);
		EXPECT_EQ(plane.orient(p3), cgogn::geometry::Orientation3D::ON);
	}
}

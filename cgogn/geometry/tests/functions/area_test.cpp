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
#include <geometry/types/eigen.h>
#include <geometry/types/vec.h>
#include <geometry/functions/area.h>
#include <gtest/gtest.h>
#include <iostream>


using StdArray = cgogn::geometry::Vec_T<std::array<double,3>>;
using EigenVec3d = Eigen::Vector3d;

TEST(Area_TEST, TriangleArea)
{
	{
		StdArray p0(0,0,0);
		StdArray p1(2,0,0);
		StdArray p2(0,2,0);
		EXPECT_DOUBLE_EQ(cgogn::geometry::triangle_area(p0,p1,p2),2.0);
	}
	{
		EigenVec3d p0(0,0,0);
		EigenVec3d p1(2,0,0);
		EigenVec3d p2(0,2,0);
		EXPECT_DOUBLE_EQ(cgogn::geometry::triangle_area(p0,p1,p2),2.0);
	}

}


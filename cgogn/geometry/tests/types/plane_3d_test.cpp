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

#include <cgogn/core/utils/numerics.h>
#include <cgogn/geometry/types/plane_3d.h>
#include <cgogn/geometry/types/geometry_traits.h>

#include <gtest/gtest.h>

using namespace cgogn::numerics;

using StdArrayf = cgogn::geometry::Vec_T<std::array<float32,3>>;
using StdArrayd = cgogn::geometry::Vec_T<std::array<float64,3>>;
using EigenVec3f = Eigen::Vector3f;
using EigenVec3d = Eigen::Vector3d;
using VecTypes = testing::Types<StdArrayf, EigenVec3f, StdArrayd ,EigenVec3d>;


template <typename Vec_T>
class Plane3D_TEST : public testing::Test {};

TYPED_TEST_CASE(Plane3D_TEST, VecTypes );

TEST(Plane3D_TEST, NameOfType)
{
	EXPECT_EQ(cgogn::name_of_type(cgogn::geometry::Plane3D<StdArrayf>(StdArrayf(),0)), "cgogn::geometry::Plane3D<cgogn::geometry::Vec_T<std::array<float32,3>>>");
	EXPECT_EQ(cgogn::name_of_type(cgogn::geometry::Plane3D<EigenVec3f>(EigenVec3f(),0)), "cgogn::geometry::Plane3D<Eigen::Matrix<float,3,1,0,3,1>>");
	EXPECT_EQ(cgogn::name_of_type(cgogn::geometry::Plane3D<StdArrayd>(StdArrayd(),0)), "cgogn::geometry::Plane3D<cgogn::geometry::Vec_T<std::array<float64,3>>>");
	EXPECT_EQ(cgogn::name_of_type(cgogn::geometry::Plane3D<EigenVec3d>(EigenVec3d(),0)), "cgogn::geometry::Plane3D<Eigen::Matrix<double,3,1,0,3,1>>");
}

TYPED_TEST(Plane3D_TEST, Project)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	cgogn::geometry::Plane3D<TypeParam> plane(TypeParam{Scalar(4), Scalar(0), Scalar(0)}, TypeParam{Scalar(0), Scalar(0), Scalar(0)});
	TypeParam p{Scalar(5), Scalar(8), Scalar(12)};
	plane.project(p);
	EXPECT_EQ(p[0], Scalar(0));
	EXPECT_EQ(p[1], Scalar(8));
	EXPECT_EQ(p[2], Scalar(12));
}

TYPED_TEST(Plane3D_TEST, Orient)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;

	cgogn::geometry::Plane3D<TypeParam> plane(TypeParam{Scalar(0), Scalar(0), Scalar(15)}, TypeParam{Scalar(0), Scalar(0), Scalar(0)});
	TypeParam p1{Scalar(546854), Scalar(864), Scalar(12)};
	TypeParam p2{Scalar(-5), Scalar(886486), Scalar(-12)};
	TypeParam p3{Scalar(44552), Scalar(7), Scalar(0)};
	EXPECT_EQ(plane.orient(p1), cgogn::geometry::Orientation3D::OVER);
	EXPECT_EQ(plane.orient(p2), cgogn::geometry::Orientation3D::UNDER);
	EXPECT_EQ(plane.orient(p3), cgogn::geometry::Orientation3D::ON);
}

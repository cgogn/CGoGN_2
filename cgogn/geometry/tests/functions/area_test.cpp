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

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/functions/area.h>

#include <gtest/gtest.h>

using StdArrayf = cgogn::geometry::Vec_T<std::array<float,3>>;
using StdArrayd = cgogn::geometry::Vec_T<std::array<double,3>>;
using EigenVec3f = Eigen::Vector3f;
using EigenVec3d = Eigen::Vector3d;
using VecTypes = testing::Types<StdArrayf, EigenVec3f, StdArrayd ,EigenVec3d>;

template <typename Vec_T>
class Area_TEST : public testing::Test
{
};

TYPED_TEST_CASE(Area_TEST, VecTypes );

TYPED_TEST(Area_TEST, TriangleArea)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	TypeParam p0(Scalar(0), Scalar(0), Scalar(0));
	TypeParam p1(Scalar(2), Scalar(0), Scalar(0));
	TypeParam p2(Scalar(0), Scalar(2), Scalar(0));
	EXPECT_DOUBLE_EQ(cgogn::geometry::area(p0, p1, p2), Scalar(2));
}

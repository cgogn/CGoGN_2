#include <geometry/plane_3d.h>
#include <gtest/gtest.h>
#include <iostream>

using Vec1 = std::array<double,3>;
using Vec2 = Eigen::Vector3d;
using Plane3D1 = cgogn::geometry::Plane3D<Vec1>;
using Plane3D2 = cgogn::geometry::Plane3D<Vec2>;

TEST(Plane3D_TEST, NameOfType)
{
	EXPECT_EQ(cgogn::name_of_type(Plane3D1(Vec1(),0.)), "geometry::Plane3D<std::array<double,3>>");
	EXPECT_EQ(cgogn::name_of_type(Plane3D2(Vec2(),0.)), "geometry::Plane3D<Eigen::Vector3d>");
}

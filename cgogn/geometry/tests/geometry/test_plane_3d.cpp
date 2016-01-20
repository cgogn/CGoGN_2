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

TEST(Plane3D_TEST, Project)
{
	{
		Plane3D1 plane(Vec1{4,0,0}, Vec1{0,0,0});
		Vec1 p{5,8,12};
		plane.project(p);
		EXPECT_EQ(p[0], 0.);
		EXPECT_EQ(p[1], 8.);
		EXPECT_EQ(p[2], 12.);
	}
	{
		Plane3D2 plane(Vec2{4,0,0}, Vec2{0,0,0});
		Vec2 p{5,8,12};
		plane.project(p);
		EXPECT_EQ(p[0], 0.);
		EXPECT_EQ(p[1], 8.);
		EXPECT_EQ(p[2], 12.);
	}
}
TEST(Plane3D_TEST, Orient)
{
	{
		Plane3D1 plane(Vec1{0,0,15}, Vec1{0,0,0});
		Vec1 p1{546854,864,12};
		Vec1 p2{-5,886486,-12};
		Vec1 p3{44552,7,0};
		EXPECT_EQ(plane.orient(p1), cgogn::geometry::Orientation3D::OVER);
		EXPECT_EQ(plane.orient(p2), cgogn::geometry::Orientation3D::UNDER);
		EXPECT_EQ(plane.orient(p3), cgogn::geometry::Orientation3D::ON);
	}
	{
		Plane3D2 plane(Vec2{0,0,15}, Vec2{0,0,0});
		Vec2 p1{546854,864,12};
		Vec2 p2{-5,886486,-12};
		Vec2 p3{44552,7,0};
		EXPECT_EQ(plane.orient(p1), cgogn::geometry::Orientation3D::OVER);
		EXPECT_EQ(plane.orient(p2), cgogn::geometry::Orientation3D::UNDER);
		EXPECT_EQ(plane.orient(p3), cgogn::geometry::Orientation3D::ON);
	}
}

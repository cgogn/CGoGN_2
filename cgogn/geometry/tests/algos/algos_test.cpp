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

#include <cgogn/core/cmap/cmap2.h>

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>
#include <cgogn/geometry/algos/area.h>
#include <cgogn/geometry/algos/centroid.h>
#include <cgogn/geometry/algos/normal.h>
#include <cgogn/geometry/algos/ear_triangulation.h>

#include <cgogn/io/map_import.h>

#include <gtest/gtest.h>
#include <iostream>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using namespace cgogn::numerics;

using StdArrayf = cgogn::geometry::Vec_T<std::array<float,3>>;
using StdArrayd = cgogn::geometry::Vec_T<std::array<double,3>>;
using EigenVec3f = Eigen::Vector3f;
using EigenVec3d = Eigen::Vector3d;
using VecTypes = testing::Types<StdArrayf, EigenVec3f, StdArrayd ,EigenVec3d>;

using CMap2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
using Dart = cgogn::Dart;
template <typename T>
using VertexAttribute = CMap2::VertexAttribute<T>;
using Vertex = CMap2::Vertex;
using Edge = CMap2::Edge;
using Face = CMap2::Face;

template <typename Vec_T>
class Algos_TEST : public testing::Test
{
protected :
	CMap2 map2_;

	void add_polygone(uint32 n)
	{
		using Scalar = typename cgogn::geometry::vector_traits<Vec_T>::Scalar;

		VertexAttribute<Vec_T> vertex_position = this->map2_.template get_attribute<Vec_T, CMap2::Vertex::ORBIT>("position");
		Scalar alpha = 0;
		Face f = map2_.add_face(n);
		map2_.foreach_incident_vertex(f, [&] (Vertex v)
		{
			vertex_position[v] = Vec_T(std::cos(alpha),std::sin(alpha),Scalar(0));
			alpha += Scalar(2*M_PI/n);
		});
	}
};

TYPED_TEST_CASE(Algos_TEST, VecTypes);

TYPED_TEST(Algos_TEST, TriangleArea)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	VertexAttribute<TypeParam> vertex_position = this->map2_.template add_attribute<TypeParam, CMap2::Vertex::ORBIT>("position");
	this->add_polygone(3);
	Dart t;
	this->map2_.foreach_dart_until([&t] (Dart d) { t = d; return false; });
	const Scalar area = cgogn::geometry::area<TypeParam>(this->map2_, Face(t), vertex_position);
	const Scalar cf_area = cgogn::geometry::convex_area<TypeParam>(this->map2_, Face(t), vertex_position);
	EXPECT_DOUBLE_EQ(area, Scalar(0.75*std::sqrt(3.0)));
	EXPECT_DOUBLE_EQ(cf_area, Scalar(0.75*std::sqrt(3.0)));
}

TYPED_TEST(Algos_TEST, QuadArea)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	VertexAttribute<TypeParam> vertex_position = this->map2_.template add_attribute<TypeParam, CMap2::Vertex::ORBIT>("position");
	this->add_polygone(4);
	Dart q;
	this->map2_.foreach_dart_until([&q] (Dart d) { q = d; return false; });
	const Scalar area = cgogn::geometry::convex_area<TypeParam>(this->map2_, Face(q), vertex_position);
	EXPECT_DOUBLE_EQ(area, Scalar(2));
}

TYPED_TEST(Algos_TEST, TriangleCentroid)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	VertexAttribute<TypeParam> vertex_position = this->map2_.template add_attribute<TypeParam, CMap2::Vertex::ORBIT>("position");
	this->add_polygone(3);
	Dart t;
	this->map2_.foreach_dart_until([&t] (Dart d) { t = d; return false; });
	const TypeParam centroid = cgogn::geometry::centroid<TypeParam>(this->map2_, Face(t), vertex_position);
	std::cout << centroid << std::endl;
	EXPECT_TRUE(cgogn::almost_equal_absolute(centroid[0], Scalar(0)));
	EXPECT_TRUE(cgogn::almost_equal_absolute(centroid[1], Scalar(0)));
	EXPECT_TRUE(cgogn::almost_equal_absolute(centroid[2], Scalar(0)));
}

TYPED_TEST(Algos_TEST, QuadCentroid)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	VertexAttribute<TypeParam> vertex_position = this->map2_.template add_attribute<TypeParam, CMap2::Vertex::ORBIT>("position");
	this->add_polygone(4);
	Dart q;
	this->map2_.foreach_dart_until([&q] (Dart d) { q = d; return false; });
	const TypeParam centroid = cgogn::geometry::centroid<TypeParam>(this->map2_, Face(q), vertex_position);
	EXPECT_TRUE(cgogn::almost_equal_absolute(centroid[0], Scalar(0)));
	EXPECT_TRUE(cgogn::almost_equal_absolute(centroid[1], Scalar(0)));
	EXPECT_TRUE(cgogn::almost_equal_absolute(centroid[2], Scalar(0)));
}

TYPED_TEST(Algos_TEST, TriangleNormal)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	VertexAttribute<TypeParam> vertex_position = this->map2_.template add_attribute<TypeParam, CMap2::Vertex::ORBIT>("position");
	this->add_polygone(3);
	Dart t;
	this->map2_.foreach_dart_until([&t] (Dart d) { t = d; return false; });
	const TypeParam& n1 = cgogn::geometry::normal<TypeParam>(this->map2_, Face(t), vertex_position);
	const TypeParam& n2 = cgogn::geometry::normal<TypeParam>(this->map2_, Face(t), vertex_position);
	EXPECT_TRUE(cgogn::almost_equal_relative(n1[0], n2[0]));
	EXPECT_TRUE(cgogn::almost_equal_relative(n1[1], n2[1]));
	EXPECT_TRUE(cgogn::almost_equal_relative(n1[2], n2[2]));

	const TypeParam& cross = n1.cross(TypeParam(Scalar(0), Scalar(0), Scalar(1)));
	EXPECT_TRUE(cgogn::almost_equal_relative(cross[0], Scalar(0)));
	EXPECT_TRUE(cgogn::almost_equal_relative(cross[1], Scalar(0)));
	EXPECT_TRUE(cgogn::almost_equal_relative(cross[2], Scalar(0)));
}

TYPED_TEST(Algos_TEST, QuadNormal)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	VertexAttribute<TypeParam> vertex_position = this->map2_.template add_attribute<TypeParam, CMap2::Vertex::ORBIT>("position");
	this->add_polygone(4);
	Dart q;
	this->map2_.foreach_dart_until([&q] (Dart d) { q = d; return false; });
	const TypeParam& n1 = cgogn::geometry::normal<TypeParam>(this->map2_, Face(q), vertex_position);
	const TypeParam& cross = n1.cross(TypeParam(Scalar(0), Scalar(0), Scalar(1)));
	EXPECT_TRUE(cgogn::almost_equal_relative(cross[0], Scalar(0)));
	EXPECT_TRUE(cgogn::almost_equal_relative(cross[1], Scalar(0)));
	EXPECT_TRUE(cgogn::almost_equal_relative(cross[2], Scalar(0)));
}

TYPED_TEST(Algos_TEST, EarTriangulation)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	VertexAttribute<TypeParam> vertex_position = this->map2_.template add_attribute<TypeParam, Vertex::ORBIT>("position");

	Face f = this->map2_.add_face(5);
	cgogn::Dart d = f.dart;

	vertex_position[Vertex(d)] = TypeParam(Scalar(0), Scalar(0), Scalar(0));
	d = this->map2_.phi1(d);
	vertex_position[Vertex(d)] = TypeParam(Scalar(10), Scalar(0), Scalar(0));
	d = this->map2_.phi1(d);
	vertex_position[Vertex(d)] = TypeParam(Scalar(10), Scalar(10), Scalar(0));
	d = this->map2_.phi1(d);
	vertex_position[Vertex(d)] = TypeParam(Scalar(5), Scalar(5), Scalar(0));
	d = this->map2_.phi1(d);
	vertex_position[Vertex(d)] = TypeParam(Scalar(0), Scalar(10), Scalar(0));

	std::vector<uint32> indices;
	cgogn::geometry::append_ear_triangulation<TypeParam>(this->map2_, f, vertex_position, indices);
	EXPECT_TRUE(indices.size() == 9);

	Scalar area = 0;
	for (size_t i = 0; i < indices.size(); i += 3)
	{
		const TypeParam& A = vertex_position[indices[i]];
		const TypeParam& B = vertex_position[indices[i+1]];
		const TypeParam& C = vertex_position[indices[i+2]];
		area += cgogn::geometry::area(A, B, C);
	}
	EXPECT_DOUBLE_EQ(area, 75.0);

	cgogn::geometry::apply_ear_triangulation<TypeParam>(this->map2_, f, vertex_position);
	EXPECT_TRUE(this->map2_.template nb_cells<Face::ORBIT>() == 3);
//	EXPECT_TRUE(this->map2_.nb_boundary_cells() == 1);
	EXPECT_TRUE(this->map2_.template nb_cells<Edge::ORBIT>() == 7);
}

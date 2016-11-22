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

#include <iostream>

#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/io/map_import.h>
#include <cgogn/modeling/algos/catmull_clark.h>

#include <gtest/gtest.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using namespace cgogn::numerics;

using StdArrayf = cgogn::geometry::Vec_T<std::array<float,3>>;
using StdArrayd = cgogn::geometry::Vec_T<std::array<double,3>>;
using EigenVec3f = Eigen::Vector3f;
using EigenVec3d = Eigen::Vector3d;
using VecTypes = testing::Types<StdArrayf, EigenVec3f, StdArrayd ,EigenVec3d>;

using CMap2 = cgogn::CMap2;
using Dart = cgogn::Dart;
template <typename T>
using VertexAttribute = CMap2::VertexAttribute<T>;
using Vertex = CMap2::Vertex;
using Edge = CMap2::Edge;
using Face = CMap2::Face;

template <typename Vec_T>
class Algos_TEST : public testing::Test
{
protected:

	CMap2 map2_;

	void add_polygone(uint32 n)
	{
		using Scalar = typename cgogn::geometry::vector_traits<Vec_T>::Scalar;

		VertexAttribute<Vec_T> vertex_position = this->map2_.template get_attribute<Vec_T, CMap2::Vertex>("position");

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

TYPED_TEST(Algos_TEST, TriangleCatmullClark)
{
	VertexAttribute<TypeParam> vertex_position = this->map2_.template add_attribute<TypeParam, CMap2::Vertex>("position");

	this->add_polygone(3);

	cgogn::modeling::catmull_clark<TypeParam>(this->map2_, vertex_position);
	cgogn::modeling::catmull_clark<TypeParam>(this->map2_, vertex_position);

	int nb_f2 = 0;
	this->map2_.foreach_cell([&nb_f2] (Face)
	{
		nb_f2++;
	});

	EXPECT_EQ(12, nb_f2);
}

TYPED_TEST(Algos_TEST, QuadCatmullClark)
{
	VertexAttribute<TypeParam> vertex_position = this->map2_.template add_attribute<TypeParam, CMap2::Vertex>("position");

	this->add_polygone(4);

	cgogn::modeling::catmull_clark<TypeParam>(this->map2_, vertex_position);
	cgogn::modeling::catmull_clark<TypeParam>(this->map2_, vertex_position);

	int nb_f2 = 0;
	this->map2_.foreach_cell([&nb_f2] (Face)
	{
		nb_f2++;
	});

	EXPECT_EQ(16, nb_f2);
}

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

#include <core/cmap/cmap2.h>

#include <modelisation/algos/catmull_clark.h>

#include <io/map_import.h>

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
using VertexAttributeHandler = CMap2::VertexAttributeHandler<T>;
using Vertex = CMap2::Vertex;
using Edge = CMap2::Edge;
using Face = CMap2::Face;

template<typename Vec_T>
class Algos_TEST : public testing::Test
{
protected :
	CMap2 map2_;
};

TYPED_TEST_CASE(Algos_TEST, VecTypes);

TYPED_TEST(Algos_TEST, TriangleCatmullClark)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	cgogn::io::import_surface<TypeParam>(this->map2_, std::string(DEFAULT_MESH_PATH) + std::string("singleTriangle.obj"));
	VertexAttributeHandler<TypeParam> vertex_position = this->map2_.template get_attribute<TypeParam, CMap2::Vertex::ORBIT>("position");

	cgogn::modelisation::catmull_clark<TypeParam>(this->map2_,vertex_position);
	cgogn::modelisation::catmull_clark<TypeParam>(this->map2_,vertex_position);

	int nb_f2 = 0;
	this->map2_.foreach_cell([&nb_f2] (Face)
	{
		nb_f2++;
	});

	EXPECT_EQ(12,nb_f2);
}

TYPED_TEST(Algos_TEST, QuadCatmullClark)
{
	using Scalar = typename cgogn::geometry::vector_traits<TypeParam>::Scalar;
	cgogn::io::import_surface<TypeParam>(this->map2_, std::string(DEFAULT_MESH_PATH) + std::string("singleQuad.obj"));
	VertexAttributeHandler<TypeParam> vertex_position = this->map2_.template get_attribute<TypeParam, CMap2::Vertex::ORBIT>("position");

	cgogn::modelisation::catmull_clark<TypeParam>(this->map2_,vertex_position);
	cgogn::modelisation::catmull_clark<TypeParam>(this->map2_,vertex_position);

	int nb_f2 = 0;
	this->map2_.foreach_cell([&nb_f2] (Face)
	{
		nb_f2++;
	});

	EXPECT_EQ(16,nb_f2);

}


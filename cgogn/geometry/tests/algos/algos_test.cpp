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

#include <geometry/types/eigen.h>
#include <geometry/types/vec.h>
#include <geometry/algos/area.h>
#include <geometry/algos/centroid.h>
#include <geometry/algos/normal.h>
#include <geometry/algos/ear_triangulation.h>

#include <io/map_import.h>

#include <gtest/gtest.h>
#include <iostream>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

using StdArray = cgogn::geometry::Vec_T<std::array<double,3>>;
using EigenVec3d = Eigen::Vector3d;

using CMap2 = cgogn::CMap2<cgogn::DefaultMapTraits>;
template <typename T>
using VertexAttributeHandler = CMap2::VertexAttributeHandler<T>;

using Vertex = CMap2::Vertex;
using Edge = CMap2::Edge;
using Face = CMap2::Face;

TEST(Algos_TEST, TriangleArea)
{
	{
		CMap2 map;
		cgogn::io::import_surface<StdArray>(map, std::string(DEFAULT_MESH_PATH) + std::string("singleTriangle.obj"));
		VertexAttributeHandler<StdArray> vertex_position = map.get_attribute<StdArray, CMap2::Vertex::ORBIT>("position");
		const double area = cgogn::geometry::triangle_area<StdArray>(map, *map.begin(), vertex_position);
		const double cf_area = cgogn::geometry::convex_face_area<StdArray>(map, *map.begin(), vertex_position);
		EXPECT_DOUBLE_EQ(area, 12.5);
		EXPECT_DOUBLE_EQ(cf_area, 12.5);
	}
	{
//		CMap2 map;
//		cgogn::io::import_surface<EigenVec3d>(map, std::string(DEFAULT_MESH_PATH) + std::string("singleTriangle.obj"));
//		VertexAttributeHandler<EigenVec3d> vertex_position = map.get_attribute<EigenVec3d, CMap2::Vertex::ORBIT>("position");
//		const double area = cgogn::geometry::triangle_area<EigenVec3d>(map, *map.begin(), vertex_position);
//		const double cf_area = cgogn::geometry::convex_face_area<EigenVec3d>(map, *map.begin(), vertex_position);
//		EXPECT_DOUBLE_EQ(area, 12.5);
//		EXPECT_DOUBLE_EQ(cf_area, 12.5);
	}
}

TEST(Algos_TEST, QuadArea)
{
	{
		CMap2 map;
		cgogn::io::import_surface<StdArray>(map, std::string(DEFAULT_MESH_PATH) + std::string("singleQuad.obj"));
		VertexAttributeHandler<StdArray> vertex_position = map.get_attribute<StdArray, CMap2::Vertex::ORBIT>("position");
		const double area = cgogn::geometry::convex_face_area<StdArray>(map, *map.begin(), vertex_position);
		EXPECT_DOUBLE_EQ(area, 10);
	}
	{
//		CMap2 map;
//		cgogn::io::import_surface<EigenVec3d>(map, std::string(DEFAULT_MESH_PATH) + std::string("singleQuad.obj"));
//		VertexAttributeHandler<EigenVec3d> vertex_position = map.get_attribute<EigenVec3d, CMap2::Vertex::ORBIT>("position");
//		const double area = cgogn::geometry::convex_face_area<EigenVec3d>(map, *map.begin(), vertex_position);
//		EXPECT_DOUBLE_EQ(area, 10);

	}
}

TEST(Algos_TEST, TriangleCentroid)
{
	{
		CMap2 map;
		cgogn::io::import_surface<StdArray>(map, std::string(DEFAULT_MESH_PATH) + std::string("singleTriangle.obj"));
		VertexAttributeHandler<StdArray> vertex_position = map.get_attribute<StdArray, CMap2::Vertex::ORBIT>("position");
		const StdArray centroid = cgogn::geometry::centroid<StdArray>(map, CMap2::Face(*map.begin()), vertex_position);
		EXPECT_DOUBLE_EQ(centroid[0], 5.0/3.0);
		EXPECT_DOUBLE_EQ(centroid[1], 5.0/3.0);
		EXPECT_DOUBLE_EQ(centroid[2], 0);
	}

	{
//		CMap2 map;
//		cgogn::io::import_surface<StdArray>(map, std::string(DEFAULT_MESH_PATH) + std::string("singleTriangle.obj"));
//		VertexAttributeHandler<EigenVec3d> vertex_position = map.get_attribute<EigenVec3d, CMap2::Vertex::ORBIT>("position");
//		const EigenVec3d centroid = cgogn::geometry::centroid<EigenVec3d>(map, CMap2::Face(*map.begin()), vertex_position);
//		EXPECT_DOUBLE_EQ(centroid[0], 5.0/3.0);
//		EXPECT_DOUBLE_EQ(centroid[1], 5.0/3.0);
//		EXPECT_DOUBLE_EQ(centroid[2], 0);
	}
}

TEST(Algos_TEST, QuadCentroid)
{
	{
		CMap2 map;
		cgogn::io::import_surface<StdArray>(map, std::string(DEFAULT_MESH_PATH) + std::string("singleQuad.obj"));
		VertexAttributeHandler<StdArray> vertex_position = map.get_attribute<StdArray, CMap2::Vertex::ORBIT>("position");
		const StdArray centroid = cgogn::geometry::centroid<StdArray>(map, CMap2::Face(*map.begin()), vertex_position);
		EXPECT_DOUBLE_EQ(centroid[0], 2.5);
		EXPECT_DOUBLE_EQ(centroid[1], 1);
		EXPECT_DOUBLE_EQ(centroid[2], 0);
	}

	{
//		CMap2 map;
//		cgogn::io::import_surface<StdArray>(map, std::string(DEFAULT_MESH_PATH) + std::string("singleQuad.obj"));
//		VertexAttributeHandler<EigenVec3d> vertex_position = map.get_attribute<EigenVec3d, CMap2::Vertex::ORBIT>("position");
//		const EigenVec3d centroid = cgogn::geometry::centroid<EigenVec3d>(map, CMap2::Face(*map.begin()), vertex_position);
//		EXPECT_DOUBLE_EQ(centroid[0], 2.5);
//		EXPECT_DOUBLE_EQ(centroid[1], 1);
//		EXPECT_DOUBLE_EQ(centroid[2], 0);
	}
}

TEST(Algos_TEST, TriangleNormal)
{
	{
		CMap2 map;
		cgogn::io::import_surface<StdArray>(map, std::string(DEFAULT_MESH_PATH) + std::string("singleTriangle.obj"));
		VertexAttributeHandler<StdArray> vertex_position = map.get_attribute<StdArray, CMap2::Vertex::ORBIT>("position");
		const StdArray& n1 = cgogn::geometry::triangle_normal<StdArray>(map, CMap2::Face(*map.begin()), vertex_position);
		const StdArray& n2 = cgogn::geometry::face_normal<StdArray>(map, CMap2::Face(*map.begin()), vertex_position);
		EXPECT_TRUE(cgogn::almost_equal_relative(n1[0], n2[0]));
		EXPECT_TRUE(cgogn::almost_equal_relative(n1[1], n2[1]));
		EXPECT_TRUE(cgogn::almost_equal_relative(n1[2], n2[2]));

		const StdArray& cross = n1.cross(StdArray(0.,0.,1.));
		EXPECT_TRUE(cgogn::almost_equal_relative(cross[0], 0.));
		EXPECT_TRUE(cgogn::almost_equal_relative(cross[1], 0.));
		EXPECT_TRUE(cgogn::almost_equal_relative(cross[2], 0.));
	}
	{
//		CMap2 map;
//		cgogn::io::import_surface<EigenVec3d>(map, std::string(DEFAULT_MESH_PATH) + std::string("singleTriangle.obj"));
//		VertexAttributeHandler<EigenVec3d> vertex_position = map.get_attribute<EigenVec3d, CMap2::Vertex::ORBIT>("position");
//		const EigenVec3d& n1 = cgogn::geometry::triangle_normal<EigenVec3d>(map, CMap2::Face(*map.begin()), vertex_position);
//		const EigenVec3d& n2 = cgogn::geometry::face_normal<EigenVec3d>(map, CMap2::Face(*map.begin()), vertex_position);
//		EXPECT_TRUE(cgogn::almost_equal_relative(n1[0], n2[0]));
//		EXPECT_TRUE(cgogn::almost_equal_relative(n1[1], n2[1]));
//		EXPECT_TRUE(cgogn::almost_equal_relative(n1[2], n2[2]));

//		const EigenVec3d& cross = n1.cross(EigenVec3d(0,0,1));
//		EXPECT_TRUE(cgogn::almost_equal_relative(cross[0], 0.));
//		EXPECT_TRUE(cgogn::almost_equal_relative(cross[1], 0.));
//		EXPECT_TRUE(cgogn::almost_equal_relative(cross[2], 0.));
	}
}

TEST(Algos_TEST, QuadNormal)
{
	{
		CMap2 map;
		cgogn::io::import_surface<StdArray>(map, std::string(DEFAULT_MESH_PATH) + std::string("singleQuad.obj"));
		VertexAttributeHandler<StdArray> vertex_position = map.get_attribute<StdArray, CMap2::Vertex::ORBIT>("position");
		const StdArray& n1 = cgogn::geometry::face_normal<StdArray>(map, CMap2::Face(*map.begin()), vertex_position);
		const StdArray& cross = n1.cross(StdArray(0.,0.,1.));
		EXPECT_TRUE(cgogn::almost_equal_relative(cross[0], 0.));
		EXPECT_TRUE(cgogn::almost_equal_relative(cross[1], 0.));
		EXPECT_TRUE(cgogn::almost_equal_relative(cross[2], 0.));
	}
	{
//		CMap2 map;
//		cgogn::io::import_surface<EigenVec3d>(map, std::string(DEFAULT_MESH_PATH) + std::string("singleQuad.obj"));
//		VertexAttributeHandler<EigenVec3d> vertex_position = map.get_attribute<EigenVec3d, CMap2::Vertex::ORBIT>("position");
//		const EigenVec3d& n1 = cgogn::geometry::triangle_normal<EigenVec3d>(map, CMap2::Face(*map.begin()), vertex_position);
//		const EigenVec3d& cross = n1.cross(EigenVec3d(0,0,1));
//		EXPECT_TRUE(cgogn::almost_equal_relative(cross[0], 0.));
//		EXPECT_TRUE(cgogn::almost_equal_relative(cross[1], 0.));
//		EXPECT_TRUE(cgogn::almost_equal_relative(cross[2], 0.));
	}
}


TEST(Algos_TEST, EarTriangulation)
{
	// with array
	{
		CMap2 map;
		VertexAttributeHandler<StdArray> vertex_position = map.add_attribute<StdArray, Vertex::ORBIT>("position");

		Face f = map.add_face(5);
		cgogn::Dart d = f.dart;

		vertex_position[Vertex(d)] = StdArray(0.0,0.0,0.0);
		d = map.phi1(d);
		vertex_position[Vertex(d)] = StdArray(10.0,0.0,0.0);
		d = map.phi1(d);
		vertex_position[Vertex(d)] = StdArray(10.0,10.0,0.0);
		d = map.phi1(d);
		vertex_position[Vertex(d)] = StdArray(5.0,5.0,0.0);
		d = map.phi1(d);
		vertex_position[Vertex(d)] = StdArray(0.0,10.0,0.0);

		std::vector<unsigned int> indices;
		cgogn::geometry::compute_ear_triangulation<StdArray>(map,f,vertex_position,indices);
		EXPECT_TRUE(indices.size() == 9);

		double area = 0.0;
		for (size_t i=0; i<indices.size(); i+=3)
		{
			const StdArray& A = vertex_position[indices[i]];
			const StdArray& B = vertex_position[indices[i+1]];
			const StdArray& C = vertex_position[indices[i+2]];
			area += cgogn::geometry::triangle_area(A,B,C);
		}
		EXPECT_DOUBLE_EQ(area,75.0);

		cgogn::geometry::apply_ear_triangulation<StdArray>(map,f,vertex_position);
		EXPECT_TRUE(map.nb_cells<Face::ORBIT>()==4);
		EXPECT_TRUE(map.nb_cells<Edge::ORBIT>()==7);
	}
	// with Eigen
	{
		CMap2 map;
		VertexAttributeHandler<EigenVec3d> vertex_position = map.add_attribute<EigenVec3d, Vertex::ORBIT>("position");

		Face f = map.add_face(5);
		cgogn::Dart d = f.dart;

		vertex_position[CMap2::Vertex(d)] = EigenVec3d(0,0,0);
		d = map.phi1(d);
		vertex_position[CMap2::Vertex(d)] = EigenVec3d(10,0,0);
		d = map.phi1(d);
		vertex_position[CMap2::Vertex(d)] = EigenVec3d(10,10,0);
		d = map.phi1(d);
		vertex_position[CMap2::Vertex(d)] = EigenVec3d(5,5,0);
		d = map.phi1(d);
		vertex_position[CMap2::Vertex(d)] = EigenVec3d(0,10,0);

		std::vector<unsigned int> indices;
		cgogn::geometry::compute_ear_triangulation<EigenVec3d>(map,f,vertex_position,indices);
		EXPECT_TRUE(indices.size() == 9);

		double area = 0.0;
		for (size_t i=0; i<indices.size(); i+=3)
		{
			const EigenVec3d& A = vertex_position[indices[i]];
			const EigenVec3d& B = vertex_position[indices[i+1]];
			const EigenVec3d& C = vertex_position[indices[i+2]];
			area += cgogn::geometry::triangle_area(A,B,C);
		}
		EXPECT_DOUBLE_EQ(area,75.0);

		cgogn::geometry::apply_ear_triangulation<EigenVec3d>(map,f,vertex_position);
		EXPECT_TRUE(map.nb_cells<Face::ORBIT>()==4);
		EXPECT_TRUE(map.nb_cells<Edge::ORBIT>()==7);
	}
}

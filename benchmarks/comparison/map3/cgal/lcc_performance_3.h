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

#ifndef BENCHMARK_COMPARISON_LCC_PERFORMANCE3_H
#define BENCHMARK_COMPARISON_LCC_PERFORMANCE3_H

#include <performance.h>

#include <CGAL/Linear_cell_complex.h>
#include <CGAL/Linear_cell_complex_operations.h>
#include <CGAL/Linear_cell_complex_constructors.h>
#include <CGAL/Cell_attribute.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <vector>

// Comment to use cmap with handle
//#define CMAP_WITH_INDEX 1

typedef CGAL::Simple_cartesian<Performance3::Real>  Kernel;
typedef Kernel::Point_3			Point_3;
typedef CGAL::Vector_3<Kernel>		Vector_3;

struct Info_for_vertex
{
  Info_for_vertex() : v(CGAL::NULL_VECTOR), m(0)
  {}

  Vector_3 v;
  int m;
};

struct Info_for_volume
{
  Info_for_volume() : m(0)
  {}

  int m;
};

#ifndef CMAP_WITH_INDEX

struct MyItem
{
  template<class Refs>
  struct Dart_wrapper
  {
    typedef CGAL::Dart<3, Refs> Dart;

    typedef CGAL::Cell_attribute_with_point<Refs, Info_for_vertex, CGAL::Tag_true> Vertex_attribute;

    typedef CGAL::Cell_attribute<Refs, Info_for_volume, CGAL::Tag_true> Vol_attribute;

    typedef CGAL::cpp0x::tuple<Vertex_attribute, void, void, Vol_attribute> Attributes;
  };
};

typedef CGAL::Linear_cell_complex<3,3,CGAL::Linear_cell_complex_traits<3,Kernel>, MyItem>	LCC_3;

#else

struct MyItem
{
  template<class Refs>
  struct Dart_wrapper
  {
    typedef CGAL::Dart_for_index<3, Refs> Dart;

    typedef CGAL::Cell_attribute_for_index_with_point<Refs, Info_for_vertex, CGAL::Tag_true> Vertex_attribute;

    typedef CGAL::Cell_attribute_for_index<Refs, Info_for_volume, CGAL::Tag_true> Vol_attribute;

    typedef CGAL::cpp0x::tuple<Vertex_attribute, void, void, Vol_attribute> Attributes;
  };
};

typedef CGAL::Linear_cell_complex_for_index<3,3,CGAL::Linear_cell_complex_traits<3,Kernel>, MyItem>	LCC_3;

#endif

typedef LCC_3::Dart_handle		 Dart_handle;
typedef LCC_3::Vertex_attribute_handle   Vertex_handle;
typedef LCC_3::Attribute_handle<3>::type Volume_handle;
typedef LCC_3::Point			 Point;
typedef LCC_3::Vector			 Vector;
typedef LCC_3::FT			 FT;


class Performance3_LCC : public Performance3
{
protected:
	void clear_mesh() override;
	bool read_mesh(const std::string& filename) override;
	void update_accelerators();
	Dart_handle create_face(Dart_handle old1,
							Vertex_handle v1, Vertex_handle v2, Vertex_handle v3,
							Volume_handle vol);
	void collapse(Dart_handle dart);
	Dart_handle tet_split(Dart_handle d1);
	bool is_boundary(const Dart_handle& _dart);
	Dart_handle getShortestEdge();
	Vector_3 barycenter_3(Dart_handle d);

protected:
	std::unique_ptr<LCC_3> lcc;
	CGAL::Unique_hash_map<Vertex_handle, std::vector<Vertex_handle>, typename LCC_3::Hash_function> adjacent_vertices;
};

#endif // BENCHMARK_COMPARISON_LCC_PERFORMANCE3_H

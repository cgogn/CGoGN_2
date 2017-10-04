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

#ifndef BENCHMARK_COMPARISON_LCC_H
#define BENCHMARK_COMPARISON_LCC_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Linear_cell_complex.h>
#include <CGAL/Linear_cell_complex_constructors.h>
#include <iostream>
#include <fstream>
#include "performance.h"

typedef CGAL::Simple_cartesian<Performance2::Real>  MyKernelLCC;
typedef MyKernelLCC::Point_3            LCCPoint_3;
typedef MyKernelLCC::Vector_3           LCCVector_3;

typedef CGAL::Linear_cell_complex_traits<3, MyKernelLCC> MyTraitsLCC;

#ifndef CMAP_WITH_INDEX

class MyItemsLCC
{
public:

  template <class LCC>
  struct Dart_wrapper
  {
	typedef CGAL::Dart<2, LCC> Dart; // Dim 2 == Polyhedron

	typedef CGAL::Cell_attribute_with_point<LCC, LCCVector_3, CGAL::Tag_true> Vertex;
	typedef CGAL::Cell_attribute<LCC, LCCVector_3, CGAL::Tag_true> Face;
	typedef CGAL::cpp0x::tuple<Vertex,void,Face> Attributes;
  };
};

typedef CGAL::Linear_cell_complex<2, 3, MyTraitsLCC, MyItemsLCC> LCC;

#else

class MyItemsLCCWithIndex
{
public:

  template <class LCC>
  struct Dart_wrapper
  {
	typedef CGAL::Dart_for_index<2, LCC> Dart; // Dim 2 == Polyhedron

	typedef CGAL::Cell_attribute_for_index_with_point<LCC, LCCVector_3, CGAL::Tag_true> Vertex;
	typedef CGAL::Cell_attribute_for_index<LCC, LCCVector_3, CGAL::Tag_true> Face;
	typedef CGAL::cpp0x::tuple<Vertex,void,Face> Attributes;
  };
};
typedef CGAL::Linear_cell_complex_for_index<2, 3, MyTraitsLCC, MyItemsLCCWithIndex> LCC;

#endif


class Performance2_LCC : public Performance2
{
protected:
	void clear_mesh() override;
	bool read_mesh(const std::string& filename) override;
	void flip_edge(LCC::Dart_handle d);
	void contract_face(LCC::Dart_handle dh);
	void collapse_edge(LCC::Dart_handle dh);

protected:
	std::unique_ptr<LCC> lcc;
};

#endif // BENCHMARK_COMPARISON_LCC_H

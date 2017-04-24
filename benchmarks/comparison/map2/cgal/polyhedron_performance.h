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

#ifndef BENCHMARK_COMPARISON_POLYHEDRON_H
#define BENCHMARK_COMPARISON_POLYHEDRON_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/HalfedgeDS_vector.h>
#include <CGAL/HalfedgeDS_list.h>
#include <CGAL/HalfedgeDS_vertex_base.h>
#include <CGAL/HalfedgeDS_halfedge_base.h>
#include <CGAL/HalfedgeDS_face_base.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <iostream>
#include <fstream>
#include "performance.h"

//*****************************************************************************
// control whether Polyhedron stores vertex and face normals
#define HAS_NORMALS 1
//*****************************************************************************

class Performance2_Polyhedron : public Performance2
{
public:
	using CGALKernel =  CGAL::Simple_cartesian<Real>;
	using Point_3 = CGALKernel::Point_3;
	using Vector_3 = CGAL::Vector_3<CGALKernel>;

protected:
	template <class Refs>
	struct MyVertex : public CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, Point_3>
	{
		typedef CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, Point_3> Base;
		MyVertex() : Base() {}
		MyVertex(const Point_3& p) : Base(p) {}
		Vector_3 normal;
	};


	template <class Refs>
	struct MyFace : public CGAL::HalfedgeDS_face_base<Refs>
	{
		Vector_3 normal;
	};


	class MyItems
	{
	public:

		template <class Refs, class Traits>
		struct Vertex_wrapper
		{
			typedef typename Traits::Point_3 Point;
#if HAS_NORMALS
			typedef MyVertex<Refs> Vertex;
#else
			typedef CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, Point> Vertex;
#endif
		};

		template <class Refs, class Traits>
		struct Halfedge_wrapper
		{
			typedef CGAL::HalfedgeDS_halfedge_base<Refs> Halfedge;
		};

		template <class Refs, class Traits>
		struct Face_wrapper
		{
#if HAS_NORMALS
			typedef MyFace<Refs> Face;
#else
			typedef CGAL::HalfedgeDS_face_base<Refs> Face;
#endif
		};
	};

	using Polyhedron = CGAL::Polyhedron_3<CGALKernel, MyItems, CGAL::HalfedgeDS_list>;

	void clear_mesh() override;
	bool read_mesh(const std::string& filename) override;
	void halfedge_collapse(Polyhedron::Halfedge_handle pq);

protected:
	std::unique_ptr<Polyhedron> P;
};

#endif // BENCHMARK_COMPARISON_POLYHEDRON_H

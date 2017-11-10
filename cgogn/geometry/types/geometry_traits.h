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

#ifndef CGOGN_GEOMETRY_TYPES_GEOMETRY_TRAITS_H_
#define CGOGN_GEOMETRY_TYPES_GEOMETRY_TRAITS_H_

#include <type_traits>
#include <cgogn/core/utils/numerics.h>

#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>

namespace cgogn
{

namespace geometry
{

template <typename Vec_T, typename Enable = void>
struct vector_traits
{};

// specialization : cgogn::geometry::Vec_T with a fixed-size array
template <typename Scalar_, std::size_t Size>
struct vector_traits<geometry::Vec_T<std::array<Scalar_, Size>>>
{
	static const std::size_t SIZE = Size;
	using Scalar = Scalar_;
};

// specialization : Eigen::Vector
template <typename Scalar_, int32 Rows, int32 Options>
struct vector_traits<Eigen::Matrix<Scalar_, Rows, 1, Options, Rows, 1>>
{
	static const std::size_t SIZE = Rows;
	using Scalar = Scalar_;
};

// specialization : Eigen::AlignedVector3
template <typename Scalar_>
struct vector_traits<Eigen::AlignedVector3<Scalar_>>
{
	static const std::size_t SIZE = 3;
	using Scalar = Scalar_;
};

// next specializations are for uniform manip of vec & scalar (vbo)
template <typename T>
struct vector_traits<T, typename std::enable_if<std::is_integral<T>::value || std::is_floating_point<T>::value>::type>
{
	static const std::size_t SIZE = 1;
	using Scalar = T;
};


template <typename T>
auto set_zero(T& t) -> typename std::enable_if<(vector_traits<T>::SIZE > 1)>::type
{
	t.setZero();
}

template <typename T>
auto set_zero(T& t) -> typename std::enable_if<(vector_traits<T>::SIZE == 1)>::type
{
	t = 0;
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_GEOMETRY_TRAITS_H_

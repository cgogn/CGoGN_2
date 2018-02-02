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
	using Type = geometry::Vec_T<std::array<Scalar_, Size>>;
};

// specialization for uniform manip of vec & scalar (vbo)

template <typename T>
struct vector_traits<T, typename std::enable_if<std::is_integral<T>::value || std::is_floating_point<T>::value>::type>
{
	static const std::size_t SIZE = 1;
	using Scalar = T;
};


// specialization : Eigen::Vector

// an utility function that return true/false_type if param is Eigen
template <typename T>
std::true_type cgogn_check_eigen_type(const Eigen::MatrixBase<T>*);
std::false_type cgogn_check_eigen_type(...);

// the class bool is_xxxx that inherit the return type of preceeding functions
template <typename T>
struct is_eigen : public decltype(cgogn_check_eigen_type(std::declval<T*>()))
{};

template <typename V>
struct vector_traits<V, typename std::enable_if < is_eigen<V>::value >::type>
{
	static const std::size_t SIZE = Eigen::internal::traits<V>::RowsAtCompileTime;;
	using Scalar = typename Eigen::internal::traits<V>::Scalar;
	using Type = Eigen::Matrix<Scalar, SIZE, 1, 0, SIZE, 1 >;
};

// convenient struct for easy SFINAE

template <typename V1, typename V2, typename Enable = void>
struct is_same2vector 
{};

template <typename V1, typename V2>
struct is_same2vector < V1, V2, typename std::enable_if < std::is_same<V1, V2>::value  || ( is_eigen<V1>::value && is_eigen<V2>::value )>::type>
{
	using S1 = typename vector_traits<V1>::Scalar;
	using S2 = typename vector_traits<V2>::Scalar;
	static const bool value = std::is_same<S1, S2>::value && (vector_traits<V1>::SIZE == vector_traits<V2>::SIZE);
};

template <typename V1, typename V2>
struct is_same2vector < V1, V2, typename std::enable_if <!(std::is_same<V1, V2>::value || (is_eigen<V1>::value && is_eigen<V2>::value))>::type>
{
	static const bool value = false;
};

template <typename V1, typename V2, typename V3>
struct is_same3vector : public std::integral_constant < bool, is_same2vector<V1, V2>::value && is_same2vector<V1, V3>::value> {};

template <typename V1, typename V2, typename V3, typename V4>
struct is_same4vector : public std::integral_constant < bool, is_same3vector<V1, V2, V3>::value && is_same2vector<V1, V4>::value> {};

template <typename V1, typename V2, typename V3, typename V4, typename V5>
struct is_same5vector : public std::integral_constant < bool, is_same4vector<V1, V2, V3, V4>::value && is_same2vector<V1, V5>::value> {};




template <typename T, typename Enable = void>
struct nb_components_traits
{};


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

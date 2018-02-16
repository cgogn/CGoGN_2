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
{
	static const bool OK = false;
};

// specialization : cgogn::geometry::Vec_T with a fixed-size array
template <typename Scalar_, std::size_t Size>
struct vector_traits<geometry::Vec_T<std::array<Scalar_, Size>>>
{
	static const bool OK = true;
	static const std::size_t SIZE = Size;
	using Scalar = Scalar_;
	using Type = geometry::Vec_T<std::array<Scalar_, Size>>;
};

// specialization for uniform manip of vec & scalar (vbo)

template <typename T>
struct vector_traits<T, typename std::enable_if<std::is_integral<T>::value || std::is_floating_point<T>::value>::type>
{
	static const bool OK = true;
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
struct vector_traits<V, typename std::enable_if < is_eigen<V>::value>::type>
{
	static const bool OK = true;
	static const std::size_t SIZE = Eigen::internal::traits<V>::RowsAtCompileTime;
	using Scalar = typename Eigen::internal::traits<V>::Scalar;
	using Type = Eigen::Matrix<Scalar, SIZE, 1, 0, SIZE, 1 >;
};


template <typename V>
struct vector_traits<Eigen::MatrixBase<V>, typename std::enable_if < is_eigen<Eigen::MatrixBase<V>>::value>::type>
{
	static const bool OK = true;
	static const std::size_t SIZE = Eigen::internal::traits<V>::RowsAtCompileTime;
	using Scalar = typename Eigen::internal::traits<V>::Scalar;
	using Type = Eigen::Matrix<Scalar, SIZE, 1, 0, SIZE, 1 >;
};


template <typename V>
using VecType = typename vector_traits<V>::Type;

// convenient struct for easy SFINAE


//template <typename V>
//constexpr bool is_vec_non_eigen()
//{
//	return vector_traits<V>::OK && !is_eigen<V>::value;
//}

template <typename V>
struct is_vec_non_eigen
{
	static const bool value = vector_traits<V>::OK && !is_eigen<V>::value;
};


/**
 * is_vectors_size<N,V1,V2,..>::value = true if all vectors are of size N
 */
template <int N, typename... Vs>
struct is_vectors_size
{
	static const bool value = true;
};

template <int N, typename V, typename... Vs>
struct is_vectors_size<N,V,Vs...>
{
	static const bool value = (vector_traits<V>::SIZE == N) && is_vectors_size<N,Vs...>::value;
};


/**
 * is_vectors_scalar<T,V1,V2,..>::value = true if all vectors scalar type are T
 */
template <typename T, typename... Vs>
struct is_vectors_scalar
{
	static const bool value = true;
};

template <typename T, typename V, typename... Vs>
struct is_vectors_scalar<T,V,Vs...>
{
	static const bool value = std::is_same<typename vector_traits<V>::Scalar,T>::value && is_vectors_scalar<T,Vs...>::value;
};


/**
 * is_same_vector<V1,V2,..>::value = true if all vectors have same size and scalar type
 */

template <typename... Vs>
struct is_same_vector
{};

template <typename V1, typename V2>
struct is_same_vector<V1,V2>
{
	static const bool value = (vector_traits<V1>::SIZE == vector_traits<V2>::SIZE) &&
			std::is_same<typename vector_traits<V1>::Scalar,typename vector_traits<V2>::Scalar>::value;
};


template <typename V1, typename V2, typename... Vs>
struct is_same_vector<V1,V2,Vs...>
{
	static const bool value = (vector_traits<V1>::SIZE == vector_traits<V2>::SIZE) &&
			std::is_same<typename vector_traits<V1>::Scalar,typename vector_traits<V2>::Scalar>::value &&
			is_same_vector<V2,Vs...>::value;
};

//template<typename V, typename... Vs>
//constexpr bool is_same_vector()
//{
//	return internal::is_same_vect<V,Vs...>::value;
//}


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

template<typename V>
using ScalarOf = typename vector_traits<V>::Scalar;


template<typename V, int N>
struct is_dim_of
{
	static const bool value = vector_traits<V>::SIZE == N;
};


template <typename VEC>
using TypeEigen = Eigen::Matrix< ScalarOf<VEC>,vector_traits<VEC>::SIZE,1>;

template <typename VEC>
using ConstTypeEigenize = Eigen::Map<const TypeEigen<VEC>>;

template <typename VEC>
using TypeEigenize = Eigen::Map<TypeEigen<VEC>>;

template <typename VEC>
inline TypeEigenize<VEC> eigenize(VEC& v) { return TypeEigenize<VEC>(&(v[0])); }

template <typename VEC>
inline ConstTypeEigenize<VEC> eigenize(const VEC& v) { return ConstTypeEigenize<VEC>(&(v[0])); }


template <typename V, typename E>
inline auto copy_to_vec(const E& v)
-> typename std::enable_if< (vector_traits<E>::SIZE==2) && (vector_traits<V>::SIZE == 2), V>::type
{ return V(v[0],v[1]); }

template <typename V, typename E>
inline auto copy_to_vec(const E& v)
-> typename std::enable_if< (vector_traits<E>::SIZE==3) && (vector_traits<V>::SIZE == 3), V>::type
{ return V(v[0],v[1],v[2]); }

template <typename V, typename E>
inline auto copy_to_vec(const E& v)
-> typename std::enable_if< (vector_traits<E>::SIZE==4) && (vector_traits<V>::SIZE == 4), V>::type
{ return V(v[0],v[1],v[2],v[3]); }



} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_GEOMETRY_TRAITS_H_

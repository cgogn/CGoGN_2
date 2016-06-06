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

#ifndef CGOGN_CORE_UTILS_TYPE_TRAITS_H_
#define CGOGN_CORE_UTILS_TYPE_TRAITS_H_

#include <type_traits>
#include <cgogn/core/utils/numerics.h>

namespace cgogn
{
namespace type_traits
{

namespace internal
{
template <class>
struct sfinae_true : std::true_type {};

template <class T>
static auto test_operator_brackets(int32 ) -> sfinae_true<decltype(std::declval<T>()[0ul])>;
template <class>
static auto test_operator_brackets(int64) -> std::false_type;

template <class T>
static auto test_size_method(int32 ) -> sfinae_true<decltype(std::declval<T>().size())>;
template <class>
static auto test_size_method(int64) -> std::false_type;

template <class T>
static auto test_begin_method(int32 ) -> sfinae_true<decltype(std::declval<T>().begin())>;
template <class>
static auto test_begin_method(int64) -> std::false_type;

template <class T>
static auto test_iterable(int32 ) -> sfinae_true<decltype(std::declval<T>().end() != std::declval<T>().end())>;
template <class>
static auto test_iterable(int64) -> std::false_type;

template <class T>
static auto test_name_of_type(int32 ) -> sfinae_true<decltype(T::cgogn_name_of_type())>;
template <class>
static auto test_name_of_type(int64) -> std::false_type;

} // namespace internal

template <class T>
struct has_operator_brackets : decltype(internal::test_operator_brackets<T>(0)){};

template <class T>
struct has_size_method : decltype(internal::test_size_method<T>(0)){};

template <class T>
struct has_begin_method : decltype(internal::test_begin_method<T>(0)){};

template <class T>
struct is_iterable : decltype(internal::test_iterable<T>(0)){};

template <class T>
struct has_cgogn_name_of_type : decltype(internal::test_name_of_type<T>(0)){};


template <typename T, typename Enable = void>
struct nested_type;

template <typename T>
struct nested_type<T, typename std::enable_if<!has_operator_brackets<T>::value>::type>
{
	using type = typename std::remove_cv< typename std::remove_reference<T>::type>::type;
};

template <typename T>
struct nested_type<T, typename std::enable_if<has_operator_brackets<T>::value>::type>
{
	using type = typename nested_type<typename std::remove_cv< typename std::remove_reference<decltype(std::declval<T>()[0ul])>::type >::type>::type;
};


template <typename T>
inline typename std::enable_if<!has_size_method<T>::value, uint32>::type nb_components(const T& );
template <typename T>
inline typename std::enable_if<has_size_method<T>::value && has_begin_method<T>::value, uint32>::type nb_components(const T& val);
template <typename T>
inline typename std::enable_if<has_size_method<T>::value && !has_begin_method<T>::value, uint32>::type nb_components(const T& val);


template <typename T>
inline typename std::enable_if<!has_size_method<T>::value, uint32>::type nb_components(const T&)
{
	return 1u;
}

template <typename T>
inline typename std::enable_if<has_size_method<T>::value && has_begin_method<T>::value, uint32>::type nb_components(const T& val)
{
	return uint32(val.size()) * nb_components(*(val.begin()));
}

template <typename T>
inline typename std::enable_if<has_size_method<T>::value && !has_begin_method<T>::value, uint32>::type nb_components(const T& val)
{
	return uint32(val.size()) * nb_components(val[0]);
}

} // namespace type_traits

} // namespace cgogn

#endif // CGOGN_CORE_UTILS_TYPE_TRAITS_H_

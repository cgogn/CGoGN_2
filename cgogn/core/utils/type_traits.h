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

namespace internal
{

namespace type_traits
{

/**
 * Traits class to inspect function characteristics (return type, arity, parameters types)
 */
template <typename T>
struct function_traits : public function_traits<decltype(&T::operator())>
{};

template <typename ClassType, typename ReturnType, typename... Args>
struct function_traits<ReturnType(ClassType::*)(Args...) const>
// we specialize for pointers to member function
{
	static const size_t arity = sizeof...(Args);
	// arity is the number of arguments.

	using result_type = ReturnType;

	template <size_t i>
	struct arg
	{
		static_assert(i < sizeof...(Args), "Trying to access to an argument whose index is higher than the function arity.");
		using type = typename std::tuple_element<i, std::tuple<Args...>>::type;
		// the i-th argument is equivalent to the i-th tuple element of a tuple
		// composed of those arguments.
	};
};

template <class>
struct sfinae_true : std::true_type {};

template <class T>
static auto test_operator_parenthesis_0(int32) -> sfinae_true<decltype(std::declval<T>()())>;
template <class>
static auto test_operator_parenthesis_0(int64) -> std::false_type;

template <class T>
static auto test_operator_parenthesis_1(int32) -> sfinae_true<decltype(std::declval<T>()(0ul))>;
template <class>
static auto test_operator_parenthesis_1(int64) -> std::false_type;

template <class T>
static auto test_operator_parenthesis_2(int32) -> sfinae_true<decltype(std::declval<T>()(0ul,0ul))>;
template <class>
static auto test_operator_parenthesis_2(int64) -> std::false_type;

template <class T>
static auto test_operator_brackets(int32) -> sfinae_true<decltype(std::declval<T>()[0ul])>;
template <class>
static auto test_operator_brackets(int64) -> std::false_type;

template <class T>
static auto test_size_method(int32) -> sfinae_true<decltype(std::declval<T>().size())>;
template <class>
static auto test_size_method(int64) -> std::false_type;

template <class T>
static auto test_begin_method(int32) -> sfinae_true<decltype(std::declval<T>().begin())>;
template <class>
static auto test_begin_method(int64) -> std::false_type;

template <class T>
static auto test_iterable(int32) -> sfinae_true<decltype(std::declval<T>().end() != std::declval<T>().end())>;
template <class>
static auto test_iterable(int64) -> std::false_type;

template <class T>
static auto test_rows_method(int32) -> sfinae_true<decltype(std::declval<T>().rows())>;
template <class>
static auto test_rows_method(int64) -> std::false_type;

template <class T>
static auto test_cols_method(int32) -> sfinae_true<decltype(std::declval<T>().cols())>;
template <class>
static auto test_cols_method(int64) -> std::false_type;

template <class T>
static auto test_name_of_type(int32) -> sfinae_true<decltype(T::cgogn_name_of_type())>;
template <class>
static auto test_name_of_type(int64) -> std::false_type;

} // namespace type_traits
} // namespace internal

template <class T>
struct has_operator_parenthesis_0 : decltype(internal::type_traits::test_operator_parenthesis_0<T>(0)){};

template <class T>
struct has_operator_parenthesis_1 : decltype(internal::type_traits::test_operator_parenthesis_1<T>(0)){};

template <class T>
struct has_operator_parenthesis_2 : decltype(internal::type_traits::test_operator_parenthesis_2<T>(0)){};

template <class T>
struct has_operator_brackets : decltype(internal::type_traits::test_operator_brackets<T>(0)){};

template <class T>
struct has_size_method : decltype(internal::type_traits::test_size_method<T>(0)){};

template <class T>
struct has_begin_method : decltype(internal::type_traits::test_begin_method<T>(0)){};

template <class T>
struct has_rows_method : decltype(internal::type_traits::test_rows_method<T>(0)){};

template <class T>
struct has_cols_method : decltype(internal::type_traits::test_cols_method<T>(0)){};

template <class T>
struct is_iterable : decltype(internal::type_traits::test_iterable<T>(0)){};

template <class T>
struct has_cgogn_name_of_type : decltype(internal::type_traits::test_name_of_type<T>(0)){};


namespace internal
{

namespace type_traits
{

/**
 * Helper to find nested type
 */
template <typename T, typename Enable = void>
struct nested_type_helper;

template <typename T>
struct nested_type_helper<T, typename std::enable_if<!has_operator_brackets<T>::value>::type>
{
	using type = typename std::remove_cv< typename std::remove_reference<T>::type>::type;
};

template <typename T>
struct nested_type_helper<T, typename std::enable_if<has_operator_brackets<T>::value>::type>
{
	using type = typename nested_type_helper<typename std::remove_cv< typename std::remove_reference<decltype(std::declval<T>()[0ul])>::type >::type>::type;
};

/**
* This helper is needed because defining the template alias directly leads to a compilation error with MSVC 2013.
*/
template<typename T>
struct array_data_type_helper
{
	using type = typename std::remove_cv< typename std::remove_reference<decltype(std::declval<T>()[0ul])>::type >::type;
};

} // namespace type_traits
} // namespace internal

/**
 * Apply recursively the operator[] and return the deepest type
 */
template<typename T>
using nested_type = typename internal::type_traits::nested_type_helper<T>::type;

/**
 * type of the data stored inside an array
 */
template<typename T>
using array_data_type = typename internal::type_traits::array_data_type_helper<T>::type;

template <typename T>
inline typename std::enable_if<!has_size_method<T>::value, uint32>::type nb_components(const T& );
template <typename T>
inline typename std::enable_if<has_size_method<T>::value && has_begin_method<T>::value, uint32>::type nb_components(const T& val);
template <typename T>
inline typename std::enable_if<has_size_method<T>::value && !has_begin_method<T>::value && (!has_rows_method<T>::value || !has_cols_method<T>::value), uint32>::type nb_components(const T& val);
template <typename T>
inline typename std::enable_if<has_rows_method<T>::value && has_cols_method<T>::value, uint32>::type nb_components(const T& val);

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
inline typename std::enable_if<has_size_method<T>::value && !has_begin_method<T>::value && (!has_rows_method<T>::value || !has_cols_method<T>::value), uint32>::type nb_components(const T& val)
{
	return uint32(val.size()) * nb_components(val[0]);
}

template <typename T>
inline typename std::enable_if<has_rows_method<T>::value && has_cols_method<T>::value, uint32>::type nb_components(const T& val)
{
	return uint32(val.rows() * val.cols());
}



template<typename F>
using func_arity = std::integral_constant<std::size_t, internal::type_traits::function_traits<F>::arity>;


template<typename F>
using func_parameter_type = typename internal::type_traits::function_traits<F>::template arg<0>::type;

template<typename F, std::size_t i>
using func_ith_parameter_type = typename internal::type_traits::function_traits<F>::template arg<i>::type;

template<typename F, typename T>
using is_func_parameter_same = std::is_same<func_parameter_type<F>, T>;

template<typename F, std::size_t i, typename T>
using is_ith_func_parameter_same = std::is_same<func_ith_parameter_type<F,i>, T>;


template<typename F>
using func_return_type = typename internal::type_traits::function_traits<F>::result_type;

template<typename F, typename T>
using is_func_return_same = std::is_same<func_return_type<F>, T>;


namespace internal
{

template<typename FUNC, typename... Args>
inline typename std::enable_if<is_func_return_same<FUNC, void>::value, bool>::type void_to_true_binder(const FUNC& func, Args... args)
{
	func(std::forward<Args>(args)...);
	return true;
}

template<typename FUNC, typename... Args>
inline typename std::enable_if<is_func_return_same<FUNC, bool>::value, bool>::type void_to_true_binder(const FUNC& func, Args... args)
{
	return func(std::forward<Args>(args)...);
}

} // namespace internal
} // namespace cgogn

#endif // CGOGN_CORE_UTILS_TYPE_TRAITS_H_

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

#ifndef CORE_UTILS_NAME_TYPES_H_
#define CORE_UTILS_NAME_TYPES_H_

#include <typeinfo>
#include <string>
#include <vector>
#include <list>
#include <array>
#include <type_traits>
#include <regex>
#include <algorithm>
#include <cctype>

#ifdef __GNUG__
#include <cstdlib>
#include <memory>
#include <cxxabi.h>
#include <limits>
#include <iostream>
#endif // __GNUG__

#include <core/utils/dll.h>
#include <core/utils/definitions.h>

namespace cgogn
{

/**
* @brief function that give a name to a type.
* It relies on typeid except if the class T has a static function cgogn_name_of_type (returning a std::string)
* See class Cell and the file name_type_test.cpp
*/
template <typename T>
inline std::string name_of_type(const T& t);


namespace internal
{

template <class>
struct sfinae_true : std::true_type{};

template <class T>
static auto test_name_of_type(int ) -> sfinae_true<decltype(T::cgogn_name_of_type())>;
template <class>
static auto test_name_of_type(long) -> std::false_type;

template <class T>
struct has_cgogn_name_of_type : decltype(test_name_of_type<T>(0)){};

// implementation for classes which have a static cgogn_name_of_type() function (returning a std::string)
template <class T>
inline auto name_of_type_impl(const T&) -> typename std::enable_if<has_cgogn_name_of_type<T>::value == false, std::string>::type;

// implementation for other classes and type
template <class T>
inline auto name_of_type_impl(const T&) -> typename std::enable_if<has_cgogn_name_of_type<T>::value == true, std::string>::type;

template <typename T>
inline std::string name_of_type_impl(const std::list<T>&);

template <typename T>
inline std::string name_of_type_impl(const std::vector<T>&);

template <typename T>
inline std::string name_of_type_impl(const std::basic_string<T>&);

template <typename T>
inline std::string name_of_type_impl(const std::basic_string<T>&)
{
	return std::string("std::basic_string<") + name_of_type(T()) + std::string(">");
}

template <typename T>
inline std::string name_of_type_impl(const std::list<T>&)
{ return std::string("std::list<") + name_of_type(T()) + std::string(">"); }

template <typename T>
inline std::string name_of_type_impl(const std::vector<T>&)
{ return std::string("std::vector<") + name_of_type(T()) + std::string(">"); }


template <class T>
inline auto name_of_type_impl(const T&)->typename std::enable_if<has_cgogn_name_of_type<T>::value == true, std::string>::type
{
	return T::cgogn_name_of_type();
}

template <typename T>
inline auto name_of_type_impl(const T&)->typename std::enable_if<has_cgogn_name_of_type<T>::value == false, std::string>::type
{
	std::string type_name = typeid(T).name();
	static_assert(has_cgogn_name_of_type<int>::value == false, "plop");
#ifdef __GNUG__
	int status = std::numeric_limits<int>::max();
	std::unique_ptr<char, void(*)(void*)> res{ abi::__cxa_demangle(type_name.c_str(), NULL, NULL, &status), std::free };
	if (status == 0)
		type_name = std::string(res.get());
	else
		std::cerr << "__cxa_demangle exited with error code " << status << std::endl;

	// integer postfixes
	{
		std::regex regex("([0-9]+)(ul|l)", std::regex_constants::ECMAScript | std::regex_constants::icase);
		std::smatch m;
		std::regex_search(type_name, m, regex);
		if (!m[1].str().empty())
			type_name = std::regex_replace(type_name, regex, m[1].str());
	}

#else // __GNUG__
#ifdef _MSC_VER
	// fix MSVC displaying "__int64" instead of long long
	if (std::is_same<T, long long>::value)
		return "long long";
	if (std::is_same<T, unsigned long long>::value)
		return "unsigned long long";

	// removing all "class " and "struct" from type_name
	{
		std::regex regex_class("class ", std::regex_constants::ECMAScript);
		std::regex rege_struct("struct ", std::regex_constants::ECMAScript);
		type_name = std::regex_replace(type_name, regex_class, "");
		type_name = std::regex_replace(type_name, rege_struct, "");
	}

#endif // _MSC_VER
#endif // __GNUG__
	// removing spaces
	{
		std::regex regex("([a-z]*)([[:space:]]+)", std::regex_constants::ECMAScript | std::regex_constants::icase);
		std::smatch m;
		std::regex_search(type_name, m, regex);
		if (m[1].str().empty())
			type_name = std::regex_replace(type_name, regex, "");
	}
		return type_name;
}

} // namespace internal

template <typename T>
inline std::string name_of_type(const T& t)
{
	return internal::name_of_type_impl(t);
}

} // namespace cgogn

#endif // CORE_UTILS_NAME_TYPES_H_

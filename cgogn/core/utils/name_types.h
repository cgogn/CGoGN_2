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

#ifndef CGOGN_CORE_UTILS_NAME_TYPES_H_
#define CGOGN_CORE_UTILS_NAME_TYPES_H_

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
#include <limits>
#include <iostream>
#include <sstream>
#endif // __GNUG__

#include <cgogn/core/dll.h>
#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/utils/type_traits.h>
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

CGOGN_CORE_API std::string demangle(const std::string& str);



// implementation for classes which have a static cgogn_name_of_type() function (returning a std::string)
template <class T>
inline auto name_of_type_impl(const T&) -> typename std::enable_if<!has_cgogn_name_of_type<T>::value, std::string>::type;

// implementation for other classes and type

// declarations

template <class T>
inline auto name_of_type_impl(const T&) -> typename std::enable_if<has_cgogn_name_of_type<T>::value, std::string>::type;

template <typename T>
inline std::string name_of_type_impl(const std::list<T>&);

template <typename T>
inline std::string name_of_type_impl(const std::vector<T>&);

template <typename T>
inline std::string name_of_type_impl(const std::basic_string<T>&);

template <typename T, std::size_t N>
inline std::string name_of_type_impl(const std::array<T, N>&);

CGOGN_CORE_API std::string name_of_type_impl(const bool&);

CGOGN_CORE_API std::string name_of_type_impl(const int8&);

CGOGN_CORE_API std::string name_of_type_impl(const uint8&);

CGOGN_CORE_API std::string name_of_type_impl(const int16&);

CGOGN_CORE_API std::string name_of_type_impl(const uint16&);

CGOGN_CORE_API std::string name_of_type_impl(const int32&);

CGOGN_CORE_API std::string name_of_type_impl(const uint32&);

CGOGN_CORE_API std::string name_of_type_impl(const int64&);

CGOGN_CORE_API std::string name_of_type_impl(const uint64&);

CGOGN_CORE_API std::string name_of_type_impl(const float32&);

CGOGN_CORE_API std::string name_of_type_impl(const float64&);

// definitions

template <typename T>
inline std::string name_of_type_impl(const std::basic_string<T>&)
{ return std::string("std::basic_string<") + name_of_type(T()) + std::string(">"); }

template <typename T>
inline std::string name_of_type_impl(const std::list<T>&)
{ return std::string("std::list<") + name_of_type(T()) + std::string(">"); }

template <typename T>
inline std::string name_of_type_impl(const std::vector<T>&)
{ return std::string("std::vector<") + name_of_type(T()) + std::string(">"); }

template <typename T, std::size_t N>
inline std::string name_of_type_impl(const std::array<T, N>&)
{ return std::string("std::array<") + name_of_type(T()) + std::string(",") + std::to_string(N) + std::string(">"); }

template <class T>
inline auto name_of_type_impl(const T&) -> typename std::enable_if<has_cgogn_name_of_type<T>::value, std::string>::type
{
	return T::cgogn_name_of_type();
}

template <typename T>
inline auto name_of_type_impl(const T&) -> typename std::enable_if<!has_cgogn_name_of_type<T>::value, std::string>::type
{
	std::string type_name = demangle(std::string(typeid(T).name()));
#ifdef __GNUG__
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
	// removing all "class " and "struct" from type_name
	{
		std::regex regex_class("class ", std::regex_constants::ECMAScript);
		std::regex rege_struct("struct ", std::regex_constants::ECMAScript);
		type_name = std::regex_replace(type_name, regex_class, "");
		type_name = std::regex_replace(type_name, rege_struct, "");
	}

#endif // _MSC_VER
#endif // __GNUG__

#ifdef __APPLE__
	// removing std::__1
	{
		std::regex regex("std::__1::", std::regex_constants::ECMAScript);
		type_name = std::regex_replace(type_name, regex, "std::");
	}
#endif // __APPLE__

#ifdef _GLIBCXX_DEBUG
// replacing std::__debug:: by std::
	{
		std::regex regex("std::__debug::", std::regex_constants::ECMAScript);
		type_name = std::regex_replace(type_name, regex, "std::");
	}
#endif // _GLIBCXX_DEBUG

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

#endif // CGOGN_CORE_UTILS_NAME_TYPES_H_

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

#include <string>
#include <vector>
#include <list>
#include <array>

#include <core/utils/dll.h>
#include <core/utils/definitions.h>


namespace Eigen
{

// forward declaration
	template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
	class Matrix;

} // end namespace Eigen

namespace cgogn
{

/**
 * @brief function that give a name to a type.
 */
template <typename T>
std::string name_of_type(const T& )
{ return T::cgogn_name_of_type(); }

template <typename T>
std::string name_of_type(const std::list<T>& );

template <typename T>
std::string name_of_type(const std::vector<T>& );

template<typename T, std::size_t size>
std::string name_of_type(const std::array<T, size>&);

template<typename T>
std::string name_of_type(const std::basic_string<T>);

template <>
CGOGN_UTILS_API std::string name_of_type(const bool& );

template <>
CGOGN_UTILS_API std::string name_of_type(const char& );

template <>
CGOGN_UTILS_API std::string name_of_type(const wchar_t& );

// char16_t and char32_t are new fundamental types according to the c++11 standard. Regrettably, in MSVC they're just typedefs.
#ifndef _MSC_VER
template <>
CGOGN_UTILS_API std::string name_of_type(const char16_t& );

template <>
CGOGN_UTILS_API std::string name_of_type(const char32_t& );
#endif

template <>
CGOGN_UTILS_API std::string name_of_type(const short& );

template <>
CGOGN_UTILS_API std::string name_of_type(const int& );

template <>
CGOGN_UTILS_API std::string name_of_type(const long& );

template <>
CGOGN_UTILS_API std::string name_of_type(const long long& );

template <>
CGOGN_UTILS_API std::string name_of_type(const signed char& );

template <>
CGOGN_UTILS_API std::string name_of_type(const unsigned char& );

template <>
CGOGN_UTILS_API std::string name_of_type(const unsigned short& );

template <>
CGOGN_UTILS_API std::string name_of_type(const unsigned int& );

template <>
CGOGN_UTILS_API std::string name_of_type(const unsigned long& );

template <>
CGOGN_UTILS_API std::string name_of_type(const unsigned long long& );

template <>
CGOGN_UTILS_API std::string name_of_type(const float& );

template <>
CGOGN_UTILS_API std::string name_of_type(const double& );

template <>
CGOGN_UTILS_API std::string name_of_type(const std::string& );

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline std::string name_of_type(const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& );

template<typename T>
std::string name_of_type(const std::basic_string<T>)
{
	return std::string("std::basic_string<") + name_of_type(T()) + std::string(">");
}

template <typename T>
std::string name_of_type(const std::list<T>& )
{ return std::string("std::list<") + name_of_type(T()) + std::string(">"); }

template <typename T>
std::string name_of_type(const std::vector<T>& )
{ return std::string("std::vector<") + name_of_type(T()) + std::string(">"); }

template<typename T, std::size_t size>
std::string name_of_type(const std::array<T, size>&)
{
	return std::string("std::array<") + name_of_type(T()) + std::string(",") + std::to_string(size) + std::string(">");
}

/**
 * @brief add cgogn_name_of_type member to a class
 *
 * If the class that you want to use as attribute is not listed above
 * use AddTypeName<T> instead of T.
 * If you develop the class T, just add as public member: static std::string cgogn_name_of_type() { return "type_name_you_develop"; }
 */
template <typename T>
class AddTypeName : public T
{
public:

	static std::string cgogn_name_of_type() { return "UNKNOWN"; }
};

} // namespace cgogn

#endif // CORE_UTILS_NAME_TYPES_H_

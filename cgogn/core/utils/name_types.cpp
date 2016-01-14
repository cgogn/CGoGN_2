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

#define CGOGN_UTILS_DLL_EXPORT

#include <core/utils/name_types.h>
CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_ON
#include <Eigen/Core>
CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_OFF
namespace cgogn
{

template <>
CGOGN_UTILS_API std::string name_of_type(const bool& )
{ return "bool"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const char& )
{ return "char"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const wchar_t& )
{ return "wchar_t"; }

#ifndef _MSC_VER
template <>
CGOGN_UTILS_API std::string name_of_type(const char16_t& )
{ return "char16_t"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const char32_t& )
{ return "char32_t"; }
#endif

template <>
CGOGN_UTILS_API std::string name_of_type(const short& )
{ return "short"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const int& )
{ return "int"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const long& )
{ return "long"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const long long& )
{ return "long long"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const signed char& )
{ return "signed char"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const unsigned char& )
{ return "unsigned char"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const unsigned short& )
{ return "unsigned short"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const unsigned int& )
{ return "unsigned int"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const unsigned long& )
{ return "unsigned long"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const unsigned long long& )
{ return "unsigned long long"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const float& )
{ return "float"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const double& )
{ return "double"; }

template <>
CGOGN_UTILS_API std::string name_of_type(const Eigen::Matrix<double,3,1,0,3,1>& )
{
	static_assert(std::is_same<Eigen::Vector3d, Eigen::Matrix<double,3,1,0,3,1>>::value,"Eigen::Matrix<double,3,1,0,3,1> and Eigen::Vector3d have to be the same.");
	return "Eigen::Vector3d";
}

} // namespace cgogn

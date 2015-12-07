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

#define CGOGN_CORE_DLL_EXPORT
#include <core/basic/name_types.h>

namespace cgogn
{

template <>
CGOGN_CORE_API std::string name_of_type(const bool& /*v*/) { return "bool"; }

template <>
CGOGN_CORE_API std::string name_of_type(const char& /*v*/) { return "char"; }

template <>
CGOGN_CORE_API std::string name_of_type(const short& /*v*/) { return "short"; }

template <>
CGOGN_CORE_API std::string name_of_type(const int& /*v*/) { return "int"; }

template <>
CGOGN_CORE_API std::string name_of_type(const long& /*v*/) { return "long"; }

template <>
CGOGN_CORE_API std::string name_of_type(const long long& /*v*/) { return "long long"; }

template <>
CGOGN_CORE_API std::string name_of_type(const signed char& /*v*/) { return "signed char"; }

template <>
CGOGN_CORE_API std::string name_of_type(const unsigned char& /*v*/) { return "unsigned char"; }

template <>
CGOGN_CORE_API std::string name_of_type(const unsigned short& /*v*/) { return "unsigned short"; }

template <>
CGOGN_CORE_API std::string name_of_type(const unsigned int& /*v*/) { return "unsigned int"; }

template <>
CGOGN_CORE_API std::string name_of_type(const unsigned long& /*v*/) { return "unsigned long"; }

template <>
CGOGN_CORE_API std::string name_of_type(const unsigned long long& /*v*/) { return "unsigned long long"; }

template <>
CGOGN_CORE_API std::string name_of_type(const float& /*v*/) { return "float"; }

template <>
CGOGN_CORE_API std::string name_of_type(const double& /*v*/) { return "double"; }

template <>
CGOGN_CORE_API std::string name_of_type(const std::string& /*v*/) { return "std::string"; }


} // namespace cgogn


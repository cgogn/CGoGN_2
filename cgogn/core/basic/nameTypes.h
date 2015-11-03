/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *                                                                  *                                                                              *
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

#ifndef __CORE_BASIC_NAME_TYPES_H__
#define __CORE_BASIC_NAME_TYPES_H__

#include <string>
#include <vector>
#include <list>

namespace cgogn
{
/**
 * @brief function that give a name to a type.
 */
template <typename T>
std::string nameOfType(const T& v)
{
	return v.CGoGNnameOfType();
}

template <> inline std::string nameOfType(const bool& /*v*/) { return "bool"; }

template <> inline std::string nameOfType(const char& /*v*/) { return "char"; }

template <> inline std::string nameOfType(const short& /*v*/) { return "short"; }

template <> inline std::string nameOfType(const int& /*v*/) { return "int"; }

template <> inline std::string nameOfType(const long& /*v*/) { return "long"; }

template <> inline std::string nameOfType(const long long& /*v*/) { return "long long"; }

template <> inline std::string nameOfType(const unsigned char& /*v*/) { return "unsigned char"; }

template <> inline std::string nameOfType(const unsigned short& /*v*/) { return "unsigned short"; }

template <> inline std::string nameOfType(const unsigned int& /*v*/) { return "unsigned int"; }

template <> inline std::string nameOfType(const unsigned long& /*v*/) { return "unsigned long"; }

template <> inline std::string nameOfType(const unsigned long long& /*v*/) { return "unsigned long long"; }

template <> inline std::string nameOfType(const float& /*v*/) { return "float"; }

template <> inline std::string nameOfType(const double& /*v*/) { return "double"; }

template <> inline std::string nameOfType(const std::string& /*v*/) { return "std::string"; }

template <typename T> inline std::string nameOfType(const std::vector<T>& /*v*/) { return "std::vector<?>"; }

template <typename T> inline std::string nameOfType(const std::list<T>& /*v*/) { return "std::list<?>"; }


/**
 * @brief add CGoGNnameOfType member to a class
 *
 * If the class that you want to use as attribute is not basic type nor std::list/std::vector,
 * use AddTypeName<T> instead of T.
 * If you develop the class T, just add as public member: static std::string CGoGNnameOfType() { return "type_name_you_develop"; }
 */
template <typename T>
class AddTypeName : public T
{
public:
	static std::string CGoGNnameOfType() { return "UNKNOWN"; }
};


template <typename T>
bool typeIsBool(T)
{
	return false;
}

template <> bool typeIsBool(bool) { return true; }


}

#endif

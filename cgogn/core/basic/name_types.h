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

#ifndef CORE_BASIC_NAME_TYPES_H_
#define CORE_BASIC_NAME_TYPES_H_

#include <string>
#include <vector>
#include <list>

namespace cgogn
{

/**
 * @brief function that give a name to a type.
 */
template <typename T>
std::string name_of_type(const T& /*v*/)
{
	return T::CGoGN_name_of_type();
}

// first we need to declare some specializations
template <typename T>  std::string name_of_type(const std::list<T>& /*v*/);
template <typename T>  std::string name_of_type(const std::vector<T>& /*v*/);

template <>  std::string name_of_type(const bool& /*v*/);
template <>  std::string name_of_type(const char& /*v*/);

template <>  std::string name_of_type(const short& /*v*/);

template <>  std::string name_of_type(const int& /*v*/);

template <>  std::string name_of_type(const long& /*v*/);

template <>  std::string name_of_type(const long long& /*v*/);
// because signed char != char
template <>  std::string name_of_type(const signed char& /*v*/);

template <>  std::string name_of_type(const unsigned char& /*v*/);

template <>  std::string name_of_type(const unsigned short& /*v*/);

template <>  std::string name_of_type(const unsigned int& /*v*/);

template <>  std::string name_of_type(const unsigned long& /*v*/);

template <>  std::string name_of_type(const unsigned long long& /*v*/);

template <>  std::string name_of_type(const float& /*v*/);

template <>  std::string name_of_type(const double& /*v*/);

template <>  std::string name_of_type(const std::string& /*v*/);

template <typename T>  std::string name_of_type(const std::vector<T>& /*v*/) { return std::string("std::vector<") + name_of_type(T()) + std::string(">"); }
template <typename T>  std::string name_of_type(const std::list<T>& /*v*/) { return std::string("std::list<")+ name_of_type(T()) + std::string(">"); }


/**
 * @brief add CGoGNname_of_type member to a class
 *
 * If the class that you want to use as attribute is not basic type nor std::list/std::vector,
 * use AddTypeName<T> instead of T.
 * If you develop the class T, just add as public member: static std::string CGoGNname_of_type() { return "type_name_you_develop"; }
 */
template <typename T>
class AddTypeName : public T
{
public:

	static std::string CGoGN_name_of_type() { return "UNKNOWN"; }
};

} // namespace cgogn

#endif // CORE_BASIC_NAME_TYPES_H_

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

#ifndef CGOGN_CORE_UTILS_STRING_H_
#define CGOGN_CORE_UTILS_STRING_H_


#include <string>
#include <cctype>

namespace cgogn
{

template <typename Char_T>
inline std::basic_string<Char_T>	to_upper(const std::basic_string<Char_T>& str)
{
	std::basic_string<Char_T> res(str);
	for (auto& c : res)
		c = Char_T(std::toupper(c));
	return res;
}

template <typename Char_T>
inline std::basic_string<Char_T>	to_lower(const std::basic_string<Char_T>& str)
{
	std::basic_string<Char_T> res(str);
	for (auto& c : res)
		c = Char_T(std::tolower(c));
	return res;
}

template <typename Char_T>
inline std::basic_string<Char_T>	get_extension(const std::basic_string<Char_T>& str)
{
	std::size_t dot = str.rfind('.');
	if (dot == std::basic_string<Char_T>::npos || dot == str.size() -1u)
		return std::basic_string<Char_T>();
	return str.substr(dot + 1u);
}

} // namespace cgogn

#endif // CGOGN_CORE_UTILS_STRING_H_

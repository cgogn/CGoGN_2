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
#include <locale>
#include <iostream>

namespace cgogn
{

template <typename Char_T>
inline std::basic_string<Char_T> to_upper(const std::basic_string<Char_T>& str)
{
	const std::locale locale;
	std::basic_string<Char_T> res(str);
	for (auto& c : res)
		c = Char_T(std::toupper(c,locale));
	return res;
}

template <typename Char_T>
inline std::basic_string<Char_T> to_lower(const std::basic_string<Char_T>& str)
{
	const std::locale locale;
	std::basic_string<Char_T> res(str);
	for (auto& c : res)
		c = Char_T(std::tolower(c,locale));
	return res;
}

template <typename Char_T>
inline std::basic_string<Char_T> extension(const std::basic_string<Char_T>& str)
{
	std::size_t dot = str.rfind('.');
	if (dot == std::basic_string<Char_T>::npos || dot == str.size() -1u)
		return std::basic_string<Char_T>();
	return str.substr(dot + 1u);
}

template <typename Char_T>
inline std::basic_string<Char_T> remove_extension(const std::basic_string<Char_T>& str)
{
	std::size_t dot = str.rfind('.');
	if (dot == std::basic_string<Char_T>::npos)
		return str;
	else
		return str.substr(0,dot);
}

template <typename Char_T>
inline bool equal_case_insensitive(const std::basic_string<Char_T>& str1, const std::basic_string<Char_T>& str2)
{
	if (str1.size() != str2.size())
		return false;
	auto it1 = str1.begin(), it2 = str2.begin();
	for(auto end = str1.end(); it1 != end ;)
	{
		if (std::tolower(*it1) != std::tolower(*it2))
			return false;
		++it1;
		++it2;
	}
	return true;
}

} // namespace cgogn

#endif // CGOGN_CORE_UTILS_STRING_H_

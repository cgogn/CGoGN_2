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

#ifdef __GNUG__
#include <memory>
#include <cxxabi.h>
#endif // __GNUG__


namespace cgogn
{

namespace internal
{

/**
 * @brief demangle, trying demangling a typename using the cxxabi
 * @param str, a type name
 * @return the demangled type name is succeded, otherwise a copy of str
 */
CGOGN_UTILS_API std::string demangle(const std::string& str)
{
#ifndef __GNUG__
	return str;
#else
	int status = std::numeric_limits<int>::max();
	std::unique_ptr<char, void(*)(void*)> res{ abi::__cxa_demangle(str.c_str(), NULL, NULL, &status), std::free };
	if (status == 0)
		return std::string(res.get());
	else
		std::cerr << "__cxa_demangle exited with error code " << status << std::endl;
	return str;
#endif // __GNUG__
}

} // namespace internal
} // namespace cgogn

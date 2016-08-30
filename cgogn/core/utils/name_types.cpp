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


#include <cgogn/core/utils/logger.h>
#include <cgogn/core/utils/name_types.h>

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
CGOGN_CORE_API std::string demangle(const std::string& str)
{
#ifndef __GNUG__
	return str;
#else
	int status = std::numeric_limits<int>::max();
	std::unique_ptr<char, void(*)(void*)> res{ abi::__cxa_demangle(str.c_str(), NULL, NULL, &status), std::free };
	if (status == 0)
		return std::string(res.get());
	else
		cgogn_log_warning("demangle") << "__cxa_demangle exited with error code " << status << ".";
	return str;
#endif // __GNUG__
}

CGOGN_CORE_API std::string name_of_type_impl(const bool&)
{
	return "bool";
}

CGOGN_CORE_API std::string name_of_type_impl(const int8&)
{
	return "int8";
}

CGOGN_CORE_API std::string name_of_type_impl(const uint8&)
{
	return "uint8";
}

CGOGN_CORE_API std::string name_of_type_impl(const int16&)
{
	return "int16";
}

CGOGN_CORE_API std::string name_of_type_impl(const uint16&)
{
	return "uint16";
}

CGOGN_CORE_API std::string name_of_type_impl(const int32&)
{
	return "int32";
}

CGOGN_CORE_API std::string name_of_type_impl(const uint32&)
{
	return "uint32";
}

CGOGN_CORE_API std::string name_of_type_impl(const int64&)
{
	return "int64";
}

CGOGN_CORE_API std::string name_of_type_impl(const uint64&)
{
	return "uint64";
}

CGOGN_CORE_API std::string name_of_type_impl(const float32&)
{
	return "float32";
}

CGOGN_CORE_API std::string name_of_type_impl(const float64&)
{
	return "float64";
}

} // namespace internal

} // namespace cgogn

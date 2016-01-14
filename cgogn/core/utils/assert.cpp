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

#include <core/utils/dll.h>
#include <core/utils/assert.h>
#include <iostream>
#include <sstream>
#include <cstdlib>

namespace cgogn
{

CGOGN_UTILS_API CGOGN_NORETURN void assertion_failed(
	const std::string& expression,
	const std::string& message,
	const std::string& file_name,
	const std::string& function_name,
	int line_number
)
{
	std::ostringstream os;
	os << "Assertion failed: " << expression;
	if (message.empty())
		os << ".\n";
	else
		os << " (" << message << ").\n";
	os << "file: " << file_name << ", function: " << function_name << ", line: " << line_number;

	std::cerr << os.str() << std::endl;
	std::abort();
}

CGOGN_UTILS_API CGOGN_NORETURN void should_not_have_reached(
	const std::string& message,
	const std::string& file_name,
	const std::string& function_name,
	int line_number
)
{
 	std::ostringstream os;
	os << "Should not have reached this point";
	if (message.empty())
		os << ".\n";
	else
		os << " (" << message << ").\n";
	os << "file: " << file_name << ", function: " << function_name << ", line: " << line_number;

	std::cerr << os.str() << std::endl;
	std::abort();
}

} // namespace cgogn

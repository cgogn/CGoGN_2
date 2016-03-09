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

#ifndef IO_C_LOCALE_H_
#define IO_C_LOCALE_H_

#include <string>
#include <clocale>

namespace cgogn
{

class Scoped_C_Locale
{
	std::string current_locale_;
public:

	/// set numeric locale to C after saving current locale
	inline Scoped_C_Locale()
	{
		current_locale_ = std::string(std::setlocale(LC_NUMERIC, NULL));
		setlocale(LC_NUMERIC, "C");
	}

	/// restore locale
	inline ~Scoped_C_Locale()
	{
		std::setlocale(LC_NUMERIC, current_locale_.c_str());
	}
};

} // namespace cgogn

#endif // IO_C_LOCALE_H_

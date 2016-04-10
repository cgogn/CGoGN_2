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

#ifndef CGOGN_IO_C_LOCALE_H_
#define CGOGN_IO_C_LOCALE_H_

#include <string>
#include <clocale>

#include <cgogn/core/utils/numerics.h>

namespace cgogn
{

class Scoped_C_Locale
{

public:
	using Self = Scoped_C_Locale;


	/// set numeric locale to C after saving current locale
	inline Scoped_C_Locale()
	{
		current_locale_ = std::string(std::setlocale(LC_NUMERIC, NULL));
		setlocale(LC_NUMERIC, "C");
	}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Scoped_C_Locale);

	/// restore locale
	inline ~Scoped_C_Locale()
	{
		std::setlocale(LC_NUMERIC, current_locale_.c_str());
	}
	private:
		std::string current_locale_;
};

} // namespace cgogn

#endif // CGOGN_IO_C_LOCALE_H_

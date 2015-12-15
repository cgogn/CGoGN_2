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

#ifndef UTILS_FAKE_ARG_USED_H_
#define UTILS_FAKE_ARG_USED_H_

/**
 * \file cgogn/utils/fake_arg_used.h
 * \brief A function to suppress unused parameters compilation warnings
 */
namespace cgogn
{
	/**
	 * \brief Suppresses compiler warnings about unused parameters
	 * \details This function is intended to make warnings silent
	 * concerning non used parameters. The corresponding code
	 * is supposed to be wiped out by the optimizer.
	 * (Removed starting from -O1 with gcc and clang)
	 */
	template <class T>
	inline void fake_arg_used(const T&)
	{}
}

#endif // UTILS_FAKE_ARG_USED_H_

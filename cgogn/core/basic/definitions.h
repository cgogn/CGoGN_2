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

#ifndef __CORE_BASIC_DEFINITIONS_H__
#define __CORE_BASIC_DEFINITIONS_H__

/**
 * \file basic/definitions.h
 * \brief Basic definitions for CGOGN API
 */


#ifdef WIN32
#ifndef CGoGN_CORE_API
#if defined CGoGN_CORE_DLL_EXPORT
#define CGoGN_CORE_API __declspec(dllexport)
#else
#define CGoGN_CORE_API __declspec(dllimport)
#endif
#endif
#else
#define CGoGN_CORE_API
#endif

#if defined(_MSC_VER) && _MSC_VER < 1900
#define CGOGN_NOEXCEPT
#else
#define CGOGN_NOEXCEPT noexcept
#endif

namespace cgogn
{

const unsigned int NB_THREADS = 8u;

} // namespace cgogn

#endif // __CORE_BASIC_DEFINITIONS_H__

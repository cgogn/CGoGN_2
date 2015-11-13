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

#ifndef UTILS_BASIC_DEFINITIONS_H_
#define UTILS_BASIC_DEFINITIONS_H_

/**
 * \brief No execpt declaration for CGOGN symbols.
 */
#if defined(_MSC_VER) && _MSC_VER < 1900
#define CGOGN_NOEXCEPT throw()
#else
#define CGOGN_NOEXCEPT noexcept
#endif

/*
 * Thread local storage. In VS <1900 it works only with POD types.
*/
#if defined(_MSC_VER) && _MSC_VER < 1900
#define CGOGN_TLS __declspec( thread )
#else
#define CGOGN_TLS thread_local
#endif

/**
 * \brief No return declaration for CGOGN symbols.
 */
#ifndef CGOGN_NORETURN
#if defined (_MSC_VER)
#define CGOGN_NORETURN __declspec(noreturn)
#else
#define CGOGN_NORETURN [[noreturn]]
#endif
#endif

/**
 * \def CGOGN_DEBUG
 * \brief This macro is set when compiling in debug mode
 *
 * \def CGOGN_PARANO
 * \brief This macro is set when compiling in debug mode
 */
#ifdef NDEBUG
#undef CGOGN_DEBUG
#undef CGOGN_PARANO
#else
#define CGOGN_DEBUG
#define CGOGN_PARANO
#endif

namespace cgogn
{

/**
 * \brief The maximum nunmber of threads created by the API.
 */
const unsigned int NB_THREADS = 8u;

} // namespace cgogn

#endif // UTILS_BASIC_DEFINITIONS_H_

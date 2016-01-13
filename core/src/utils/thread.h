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

#ifndef UTILS_THREAD_H_
#define UTILS_THREAD_H_

#include <utils/buffers.h>
#include <utils/dll.h>

namespace cgogn
{

/**
 * \brief The maximum nunmber of threads created by the API.
 */
const unsigned int NB_THREADS = 8u;

/// buffers of pre-allocated vectors of dart or unsigned int
extern CGOGN_TLS Buffers<Dart>* dart_buffers_thread;
extern CGOGN_TLS Buffers<unsigned int>* uint_buffers_thread;

/**
 * @brief function to call at begin of each thread which use a map
 */
CGOGN_UTILS_API void thread_start();

/**
 * @brief function to call at end of each thread which use a map
 */
CGOGN_UTILS_API void thread_stop();

CGOGN_UTILS_API Buffers<Dart>*         get_dart_buffers();
CGOGN_UTILS_API Buffers<unsigned int>* get_uint_buffers();

} // namespace cgogn

#endif // UTILS_THREAD_H_

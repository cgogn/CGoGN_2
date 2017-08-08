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

#ifndef CGOGN_CORE_UTILS_THREAD_H_
#define CGOGN_CORE_UTILS_THREAD_H_

#include <thread>
#include <vector>

#include <cgogn/core/dll.h>
#include <cgogn/core/utils/numerics.h>

#include <cgogn/core/basic/dart.h>

namespace cgogn
{

// forward declaration of the ThreadPool class
class ThreadPool;

// forward declaration of the Buffers class
template <typename T>
class Buffers;

CGOGN_CORE_API ThreadPool* thread_pool();

CGOGN_CORE_API ThreadPool* external_thread_pool();


const uint32 PARALLEL_BUFFER_SIZE = 1024u;

/// buffers of pre-allocated vectors of dart or uint32
extern CGOGN_TLS Buffers<Dart>* dart_buffers_thread_;
extern CGOGN_TLS Buffers<uint32>* uint_buffers_thread_;

extern CGOGN_TLS uint32 thread_marker_index_;
extern CGOGN_TLS uint32 thread_index_;

/**
 * @brief
 *
 */
/**
 * @brief function to call at begin of each thread which use a map
 * @param ind index of thread in the pool: 0,1,2,3,....
 * @param shift_marker_index 0 for main thread / 1 for internal thread_pool / 1+max_nb_workers for external thread
 */
CGOGN_CORE_API void thread_start(uint32 ind, uint32 shift_marker_index);

/**
 * @brief function to call at end of each thread which use a map
 */
CGOGN_CORE_API void thread_stop();

CGOGN_CORE_API Buffers<Dart>*   dart_buffers();
CGOGN_CORE_API Buffers<uint32>* uint_buffers();
/**
 * @brief thread index in marker table (internal use only)
 */
CGOGN_CORE_API uint32 current_thread_marker_index();

/**
 * @brief thread index [0..nb_workers] for use in code of lambdas
 */
CGOGN_CORE_API uint32 current_thread_index();



} // namespace cgogn

#endif // CGOGN_CORE_UTILS_THREAD_H_

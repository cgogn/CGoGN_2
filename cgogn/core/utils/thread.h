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

/**
 * \brief The maximum nunmber of threads created by the API.
 */
const uint32 MAX_NB_THREADS = 8u;
CGOGN_CORE_API extern uint32 NB_THREADS;

CGOGN_CORE_API ThreadPool* thread_pool();

inline uint32 nb_threads()
{
	uint32 c = std::thread::hardware_concurrency();
	return c < MAX_NB_THREADS ? c : MAX_NB_THREADS;
}

const uint32 PARALLEL_BUFFER_SIZE = 1024u;

/// buffers of pre-allocated vectors of dart or uint32
extern CGOGN_TLS Buffers<Dart>* dart_buffers_thread_;
extern CGOGN_TLS Buffers<uint32>* uint_buffers_thread_;

/**
 * @brief function to call at begin of each thread which use a map
 */
CGOGN_CORE_API void thread_start();

/**
 * @brief function to call at end of each thread which use a map
 */
CGOGN_CORE_API void thread_stop();

CGOGN_CORE_API Buffers<Dart>*   dart_buffers();
CGOGN_CORE_API Buffers<uint32>* uint_buffers();

} // namespace cgogn

#endif // CGOGN_CORE_UTILS_THREAD_H_

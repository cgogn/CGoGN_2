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

#include <core/utils/thread.h>
#include <core/utils/buffers.h>
#include <core/utils/thread_pool.h>

namespace cgogn
{

CGOGN_UTILS_API unsigned int NB_THREADS = get_nb_threads();

CGOGN_TLS Buffers<Dart>* dart_buffers_thread = nullptr;
CGOGN_TLS Buffers<unsigned int>* uint_buffers_thread = nullptr;

CGOGN_UTILS_API void thread_start()
{
	if (dart_buffers_thread == nullptr)
		dart_buffers_thread = new Buffers<Dart>();

	if (uint_buffers_thread == nullptr)
		uint_buffers_thread = new Buffers<unsigned int>();
}

CGOGN_UTILS_API void thread_stop()
{
	delete dart_buffers_thread;
	delete uint_buffers_thread;
	dart_buffers_thread = nullptr;
	uint_buffers_thread = nullptr;
}

CGOGN_UTILS_API Buffers<Dart>* get_dart_buffers()
{
	return dart_buffers_thread;
}

CGOGN_UTILS_API Buffers<unsigned int>* get_uint_buffers()
{
	return uint_buffers_thread;
}

CGOGN_UTILS_API ThreadPool* get_thread_pool()
{
	static std::unique_ptr<ThreadPool> pool(new ThreadPool);
	return pool.get();
}

} // namespace cgogn

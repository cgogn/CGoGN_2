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


#include <cgogn/core/utils/thread.h>
#include <cgogn/core/utils/buffers.h>
#include <cgogn/core/utils/thread_pool.h>

namespace cgogn
{
CGOGN_TLS Buffers<Dart>* dart_buffers_thread_ = nullptr;
CGOGN_TLS Buffers<uint32>* uint_buffers_thread_ = nullptr;
CGOGN_TLS uint32 thread_marker_index_;
CGOGN_TLS uint32 thread_index_;


CGOGN_CORE_API void thread_start(uint32 ind, uint32 shift_marker_index)
{
	if (dart_buffers_thread_ == nullptr)
		dart_buffers_thread_ = new Buffers<Dart>();

	if (uint_buffers_thread_ == nullptr)
		uint_buffers_thread_ = new Buffers<uint32>();

	thread_index_ = ind;
	thread_marker_index_ = ind + shift_marker_index;
}

CGOGN_CORE_API void thread_stop()
{
	delete dart_buffers_thread_;
	delete uint_buffers_thread_;
	dart_buffers_thread_ = nullptr;
	uint_buffers_thread_ = nullptr;
}

CGOGN_CORE_API Buffers<Dart>* dart_buffers()
{
	return dart_buffers_thread_;
}

CGOGN_CORE_API Buffers<uint32>* uint_buffers()
{
	return uint_buffers_thread_;
}

CGOGN_CORE_API uint32 current_thread_marker_index()
{
	return thread_marker_index_;
}


CGOGN_CORE_API uint32 current_thread_index()
{
	return thread_index_;
}


CGOGN_CORE_API ThreadPool* thread_pool()
{
	// thread safe accoring to http://stackoverflow.com/questions/8102125/is-local-static-variable-initialization-thread-safe-in-c11
	static ThreadPool pool("internal",1);
	return &pool;
}

CGOGN_CORE_API ThreadPool* external_thread_pool()
{
	// thread safe accoring to http://stackoverflow.com/questions/8102125/is-local-static-variable-initialization-thread-safe-in-c11
	static ThreadPool u_pool("external",1+ std::thread::hardware_concurrency());
	return &u_pool;
}

} // namespace cgogn

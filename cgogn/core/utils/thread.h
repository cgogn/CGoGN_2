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

#ifndef CORE_UTILS_THREAD_H_
#define CORE_UTILS_THREAD_H_

#include <core/utils/buffers.h>
#include <core/utils/dll.h>
#include <core/utils/thread_barrier.h>
#include <core/utils/thread_pool.h>

namespace cgogn
{

/**
 * \brief The maximum nunmber of threads created by the API.
 */
const unsigned int MAX_NB_THREADS = 8u;
CGOGN_UTILS_API extern unsigned int NB_THREADS;

CGOGN_UTILS_API ThreadPool* get_thread_pool();

inline unsigned int get_nb_threads()
{
	unsigned int c = std::thread::hardware_concurrency();
	return c < MAX_NB_THREADS ? c : MAX_NB_THREADS;
}

const unsigned int PARALLEL_BUFFER_SIZE = 1024u;

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

template <typename ELEM, typename FUNC>
class ThreadFunction
{
private:

	FUNC f_;
	std::vector<ELEM>& elements_;
	Barrier& sync1_;
	Barrier& sync2_;
	bool& finished_;
	unsigned int thread_order_;

public:

	ThreadFunction(
		FUNC f,
		std::vector<ELEM>& elements,
		Barrier& sync1,
		Barrier& sync2,
		bool& finished,
		unsigned int thread_order
	) :
		f_(f),
		elements_(elements),
		sync1_(sync1),
		sync2_(sync2),
		finished_(finished),
		thread_order_(thread_order)
	{}

	void operator()()
	{
		thread_start();
		while (true)
		{
			sync2_.wait(); // wait for vectors to be filled
			if (finished_)
				break;
			for (ELEM& e : elements_)
				f_(e, thread_order_);
			sync1_.wait(); // wait every thread has finished
		}
		thread_stop();
	}
};

} // namespace cgogn

#endif // CORE_UTILS_THREAD_H_

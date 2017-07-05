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


#include <cgogn/core/utils/thread_pool.h>

namespace cgogn

{

std::vector<std::thread::id> ThreadPool::threads_ids() const
{
	std::vector<std::thread::id> res;
	res.reserve(workers_.size());
	for (const std::thread& w : workers_)
		res.push_back(w.get_id());
	return res;
}

ThreadPool::~ThreadPool()
{
	{
		std::unique_lock<std::mutex> lock(queue_mutex_);
		stop_ = true;
	}
#if !(defined(CGOGN_WIN_VER) && (CGOGN_WIN_VER <= 61))
	condition_.notify_all();
#endif
	for(std::thread &worker: workers_)
		worker.join();
}

void ThreadPool::init_pool(uint32 nb_workers)
{

	// wait tasks to finish
	{
		std::unique_lock<std::mutex> lock(queue_mutex_);
		stop_ = true;
	}
#if !(defined(CGOGN_WIN_VER) && (CGOGN_WIN_VER <= 61))
	condition_.notify_all();
#endif
	for(std::thread &worker: workers_)
		worker.join();
	{
		std::unique_lock<std::mutex> lock(queue_mutex_);
		stop_ = false;
	}



	for(uint32 i = 0u; i< nb_workers; ++i)
	{
		workers_.emplace_back(
		[this, i]
		{
			cgogn::thread_start();
			for(;;)
			{
				PackagedTask task;
				{
					std::unique_lock<std::mutex> lock(this->queue_mutex_);
					this->condition_.wait(
						lock,
						[this] { return this->stop_ || !this->tasks_.empty(); }
					);
					if(this->stop_ && this->tasks_.empty())
					{
						cgogn::thread_stop();
						return;
					}

					task = std::move(this->tasks_.front());
					this->tasks_.pop();
				}
#if defined(_MSC_VER) && _MSC_VER < 1900
				(*task)(i);
#else
				task(i);
#endif
			}
		});
	}
}

ThreadPool::ThreadPool()
	: stop_(false), cpu_usage_(1.0f)
{
	uint32 nb_working_threads = std::min(std::thread::hardware_concurrency(),MAX_NB_THREADS); // possible to do -1 only if nbt==8 ?
	init_pool(nb_working_threads);
}

} // namespace cgogn


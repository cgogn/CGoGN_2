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

#include <core/utils/thread_pool.h>

namespace cgogn

{

std::vector<std::thread::id> ThreadPool::get_threads_ids() const
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
	condition_.notify_all();
	for(std::thread &worker: workers_)
		worker.join();
}

ThreadPool::ThreadPool()
	: stop_(false)
{
	for(unsigned int i = 0u; i< cgogn::get_nb_threads() -1u;++i)
		workers_.emplace_back(
					[this,i]
		{
			cgogn::thread_start();
			for(;;)
			{
				std::function<void(unsigned int)> task;

				{
					std::unique_lock<std::mutex> lock(this->queue_mutex_);
					this->condition_.wait(lock,
										  [this]{ return this->stop_ || !this->tasks_.empty(); });
					if(this->stop_ && this->tasks_.empty())
					{
						cgogn::thread_stop();
						return;
					}

					task = std::move(this->tasks_.front());
					this->tasks_.pop();
				}
				task(i);
			}
		}
		);
}

} // namespace cgogn


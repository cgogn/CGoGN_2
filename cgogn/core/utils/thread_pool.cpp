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



ThreadPool::ThreadPool(uint32 shift_index)
	:  stop_(false), shift_index_(shift_index)
{
	uint32 nb_ww = std::thread::hardware_concurrency();
	this->nb_working_workers_ = nb_ww;
	for(uint32 i = 0u; i< nb_ww; ++i)
	{
		workers_.emplace_back(
		[this, i] () -> void
		{
			cgogn::thread_start(this->shift_index_+i);
			for(;;)
			{
				while (i >= this->nb_working_workers_)
				{
					std::unique_lock<std::mutex> lock(this->running_mutex_);
					this->condition_running_.wait(lock);
				}

				std::unique_lock<std::mutex> lock(this->queue_mutex_);
				this->condition_.wait(
					lock,
					[this] { return this->stop_ || !this->tasks_.empty(); }
				);

				if (this->stop_ && this->tasks_.empty())
				{
					cgogn::thread_stop();
					return;
				}

				if (i < this->nb_working_workers_)
				{
					PackagedTask task = std::move(this->tasks_.front());
					this->tasks_.pop();
					lock.unlock();
#if defined(_MSC_VER) && _MSC_VER < 1900
					(*task)(i);
#else
					task(i);
#endif
				}
				else
				{
					lock.unlock();
					condition_.notify_one();
				}
			}
		});
	}
}



void ThreadPool::set_nb_workers(uint32 nb )
{
	if (nb == 0xffffffff)
		nb_working_workers_ = uint32(workers_.size());
	else
		nb_working_workers_ = std::min(uint32(workers_.size()), nb);

	condition_running_.notify_all();

	cgogn_log_info("ThreadPool") << "using " << nb_working_workers_ << " thread-workers";
}

} // namespace cgogn


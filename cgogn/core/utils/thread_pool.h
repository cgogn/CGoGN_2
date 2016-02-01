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

#ifndef CORE_UTILS_THREADPOOL_H_
#define CORE_UTILS_THREADPOOL_H_

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <atomic>

#include <core/utils/assert.h>
#include <core/utils/thread.h>

namespace cgogn
{

class ThreadPool {
public:
	ThreadPool();
	ThreadPool(const ThreadPool&) = delete;
	ThreadPool& operator=(const ThreadPool&) = delete;
	ThreadPool(ThreadPool&&) = delete;
	ThreadPool& operator=(ThreadPool&&) = delete;

	template<class F, class... Args>
	auto enqueue(F&& f, Args&&... args)
	-> std::future<typename std::result_of<F(unsigned int, Args...)>::type>;

	template<class F, class... Args>
	void enqueue_no_return(F&& f, Args&&... args);

	std::vector<std::thread::id> get_threads_ids() const;
	virtual ~ThreadPool();

private:
	// need to keep track of threads so we can join them
	std::vector< std::thread > workers_;
	// the task queue
	std::queue< std::function<void(unsigned int)> > tasks_;

	// synchronization
	std::mutex queue_mutex_;
	std::condition_variable condition_;
	std::atomic_bool stop_;
};


// add new work item to the pool
template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args)
-> std::future<typename std::result_of<F(unsigned int, Args...)>::type>
{
	using return_type = typename std::result_of<F(unsigned int, Args...)>::type;

	// don't allow enqueueing after stopping the pool
	if(stop_)
		cgogn_assert_not_reached("enqueue on stopped ThreadPool");

	auto task = std::make_shared< std::packaged_task<return_type(unsigned int)> >([&](unsigned int i)
	{
		return std::bind(std::forward<F>(f), i, std::forward<Args>(args)...)();
	}
	);

	std::future<return_type> res = task->get_future();
	{
		std::unique_lock<std::mutex> lock(queue_mutex_);
		// Push work back on the queue
		tasks_.emplace([task](unsigned int i){ (*task)(i); });
	}
	// Notify a thread that there is new work to perform
	condition_.notify_one();
	return res;
}

template<class F, class... Args>
void ThreadPool::enqueue_no_return(F&& f, Args&&... args)
{
	// don't allow enqueueing after stopping the pool
	if(stop_)
		cgogn_assert_not_reached("enqueue on stopped ThreadPool");
	{
		std::unique_lock<std::mutex> lock(queue_mutex_);

		// Push work back on the queue
		tasks_.emplace ([&](unsigned int i)
	{
		std::bind(std::forward<F>(f), i, std::forward<Args>(args)...)();
	});
	}

	// Notify a thread that there is new work to perform
	condition_.notify_one ();
}

} // namespace cgogn

#endif // CORE_UTILS_THREADPOOL_H_

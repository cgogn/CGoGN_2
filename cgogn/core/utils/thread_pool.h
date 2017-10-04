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

/*
 * IMPORTANT : The ThreadPool code (thread_pool.h and thread_pool.cpp) is
 * based on "A Simple c++11 threadpool implementation" found on github
 * (https://github.com/progschj/ThreadPool, latest commit : 9a42ec1 )
 * (c) 2012 Jakob Progsch, Václav Zeman
 * It has been modified to fit to our purposes.
 * A copy of its license is provided in the following lines.
 */

/****************************************************************************
*Copyright (c) 2012 Jakob Progsch, Václav Zeman                             *
*                                                                           *
*This software is provided 'as-is', without any express or implied          *
*warranty. In no event will the authors be held liable for any damages      *
*arising from the use of this software.                                     *
*                                                                           *
*Permission is granted to anyone to use this software for any purpose,      *
*including commercial applications, and to alter it and redistribute it     *
*freely, subject to the following restrictions:                             *
*                                                                           *
*1. The origin of this software must not be misrepresented; you must not    *
*claim that you wrote the original software. If you use this software       *
*in a product, an acknowledgment in the product documentation would be      *
*appreciated but is not required.                                           *
*                                                                           *
*2. Altered source versions must be plainly marked as such, and must not be *
*misrepresented as being the original software.                             *
*                                                                           *
*3. This notice may not be removed or altered from any source               *
*distribution.                                                              *
****************************************************************************/

#ifndef CGOGN_CORE_UTILS_THREADPOOL_H_
#define CGOGN_CORE_UTILS_THREADPOOL_H_

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>

#include <cgogn/core/utils/logger.h>
#include <cgogn/core/utils/assert.h>
#include <cgogn/core/utils/thread.h>

namespace cgogn
{

class CGOGN_CORE_API ThreadPool final
{
public:

	ThreadPool(const std::string& name, uint32 shift_index);
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(ThreadPool);

#if defined(_MSC_VER) && _MSC_VER < 1900
	using PackagedTask = std::shared_ptr<std::packaged_task<void()>>; // avoiding a MSVC 2013 Bug
#else
	using PackagedTask = std::packaged_task<void()>;
#endif

	template <class F, class... Args>
	std::future<void> enqueue(const F& f, Args&&... args);

	~ThreadPool();

	/**
	 * @brief get the number of currently working thread for parallel algos
	 */
	inline uint32 nb_workers() const
	{
		return nb_working_workers_;
	}

	/**
	* @brief get the number of threads that could be used for parallel algos
	*/
	inline uint32 max_nb_workers() const
	{
		return uint32(workers_.size());
	}

	/**
	 * @brief set nb working threads for parallel algos ( no param = full power)
	 * @param nb [0,nb_max_workers()] (for 0 parallel algo are replaced by normal version)
	 */
	void set_nb_workers(uint32 nb = 0xffffffff);

private:
#pragma warning(push)
#pragma warning(disable:4251)

	// just info log
	std::string name_;

	// need to keep track of threads so we can join them
	std::vector<std::thread> workers_;
	// the task queue
	std::queue<PackagedTask> tasks_;

	// synchronization
	std::mutex queue_mutex_;
	std::condition_variable condition_;
	bool stop_;

	// limit usage to the n-th first workers
	uint32 nb_working_workers_;
	std::mutex running_mutex_;
	std::condition_variable condition_running_;

	uint32 shift_index_;

#pragma warning(pop)
};

// add new work item to the pool


template <class F, class... Args>
std::future<void> ThreadPool::enqueue(const F& f, Args&&... args)
{
	static_assert(std::is_same<typename std::result_of<F(Args...)>::type,void>::value,"The thread pool only accept non-returning functions.");

#if defined(_MSC_VER) && _MSC_VER < 1900
	PackagedTask task = std::make_shared<std::packaged_task<void()>>(std::bind(f, std::forward<Args>(args)...));
	std::future<void> res = task->get_future();
#else
	PackagedTask task([&, f]() -> void
	{
		f(std::forward<Args>(args)...);
	});
	std::future<void> res = task.get_future();
#endif

	{
		std::unique_lock<std::mutex> lock(queue_mutex_);
		// don't allow enqueueing after stopping the pool
		if (stop_)
		{
			cgogn_log_error("ThreadPool::enqueue") << "Enqueue on stopped ThreadPool.";
			cgogn_assert_not_reached("enqueue on stopped ThreadPool");
		}
		// Push work back on the queue
		tasks_.push(std::move(task));
	}
	// Notify a thread that there is new work to perform
	condition_.notify_one();
	return res;
}

/**
 * launch an external thread
 */
template <class F, class... Args>
std::future<void> launch_thread(const F& f, Args&&... args)
{
	return external_thread_pool()->enqueue(f, args...);
}

} // namespace cgogn

#endif // CGOGN_CORE_UTILS_THREADPOOL_H_

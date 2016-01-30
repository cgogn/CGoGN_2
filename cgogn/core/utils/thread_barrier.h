/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* version 0.1                                                                  *
* Copyright (C) 2009-2012, IGG Team, LSIIT, University of Strasbourg           *
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

#ifndef CORE_UTILS_THREAD_BARRIER_H_
#define CORE_UTILS_THREAD_BARRIER_H_

#include <thread>
#include <mutex>
#include <condition_variable>

namespace cgogn
{

/**
* Implementation of simple counter barrier (rdv)
* for c++11 std::thread
*/
class Barrier
{
private:

	unsigned int init_count_;
	unsigned int count_;
	unsigned int generation_;
	
	std::mutex protect_;
	std::condition_variable cond_;

public:

	/**
	* constructor
	* @param count number of threads to syncronize
	*/
	inline Barrier(unsigned int count) :
		init_count_(count),
		count_(count),
		generation_(0)
	{}
	
	inline void wait()
	{
		std::unique_lock<std::mutex> lock(protect_);
		unsigned int gen = generation_;

		if (--count_ == 0)
		{
			++generation_;
			count_ = init_count_;
			cond_.notify_all();
		}
		else
			cond_.wait(lock, [this, gen] () { return gen != generation_; });
	}
};

} // namespace cgogn

#endif // CORE_UTILS_THREAD_BARRIER_H_

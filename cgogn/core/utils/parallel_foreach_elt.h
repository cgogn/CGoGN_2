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

#ifndef CGOGN_CORE_UTILS_PARA_FOR_ELT_H_
#define CGOGN_CORE_UTILS_PARA_FOR_ELT_H_

#include <vector>
#include <type_traits>

#include <cgogn/core/utils/thread.h>
#include <cgogn/core/utils/thread_pool.h>
#include <cgogn/core/utils/buffers.h>

namespace cgogn
{

template <typename CONT, typename FUNC>
void parallel_foreach_element(CONT& cont, const FUNC& f)
{
	using iterELt = decltype(cont.begin());
	using T_ELT = decltype(*(cont.begin()));
	using PF  = func_ith_parameter_type<FUNC,0>;
	using PFR = typename std::remove_reference<PF>::type;
	using TP1 = typename std::remove_cv<PFR>::type;
	using TP2 = typename std::remove_cv<typename std::remove_reference<T_ELT>::type>::type;

	static_assert(std::is_same<TP1,TP2>::value &&
				  (((std::is_const<PFR>::value || !std::is_reference<PF>::value ) && std::is_const<CONT>::value)
				   ||(!std::is_const<CONT>::value)),
				  "Wrong function parameter type");


	using VectItELt = std::vector<iterELt>;
	using Future = std::future<typename std::result_of<FUNC(T_ELT)>::type>;

	ThreadPool* thread_pool = cgogn::thread_pool();
	uint32 nb_workers = thread_pool->nb_workers();

	if (nb_workers == 0)
	{
		for (T_ELT e: cont)
			f(e);
		return;
	}

	std::array<std::vector<VectItELt*>, 2> elts_buffers;
	std::array<std::vector<Future>, 2> futures;
	elts_buffers[0].reserve(nb_workers);
	elts_buffers[1].reserve(nb_workers);
	futures[0].reserve(nb_workers);
	futures[1].reserve(nb_workers);

	Buffers<iterELt> buffs;

	iterELt it = cont.begin();
	iterELt last = cont.end();

	uint32 i = 0u; // buffer id (0/1)
	uint32 j = 0u; // thread id (0..nb_workers)
	while (it != last)
	{
		// fill buffer
		elts_buffers[i].push_back(buffs.buffer());
		VectItELt& elts = *elts_buffers[i].back();
		elts.reserve(PARALLEL_BUFFER_SIZE);
		for (unsigned k = 0u; k < PARALLEL_BUFFER_SIZE && it!= last; ++it,++k)
		{
			elts.push_back(it);
		}
		// launch thread
		futures[i].push_back(thread_pool->enqueue([&elts, &f] ()
		{
			for (auto e : elts)
				f(*e);
		}));
		// next thread
		if (++j == nb_workers)
		{	// again from 0 & change buffer
			j = 0;
			i = (i+1u) % 2u;
			for (auto& fu : futures[i])
				fu.wait();
			for (auto& b : elts_buffers[i])
				buffs.release_buffer(b);
			futures[i].clear();
			elts_buffers[i].clear();
		}
	}

	// clean all at end
	for (auto& fu : futures[0u])
		fu.wait();
	for (auto& b : elts_buffers[0u])
		buffs.release_buffer(b);
	for (auto& fu : futures[1u])
		fu.wait();
	for (auto& b : elts_buffers[1u])
		buffs.release_buffer(b);
}






template <typename CONT_A, typename CONT_B, typename FUNC>
void parallel_foreach_element(CONT_A& cont_a, CONT_B& cont_b, const FUNC& f)
{
	using iterELtA = decltype(cont_a.begin());
	using iterELtB = decltype(cont_b.begin());
	using T_ELT_A = decltype(*(cont_a.begin()));
	using T_ELT_B = decltype(*(cont_b.begin()));
	using PFA  = func_ith_parameter_type<FUNC,0>;
	using PFRA = typename std::remove_reference<PFA>::type;
	using TPA1 = typename std::remove_cv<PFRA>::type;
	using TPA2 = typename std::remove_cv<typename std::remove_reference<T_ELT_A>::type>::type;

	static_assert(std::is_same<TPA1,TPA2>::value &&
				  (((std::is_const<PFRA>::value || !std::is_reference<PFA>::value) && std::is_const<CONT_A>::value)||(!std::is_const<CONT_A>::value)),
				  "Wrong function first parameter type");

	using PFB  = func_ith_parameter_type<FUNC,1>;
	using PFRB = typename std::remove_reference<PFB>::type;
	using TPB1 = typename std::remove_cv<PFRB>::type;
	using TPB2 = typename std::remove_cv<typename std::remove_reference<T_ELT_B>::type>::type;

	static_assert(std::is_same<TPB1,TPB2>::value &&
				  (((std::is_const<PFRB>::value || !std::is_reference<PFB>::value) && std::is_const<CONT_B>::value)||(!std::is_const<CONT_B>::value)),
				  "Wrong function second parameter type");


	using iterPair = std::pair<iterELtA,iterELtB>;

	using VectItELt = std::vector<iterPair>;
	using Future = std::future<typename std::result_of<FUNC(T_ELT_A,T_ELT_B)>::type>;

	ThreadPool* thread_pool = cgogn::thread_pool();
	uint32 nb_workers = thread_pool->nb_workers();

	if (nb_workers == 0)
	{
		auto it=cont_a.begin();
		auto jt=cont_b.begin();
		auto end_a=cont_a.end();
		auto end_b=cont_b.end();
		while (it != end_a && jt != end_b)
		{
			f(*it,*jt);
			it++;
			jt++;
		}
		return;
	}

	std::array<std::vector<VectItELt*>, 2> elts_buffers;
	std::array<std::vector<Future>, 2> futures;
	elts_buffers[0].reserve(nb_workers);
	elts_buffers[1].reserve(nb_workers);
	futures[0].reserve(nb_workers);
	futures[1].reserve(nb_workers);

	Buffers<iterPair> buffs;

	iterELtA it = cont_a.begin();
	iterELtA last = cont_a.end();

	iterELtB it2 = cont_b.begin();
	iterELtB last2 = cont_b.end();

	uint32 i = 0u; // buffer id (0/1)
	uint32 j = 0u; // thread id (0..nb_workers)
	while (it != last && it2 != last2)
	{
		// fill buffer
		elts_buffers[i].push_back(buffs.buffer());
		VectItELt& elts = *elts_buffers[i].back();
		elts.reserve(PARALLEL_BUFFER_SIZE);
		for (unsigned k = 0u; k < PARALLEL_BUFFER_SIZE && it!= last && it2 != last2; ++it,++it2,++k)
		{
			elts.push_back(std::make_pair(it,it2));
		}
		// launch thread
		futures[i].push_back(thread_pool->enqueue([&elts, &f] ()
		{
			for (auto e : elts)
				f(*(e.first),*(e.second));
		}));
		// next thread
		if (++j == nb_workers)
		{	// again from 0 & change buffer
			j = 0;
			i = (i+1u) % 2u;
			for (auto& fu : futures[i])
				fu.wait();
			for (auto& b : elts_buffers[i])
				buffs.release_buffer(b);
			futures[i].clear();
			elts_buffers[i].clear();
		}
	}

	// clean all at end
	for (auto& fu : futures[0u])
		fu.wait();
	for (auto& b : elts_buffers[0u])
		buffs.release_buffer(b);
	for (auto& fu : futures[1u])
		fu.wait();
	for (auto& b : elts_buffers[1u])
		buffs.release_buffer(b);
}


} // namespace cgogn

#endif // CGOGN_CORE_UTILS_BUFFERS_H_

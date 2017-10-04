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
#include <utility>


#include <cgogn/core/utils/thread.h>
#include <cgogn/core/utils/thread_pool.h>
#include <cgogn/core/utils/buffers.h>

namespace cgogn
{
/**
 * @brief apply f function on each element of a container in parallel
 * @param cont container
 * @param f function with 1 params (contents of the container)
 */
template <typename CONT, typename FUNC>
void parallel_foreach_element(CONT& cont, const FUNC& f)
{
	using IterElt = decltype(cont.begin());
	using T_ELT = decltype(*(cont.begin()));
	using PF  = func_ith_parameter_type<FUNC,0>;
	using PFR = typename std::remove_reference<PF>::type;
	using TP1 = typename std::remove_cv<PFR>::type;
	using TP2 = typename std::remove_cv<typename std::remove_reference<T_ELT>::type>::type;

	static_assert(std::is_same<TP1,TP2>::value &&
				  (((std::is_const<PFR>::value || !std::is_reference<PF>::value ) && std::is_const<CONT>::value)
				   ||(!std::is_const<CONT>::value)),
				  "Wrong function parameter type");


	using VectItELt = std::vector<IterElt>;
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

	Buffers<IterElt> buffs;

	IterElt it = cont.begin();
	IterElt last = cont.end();

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



namespace internal
{
	template <typename FUNC, int ITH, typename CONT, typename T_ELT >
	struct pfe_check_func_param
	{
		using PF = func_ith_parameter_type<FUNC, ITH>;
		using PFR = typename std::remove_reference<PF>::type;
		using TP1 = typename std::remove_cv<PFR>::type;
		using TP2 = typename std::remove_cv<typename std::remove_reference<T_ELT>::type>::type;

		constexpr static bool value = std::is_same<TP1, TP2>::value &&
			(((std::is_const<PFR>::value || !std::is_reference<PF>::value) && std::is_const<CONT>::value) || (!std::is_const<CONT>::value));

		static void print()
		{
			std::cout << std::boolalpha << std::is_const<PFR>::value << " / " << std::is_reference<PF>::value << " / " << std::is_const<CONT>::value << std::endl;
			std::cout << typeid(CONT).name() << std::endl;

		}
	};


	template < typename PFP, typename FUNC>
	void parallel_foreach_elements(PFP p, const FUNC& f)
	{
		using Iterators = typename PFP::Iterators;

		using Future = std::future<void>;
		using VectItELt = std::vector<Iterators>;

		ThreadPool* thread_pool = cgogn::thread_pool();
		uint32 nb_workers = thread_pool->nb_workers();

		// mono-thread case
		if (nb_workers == 0)
		{
			Iterators its = p.begin();
			Iterators ends = p.end();
			while (PFP::diff(its, ends))
			{
				PFP::call(f, its);
				p.next(its);
			}
			return;
		}

		std::array<std::vector<VectItELt*>, 2> elts_buffers;
		std::array<std::vector<Future>, 2> futures;
		elts_buffers[0].reserve(nb_workers);
		elts_buffers[1].reserve(nb_workers);
		futures[0].reserve(nb_workers);
		futures[1].reserve(nb_workers);

		Buffers<Iterators> buffs;
		Iterators its = p.begin();
		Iterators ends = p.end();
		uint32 i = 0u; // buffer id (0/1)
		uint32 j = 0u; // thread id (0..nb_workers)

		while (PFP::diff(its, ends))
		{
			// fill buffer
			elts_buffers[i].push_back(buffs.buffer());
			VectItELt& elts = *elts_buffers[i].back();
			elts.reserve(PARALLEL_BUFFER_SIZE);
			for (unsigned k = 0u; k < PARALLEL_BUFFER_SIZE && PFP::diff(its, ends); p.next(its), ++k)
			{
				elts.push_back(its);
			}
			// launch thread
			futures[i].push_back(thread_pool->enqueue([&elts, &f]()
			{
				for (auto e : elts)
					PFP::call(f, e);
			}));
			// next thread
			if (++j == nb_workers)
			{	// again from 0 & change buffer
				j = 0;
				i = (i + 1u) % 2u;
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

}

#define CONT(I) std::get<I>(c_)

/**
 * @brief apply f function on each element of 2 containers in parallel
 * @param a container
 * @param b container
 * @param f function with 2 params (contents of each container)
 */
template < typename A, typename B, typename FUNC>
void parallel_foreach_element(A& a, B& b, const FUNC& f)
{
	struct pfe_param
	{
		std::tuple<A*, B*> c_;

		using Iterators = std::tuple<
			decltype(CONT(0)->begin()),
			decltype(CONT(1)->begin())>;

		inline void next(Iterators& its)
		{
			++(std::get<0>(its));
			++(std::get<1>(its));
		}

		static inline bool diff(const Iterators& its, const Iterators& jts)
		{
			return
				(std::get<1>(its) != std::get<1>(jts)) &&
				(std::get<0>(its) != std::get<0>(jts));
		}

		Iterators begin()
		{
			return std::make_tuple(CONT(0)->begin(), CONT(1)->begin());
		}

		Iterators end()
		{
			return std::make_tuple(CONT(0)->end(), CONT(1)->end());
		}

		static void call(const FUNC& f, const Iterators& its)
		{
			static_assert(internal::pfe_check_func_param < FUNC, 0, A, decltype(*(CONT(0)->begin()))>::value, "wrong first param type");
			static_assert(internal::pfe_check_func_param < FUNC, 1, B, decltype(*(CONT(1)->begin()))>::value, "wrong second param type");
			f(*(std::get<0>(its)), *(std::get<1>(its)));
		}
	};


	pfe_param p = { std::make_tuple(&a, &b) };
	internal::parallel_foreach_elements(p, f);
}

/**
 * @brief apply f function on each element of 3 containers in parallel
 * @param a container
 * @param b container
 * @param c container
 * @param f function with 3 params (contents of each container)
 */
template < typename A, typename B, typename C, typename FUNC>
void parallel_foreach_element(A& a, B& b, C& c, const FUNC& f)
{
	struct pfe_param
	{
		std::tuple<A*, B*, C*> c_;

		using Iterators = std::tuple<
			decltype(CONT(0)->begin()),
			decltype(CONT(1)->begin()),
			decltype(CONT(2)->begin())>;

		inline void next(Iterators& its)
		{
			++(std::get<0>(its));
			++(std::get<1>(its));
			++(std::get<2>(its));
		}

		static inline bool diff(const Iterators& its, const Iterators& jts)
		{
			return
				(std::get<2>(its) != std::get<2>(jts)) &&
				(std::get<1>(its) != std::get<1>(jts)) &&
				(std::get<0>(its) != std::get<0>(jts));
		}

		Iterators begin()
		{
			return std::make_tuple(CONT(0)->begin(), CONT(1)->begin(), CONT(2)->begin());
		}

		Iterators end()
		{
			return std::make_tuple(CONT(0)->end(), CONT(1)->end(), CONT(2)->end());
		}

		static void call(const FUNC& f, const Iterators& its)
		{
			static_assert(internal::pfe_check_func_param < FUNC, 0, A, decltype(*(CONT(0)->begin()))>::value, "wrong first param type");
			static_assert(internal::pfe_check_func_param < FUNC, 1, B, decltype(*(CONT(1)->begin()))>::value, "wrong second param type");
			static_assert(internal::pfe_check_func_param < FUNC, 2, C, decltype(*(CONT(2)->begin()))>::value, "wrong third param type");
			f(*(std::get<0>(its)), *(std::get<1>(its)), *(std::get<2>(its)));
		}
	};
	
	pfe_param p = { std::make_tuple(&a, &b, &c) };
	internal::parallel_foreach_elements(p, f);
}



/**
 * @brief apply f function on each element of 4 containers in parallel
 * @param a container
 * @param b container
 * @param c container
 * @param d container
 * @param f function with 4 params (contents of each container)
 */
template < typename A, typename B, typename C, typename D, typename FUNC>
void parallel_foreach_element(A& a, B& b, C& c, D& d, const FUNC& f)
{
	struct pfe_param
	{
		std::tuple<A*, B*, C* ,D*> c_;

		using Iterators = std::tuple<
			decltype(CONT(0)->begin()),
			decltype(CONT(1)->begin()),
			decltype(CONT(2)->begin()),
			decltype(CONT(3)->begin())>;

		inline void next(Iterators& its)
		{
			++(std::get<0>(its));
			++(std::get<1>(its));
			++(std::get<2>(its));
			++(std::get<3>(its));
		}

		static inline bool diff(const Iterators& its, const Iterators& jts)
		{
			return
				(std::get<3>(its) != std::get<3>(jts)) &&
				(std::get<2>(its) != std::get<2>(jts)) &&
				(std::get<1>(its) != std::get<1>(jts)) &&
				(std::get<0>(its) != std::get<0>(jts));
		}

		Iterators begin()
		{
			return std::make_tuple(CONT(0)->begin(), CONT(1)->begin(), CONT(2)->begin(), CONT(3)->begin());
		}

		Iterators end()
		{
			return std::make_tuple(CONT(0)->end(), CONT(1)->end(), CONT(2)->end(), CONT(3)->end());
		}

		static void call(const FUNC& f, const Iterators& its)
		{
			static_assert(internal::pfe_check_func_param < FUNC, 0, A, decltype(*(CONT(0)->begin()))>::value, "wrong first param type");
			static_assert(internal::pfe_check_func_param < FUNC, 1, B, decltype(*(CONT(1)->begin()))>::value, "wrong second param type");
			static_assert(internal::pfe_check_func_param < FUNC, 2, C, decltype(*(CONT(2)->begin()))>::value, "wrong third param type");
			static_assert(internal::pfe_check_func_param < FUNC, 3, D, decltype(*(CONT(3)->begin()))>::value, "wrong fourth param type");
			f(*(std::get<0>(its)), *(std::get<1>(its)), *(std::get<2>(its)), *(std::get<3>(its)));
		}
	};

	pfe_param p = { std::make_tuple(&a, &b, &c, &d) };
	internal::parallel_foreach_elements(p, f);
}
#undef CONT


} // namespace cgogn

#endif // CGOGN_CORE_UTILS_BUFFERS_H_

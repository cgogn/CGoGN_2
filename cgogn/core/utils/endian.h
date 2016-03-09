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

#ifndef CORE_UTILS_ENDIAN_H_
#define CORE_UTILS_ENDIAN_H_

#include <cstdint>
#include <type_traits>
#include <core/utils/definitions.h>

namespace cgogn
{

namespace internal
{

#if CGOGN_ENDIANNESS == CGOGN_BIG_ENDIAN
const bool cgogn_is_big_endian = true;
const bool cgogn_is_little_endian = false;
#else
const bool cgogn_is_big_endian = false;
const bool cgogn_is_little_endian = true;
#endif

inline std::uint16_t swap_endianness16(std::uint16_t x)
{
	return	((x >> 8) & 0x00FF) | ((x << 8) & 0xFF00);
}

inline std::uint32_t swap_endianness32(std::uint32_t x)
{
	return	((x >> 24) & 0x000000FF) | ((x >>  8) & 0x0000FF00) |
			((x <<  8) & 0x00FF0000) | ((x << 24) & 0xFF000000);
}

inline std::uint64_t swap_endianness64(std::uint64_t x)
{
	return	((x >> 56) & 0x00000000000000FF) | ((x >> 40) & 0x000000000000FF00) |
			((x >> 24) & 0x0000000000FF0000) | ((x >>  8) & 0x00000000FF000000) |
			((x <<  8) & 0x000000FF00000000) | ((x << 24) & 0x0000FF0000000000) |
			((x << 40) & 0x00FF000000000000) | ((x << 56) & 0xFF00000000000000);
}

template<typename UINT, bool COND>
inline UINT swap_endianness_if(UINT x)
{
	static_assert(std::is_same<UINT, std::uint16_t>::value ||
				  std::is_same<UINT, std::uint32_t>::value ||
				  std::is_same<UINT, std::uint64_t>::value, "This function is specialized for 16, 32 or 64 bits uints.");

	if (COND)
	{
		if (std::is_same<UINT, std::uint16_t>::value)
			return UINT(swap_endianness16(std::uint16_t(x)));
		if (std::is_same<UINT, std::uint32_t>::value)
			return UINT(swap_endianness32(std::uint32_t(x)));
		if (std::is_same<UINT, std::uint64_t>::value)
			return UINT(swap_endianness64(std::uint64_t(x)));
	}
	return x;
}

} // namespace internal


template<typename UINT>
inline UINT swap_endianness_system_big(UINT x)
{
	return internal::swap_endianness_if<UINT, internal::cgogn_is_little_endian>(x);
}

template<typename UINT>
inline UINT swap_endianness_system_little(UINT x)
{
	return internal::swap_endianness_if<UINT, internal::cgogn_is_big_endian>(x);
}



} // namespace cgogn

#endif // CORE_UTILS_ENDIAN_H_

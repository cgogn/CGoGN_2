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

inline std::uint16_t swap_endianness16u(std::uint16_t x)
{
	return	((x >> 8) & 0x00FF) | ((x << 8) & 0xFF00);
}

inline std::uint32_t swap_endianness32u(std::uint32_t x)
{
	return	((x >> 24) & 0x000000FF) | ((x >>  8) & 0x0000FF00) |
			((x <<  8) & 0x00FF0000) | ((x << 24) & 0xFF000000);
}

inline std::uint64_t swap_endianness64u(std::uint64_t x)
{
	return	((x >> 56) & 0x00000000000000FF) | ((x >> 40) & 0x000000000000FF00) |
			((x >> 24) & 0x0000000000FF0000) | ((x >>  8) & 0x00000000FF000000) |
			((x <<  8) & 0x000000FF00000000) | ((x << 24) & 0x0000FF0000000000) |
			((x << 40) & 0x00FF000000000000) | ((x << 56) & 0xFF00000000000000);
}

inline std::int16_t swap_endianness16(std::int16_t x)
{
// boost/endian/conversion.hpp
	return	(static_cast<std::uint16_t>(x) << 8) |
			(static_cast<std::uint16_t>(x) >> 8);
}

inline std::int32_t swap_endianness32(std::int32_t x)
{
// boost/endian/conversion.hpp
	using std::uint32_t;

	uint32_t step16;
	step16 = static_cast<uint32_t>(x) << 16 | static_cast<uint32_t>(x) >> 16;
	return	((static_cast<uint32_t>(step16) << 8) & 0xff00ff00) |
			((static_cast<uint32_t>(step16) >> 8) & 0x00ff00ff);
}

inline std::int64_t swap_endianness64(std::int64_t x)
{
// boost/endian/conversion.hpp
	using std::uint64_t;
	using std::int64_t;

	uint64_t step32, step16;
	step32 = static_cast<uint64_t>(x) << 32 | static_cast<uint64_t>(x) >> 32;
	step16 =	(step32 & 0x0000FFFF0000FFFFULL) << 16 |
				(step32 & 0xFFFF0000FFFF0000ULL) >> 16;
	return static_cast<int64_t>(	(step16 & 0x00FF00FF00FF00FFULL) << 8 |
									(step16 & 0xFF00FF00FF00FF00ULL) >> 8);
}

inline float swap_endianness_float(float x)
{
	union U32F32
	{
		std::uint32_t	as_u32;
		float			as_f32;
	} u;
	u.as_f32 = x;
	u.as_u32 = swap_endianness32u(u.as_u32);
	return u.as_f32;
}

inline double swap_endianness_double(double x)
{
	union U64F64
	{
		std::uint64_t	as_u64;
		double			as_f64;
	} u;
	u.as_f64 = x;
	u.as_u64 = swap_endianness64u(u.as_u64);
	return u.as_f64;
}

template<typename T, bool COND>
inline T swap_endianness_if(T x)
{
	static_assert(	std::is_same<T, char>::value ||
					std::is_same<T, std::uint8_t>::value ||
					std::is_same<T, std::uint16_t>::value ||
					std::is_same<T, std::uint32_t>::value ||
					std::is_same<T, std::uint64_t>::value ||
					std::is_same<T, std::int8_t>::value ||
					std::is_same<T, std::int16_t>::value ||
					std::is_same<T, std::int32_t>::value ||
					std::is_same<T, std::int64_t>::value ||
					std::is_same<T, float>::value ||
					std::is_same<T, double>::value, "This function is specialized for 8,16, 32 or 64 bits (u))ints, floats and doubles.");

	if (COND)
	{
		if (std::is_same<T, std::uint16_t>::value)
			return swap_endianness16u(x);
		if (std::is_same<T, std::uint32_t>::value)
			return swap_endianness32u(x);
		if (std::is_same<T, std::uint64_t>::value)
			return swap_endianness64u(x);
		if (std::is_same<T, std::int16_t>::value)
			return swap_endianness16(x);
		if (std::is_same<T, std::int32_t>::value)
			return swap_endianness32(x);
		if (std::is_same<T, std::int64_t>::value)
			return swap_endianness64(x);
		if (std::is_same<T, float>::value)
			return swap_endianness_float(x);
		if (std::is_same<T, double>::value)
			return swap_endianness_double(x);
	}
	return x;
}

} // namespace internal

template<typename T>
inline T swap_endianness(T x)
{
	return internal::swap_endianness_if<T, true>(x);
}

template<typename T>
inline T swap_endianness_native_big(T x)
{
	return internal::swap_endianness_if<T, internal::cgogn_is_little_endian>(x);
}

template<typename T>
inline T swap_endianness_native_little(T x)
{
	return internal::swap_endianness_if<T, internal::cgogn_is_big_endian>(x);
}

} // namespace cgogn

#endif // CORE_UTILS_ENDIAN_H_

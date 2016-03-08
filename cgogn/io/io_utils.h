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

#ifndef IO_IO_UTILS_H_
#define IO_IO_UTILS_H_

#include <type_traits>
#include <sstream>

#include <core/utils/endian.h>
#include <geometry/types/geometry_traits.h>
#include <io/dll.h>

namespace cgogn
{

namespace io
{

enum FileType
{
	FileType_UNKNOWN = 0,
	FileType_OFF,
	FileType_OBJ,
	FileType_PLY,
	FileType_VTK_LEGACY,
	FileType_VTU
};

enum DataType
{
	CHAR = 0,
	INT8,
	UINT8,
	INT16,
	UINT16,
	INT32,
	UINT32,
	INT64,
	UINT64,
	FLOAT,
	DOUBLE,
	UNKNOWN
};
CGOGN_IO_API FileType get_file_type(const std::string& filename);
CGOGN_IO_API DataType get_data_type(const std::string& type_name);
CGOGN_IO_API std::vector<unsigned char> base64_decode(const std::string& input, std::size_t begin, std::size_t length);

#ifdef CGOGN_WITH_ZLIB
CGOGN_IO_API std::vector<unsigned char> zlib_decompress(const std::string& input, DataType header_type);
#endif

namespace internal
{

// #1 return default value when U and T don't have the same nb of components.
template<typename U, typename T>
inline auto convert(const T&) -> typename std::enable_if<!std::is_same< std::integral_constant<unsigned int, geometry::nb_components_traits<T>::value>, std::integral_constant<unsigned int, geometry::nb_components_traits<U>::value>>::value,U>::type
{
	std::cerr << "Cannot convert data of type\"" << name_of_type(T()) << "\" to type \"" << name_of_type(U()) << "\"." << std::endl;
	return U();
}

// #2 cast x if both types have only one component.
template<typename U, typename T>
inline auto convert(const T&x) -> typename std::enable_if<(std::is_arithmetic<T>::value || std::is_floating_point<T>::value) && (std::is_arithmetic<U>::value || std::is_floating_point<U>::value),U>::type
{
	return U(x);
}

// #3 copy component by component if both type have the same number of components (>1)
template<typename U, typename T>
inline auto convert(const T& x) -> typename std::enable_if<!std::is_arithmetic<T>::value && !std::is_floating_point<T>::value && std::is_same< std::integral_constant<unsigned int, geometry::nb_components_traits<T>::value>, std::integral_constant<unsigned int, geometry::nb_components_traits<U>::value>>::value, U>::type
{
	U res;
	for(unsigned int i = 0u; i < geometry::nb_components_traits<T>::value ; ++i)
		res[i] = typename geometry::vector_traits<U>::Scalar(x[i]);
	return res;
}


template<typename T>
inline typename std::enable_if<std::is_arithmetic<T>::value || std::is_floating_point<T>::value, T>::type swap_endianness(const T& x)
{
	return ::cgogn::swap_endianness(x);
}

template<typename T>
inline typename std::enable_if<(!std::is_arithmetic<T>::value) && !std::is_floating_point<T>::value, T>::type swap_endianness(T& x)
{
	for (std::size_t i = 0u ; i < geometry::vector_traits<T>::SIZE; ++i)
		x[i] = ::cgogn::swap_endianness(x[i]);
	return x;
}

template<typename T>
inline typename std::enable_if<std::is_arithmetic<T>::value || std::is_floating_point<T>::value, std::istream&>::type parse(std::istream& iss, T& x)
{
	iss >> x;
	return iss;
}

template<typename T>
inline typename std::enable_if<!std::is_arithmetic<T>::value && !std::is_floating_point<T>::value, std::istream&>::type parse(std::istream& iss, T& x)
{
	for (std::size_t i = 0u ; i < geometry::vector_traits<T>::SIZE; ++i)
		iss >> x[i];
	return iss;
}

} // namespace internal
} // namespace io
} // namespace cgogn

#endif // IO_IO_UTILS_H_

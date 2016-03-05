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

inline FileType get_file_type(const std::string& filename)
{
	const std::string& extension = to_lower(get_extension(filename));
	if (extension == "off")
		return FileType::FileType_OFF;
	if (extension == "obj")
		return FileType::FileType_OBJ;
	if (extension == "ply")
		return FileType::FileType_PLY;
	if (extension == "vtk")
		return FileType::FileType_VTK_LEGACY;
	if (extension == "vtu")
		return FileType::FileType_VTU;
	return FileType::FileType_UNKNOWN;
}

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

inline DataType get_data_type(const std::string& type_name)
{
	if (type_name == name_of_type(float()))
		return DataType::FLOAT;
	else if (type_name == name_of_type(double()))
		return DataType::DOUBLE;
	else if (type_name == name_of_type(char()))
		return DataType::CHAR;
	else if (type_name == name_of_type(std::int8_t()))
		return DataType::INT8;
	else if (type_name == name_of_type(std::uint8_t()))
		return DataType::UINT8;
	else if (type_name == name_of_type(std::int16_t()))
		return DataType::INT16;
	else if (type_name == name_of_type(std::uint16_t()))
		return DataType::UINT16;
	else if (type_name == name_of_type(std::int32_t()))
		return DataType::INT32;
	else if (type_name == name_of_type(std::uint32_t()))
		return DataType::UINT32;
	else if (type_name == name_of_type(std::int64_t()))
		return DataType::INT64;
	else if (type_name == name_of_type(std::uint64_t()))
		return DataType::UINT64;

	return DataType::UNKNOWN;
}

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

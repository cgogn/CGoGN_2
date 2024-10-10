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

#ifndef CGOGN_CORE_UTILS_SERIALIZATION_H_
#define CGOGN_CORE_UTILS_SERIALIZATION_H_

#include <iostream>
#include <vector>
#include <list>
#include <array>

#include <cgogn/core/utils/assert.h>
#include <cgogn/core/utils/endian.h>
#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/utils/type_traits.h>
#include <cgogn/core/cgogn_core_export.h>

namespace cgogn
{

namespace internal
{
/**
 * Here we can find the declarations of several helper functions and classes. Their definition can be found at the end of the file
 */

template<typename T>
inline typename std::enable_if<!has_size_method<T>::value && !is_iterable<T>::value, std::size_t>::type size_helper(const T&);
template<typename T>
inline typename std::enable_if<has_size_method<T>::value, std::size_t>::type size_helper(const T& x);

template <typename T>
inline typename std::enable_if<(!has_size_method<T>::value && !is_iterable<T>::value) || std::is_same<T,std::string>::value, void>::type parse_helper(std::istream& iss, T& x);
template <typename T>
inline typename std::enable_if<(is_iterable<T>::value || has_size_method<T>::value) && !std::is_same<T,std::string>::value, void>::type parse_helper(std::istream& iss, T& x);

template <typename T>
inline typename std::enable_if<std::is_arithmetic<typename std::remove_reference<T>::type>::value, void>::type serialize_binary_helper(std::ostream& o, T&& x, bool little_endian);
template <typename T>
inline typename std::enable_if<has_cgogn_binary_serialize<T>::value, void>::type serialize_binary_helper(std::ostream& o, T&& x, bool little_endian);
template <typename T>
inline typename std::enable_if<!std::is_arithmetic<typename std::remove_reference<T>::type>::value && !has_cgogn_binary_serialize<T>::value, void>::type serialize_binary_helper(std::ostream& o, T&& x, bool little_endian);

template <typename T, std::size_t Precision>
inline typename std::enable_if<std::is_arithmetic<typename std::remove_reference<T>::type>::value || (!is_iterable<T>::value && !has_size_method<T>::value), void>::type ostream_writer_helper(std::ostream& o, const T& x, bool binary, bool little_endian);
template <typename T, std::size_t Precision>
inline typename std::enable_if<is_iterable<T>::value || (has_size_method<T>::value && has_operator_brackets<T>::value) || has_rows_method<T>::value, void>::type ostream_writer_helper(std::ostream& o, const T& x, bool binary, bool little_endian);

} // namespace internal


namespace serialization
{

/**
 * @brief serialize_binary function, serialize the data contained in x in o in binary mode (and swapping endianness if little_endian != internal::cgogn_is_little_endian)
 * @param o, the output ostream
 * @param x, the data to be serialized
 * @param little_endian, true iff the data has to be written in little_endian order.
 * @warning This function need to be specialized for any user-defined type.
 */
template <typename T>
inline void serialize_binary(std::ostream& o, T&& x, bool little_endian)
{
	internal::serialize_binary_helper(o,std::forward<T>(x),little_endian);
}


/**
 * @brief size function
 * @param data
 * @return return data.size() if data has a size method, 1 otherwise
 */
template<typename T>
inline std::size_t size(const T& data)
{
	return internal::size_helper(data);
}

/**
 * @brief parse, extract data x from the istream iss
 * @param iss
 * @param x
 */
template<typename T>
inline void parse(std::istream& iss, T& x)
{
	internal::parse_helper(iss,x);
}

/**
 * @brief ostream_writer, write data x to the ostream o
 * @param o, the destination ostream
 * @param x, the data
 * @param binary, true iff writing in binary mode
 * @param little_endian, if writing in binary mode, true iff writing data in little endian order.
 */
template <typename T, std::size_t Precision = 8ul>
inline void ostream_writer(std::ostream& o, const T& x, bool binary = false, bool little_endian = internal::cgogn_is_little_endian)
{
	internal::ostream_writer_helper<T,Precision>(o,x,binary,little_endian);
}




template <typename T>
void load(std::istream& istream, T* dest, std::size_t quantity)
{
	cgogn_assert(dest != nullptr);
	istream.read(reinterpret_cast<char*>(dest), static_cast<std::streamsize>(quantity*sizeof(T)));
}

template <typename T>
void save(std::ostream& ostream, T const* src, std::size_t quantity)
{
	cgogn_assert(src != nullptr);
	ostream.write(reinterpret_cast<const char*>(src), static_cast<std::streamsize>(quantity*sizeof(T)));
}

template <typename T>
std::size_t data_length(T const* /*src*/, std::size_t quantity)
{
	return quantity*sizeof(T);
}

// data size is known or not ?
template <typename T>
bool known_size(T const* /*src*/)
{
	return true;
}

template <>
CGOGN_CORE_EXPORT bool known_size<std::string>(std::string const* /*src*/);

template <typename U>
bool known_size(std::vector<U> const* /*src*/)
{
	return false;
}

template <typename U>
bool known_size(std::list<U> const* /*src*/)
{
	return false;
}

template <typename U, std::size_t size>
bool known_size(std::array<U, size> const* /*src*/)
{
	return false;
}

// first step : declare all overrides of load and save
template <typename U>
void load(std::istream& istream, std::vector<U>* dest, std::size_t quantity);

template <typename U>
void save(std::ostream& ostream, std::vector<U> const* src, std::size_t quantity);

template <typename U>
std::size_t data_length(std::vector<U> const* src, std::size_t quantity);


template <typename U>
void load(std::istream& istream, std::list<U>* dest, std::size_t quantity);

template <typename U>
void save(std::ostream& ostream, std::list<U> const* src, std::size_t quantity);

template <typename U>
std::size_t data_length(std::list<U> const* src, std::size_t quantity);


template <typename U, std::size_t size>
void load(std::istream& istream, std::array<U, size>* dest, std::size_t quantity);

template <typename U, std::size_t size>
void save(std::ostream& ostream, std::array<U, size> const* src, std::size_t quantity);

template <typename U, std::size_t size>
std::size_t data_length(std::array<U, size>const* src, std::size_t quantity);


template <>
CGOGN_CORE_EXPORT void load<std::string>(std::istream& istream, std::string* dest, std::size_t quantity);

template <>
CGOGN_CORE_EXPORT void save<std::string>(std::ostream& ostream, std::string const* src, std::size_t quantity);

template <>
CGOGN_CORE_EXPORT std::size_t data_length<std::string>(std::string const* src, std::size_t quantity);


// loading n vectors
template <typename U>
void load(std::istream& istream, std::vector<U>* dest, std::size_t quantity)
{
	cgogn_assert(istream.good());
	cgogn_assert(dest != nullptr);

	for (std::size_t i = 0u; i < quantity; ++i)
	{
		uint32 vecSize;
		istream.read(reinterpret_cast<char*>(&vecSize), sizeof(uint32));
		dest[i].resize(vecSize);
		load(istream, &(dest[i][0]), vecSize);
	}
}

// saving n vectors
template <typename U>
void save(std::ostream& ostream, std::vector<U> const* src, std::size_t quantity)
{
	cgogn_assert(ostream.good());
	cgogn_assert(src != nullptr);

	for (std::size_t i = 0u; i < quantity; ++i)
	{
		const uint32 size = uint32(src[i].size());
		ostream.write(reinterpret_cast<const char *>(&size), sizeof(uint32));
		save(ostream, &(src[i][0]), size);
	}
}

// compute data length of vector
template <typename U>
std::size_t data_length(std::vector<U> const * src, std::size_t quantity)
{
	cgogn_assert(src != nullptr);

	std::size_t total = 0u;
	for (std::size_t i = 0u; i < quantity; ++i)
	{
		total += sizeof(uint32);// for size
		total += data_length(&(src[i][0]), src[i].size());
	}
	return total;
}


// loading n lists
template <typename U>
void load(std::istream& istream, std::list<U>* dest, std::size_t quantity)
{
	cgogn_assert(istream.good());
	cgogn_assert(dest != nullptr);

	for (std::size_t i = 0u; i < quantity; ++i)
	{
		uint32 listSize;
		istream.read(reinterpret_cast<char*>(&listSize), sizeof(uint32));
		std::vector<U> temp;
		temp.resize(listSize);
		load(istream, &(temp[0]), listSize);
		for(auto&& x : temp)
			dest[i].emplace_back(std::move(x));
	}
}

// saving n lists
template <typename U>
void save(std::ostream& ostream, std::list<U> const* src, std::size_t quantity)
{
	cgogn_assert(ostream.good());
	cgogn_assert(src != nullptr);

	for (std::size_t i = 0u; i < quantity; ++i)
	{
		const uint32 size = uint32(src[i].size());
		ostream.write(reinterpret_cast<const char *>(&size), sizeof(uint32));
		for (const auto& elem : src[i])
			save(ostream, &elem, 1);
	}
}

// compute data length of list
template <typename U>
std::size_t data_length(std::list<U> const* src, std::size_t quantity)
{
	cgogn_assert(src != nullptr);

	std::size_t total = 0u;
	for (std::size_t i = 0u; i < quantity; ++i)
	{
		total += sizeof(uint32); // for size
		for (const auto& elem : src[i])
			total += data_length(&elem, 1);
	}
	return total;
}


template <typename U, std::size_t size>
void load(std::istream& istream, std::array<U, size>* dest, std::size_t quantity)
{
	cgogn_assert(istream.good());
	cgogn_assert(dest != nullptr);

	for (std::size_t i = 0u; i < quantity; ++i)
		load(istream, &(dest[i][0]), size);
}

template <typename U, std::size_t size>
void save(std::ostream& ostream, std::array<U, size> const* src, std::size_t quantity)
{
	cgogn_assert(ostream.good());
	cgogn_assert(src);

	for (std::size_t i = 0u; i < quantity; ++i)
		save(ostream, &(src[i][0]), size);
}

template <typename U, std::size_t size>
std::size_t data_length(std::array<U, size>const* src, std::size_t quantity)
{
	cgogn_assert(src != nullptr);
	std::size_t total = 0u;
	for (std::size_t i = 0u; i < quantity; ++i)
	{
		for (const auto& elem : src[i])
			total += data_length(&elem, 1);
	}
	return total;
}

} // namespace serialization

namespace internal
{

template <typename T>
inline typename std::enable_if<std::is_arithmetic<typename std::remove_reference<T>::type>::value, void>::type serialize_binary_helper(std::ostream& o, T&& x, bool little_endian)
{
	if (little_endian != internal::cgogn_is_little_endian)
		x = swap_endianness(x);
	o.write(reinterpret_cast<const char*>(&x), static_cast<std::streamsize>(sizeof(T)));
}

template <typename T>
inline typename std::enable_if<has_cgogn_binary_serialize<T>::value, void>::type serialize_binary_helper(std::ostream& o, T&& x, bool little_endian)
{
	x.cgogn_binary_serialize(o,little_endian);
}

template <typename T>
inline typename std::enable_if<!std::is_arithmetic<typename std::remove_reference<T>::type>::value && !has_cgogn_binary_serialize<T>::value, void>::type serialize_binary_helper(std::ostream& o, T&& x, bool little_endian)
{
	unused_parameters(o,x,little_endian);
	cgogn_assert_not_reached("Error : serialize_binary_helper function called with a non-arithmetic type. You need to specialize the cgogn::serialization::serialize_binary function for your type.");
}




template<typename T>
inline typename std::enable_if<!has_size_method<T>::value && !is_iterable<T>::value, std::size_t>::type size_helper(const T&)
{
	return 1;
}

template<typename T>
inline typename std::enable_if<has_size_method<T>::value, std::size_t>::type size_helper(const T& x)
{
	return x.size();
}



template <typename T>
inline typename std::enable_if<(!has_size_method<T>::value && !is_iterable<T>::value) || std::is_same<T,std::string>::value, void>::type parse_helper(std::istream& iss, T& x)
{
	iss >> x;
}

template <typename T>
inline typename std::enable_if<(is_iterable<T>::value || has_size_method<T>::value) && !std::is_same<T,std::string>::value, void>::type parse_helper(std::istream& iss, T& x)
{
	using value_type = typename cgogn::array_data_type<T>;
	cgogn::for_each(x,[&iss,&x](value_type& v)
	{
		serialization::parse(iss,v);
	});
}



template <typename T, std::size_t Precision>
inline typename std::enable_if<std::is_arithmetic<typename std::remove_reference<T>::type>::value || (!is_iterable<T>::value && !has_size_method<T>::value), void>::type ostream_writer_helper(std::ostream& o, const T& x, bool binary, bool little_endian)
{
	using numerical_type = typename fixed_precision<T, Precision>::type;
	if (binary)
	{
		numerical_type tmp = static_cast<numerical_type>(x);
		serialization::serialize_binary(o, tmp, little_endian);
	} else
		o << x;
}

template <typename T, std::size_t Precision>
inline typename std::enable_if<is_iterable<T>::value || (has_size_method<T>::value && has_operator_brackets<T>::value) || has_rows_method<T>::value, void>::type ostream_writer_helper(std::ostream& o, const T& x, bool binary, bool little_endian)
{
	using value_type = typename cgogn::array_data_type<T>;
	cgogn::for_each(x,[binary, little_endian, &o](const value_type& val)
	{
		serialization::ostream_writer<value_type, Precision>(o, val, binary, little_endian);
		if (!binary)
			o << " ";
	});
}

} // namespace internal

} // namespace cgogn

#endif // CGOGN_CORE_UTILS_SERIALIZATION_H_

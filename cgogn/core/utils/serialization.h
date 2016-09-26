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
#include <cgogn/core/dll.h>

namespace cgogn
{

namespace serialization
{

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

template<typename T>
inline typename std::enable_if<!has_size_method<T>::value, std::size_t>::type size(const T& x);
template<typename T>
inline typename std::enable_if<has_size_method<T>::value, std::size_t>::type size(const T& x);

template<typename T>
inline typename std::enable_if<!has_size_method<T>::value, std::size_t>::type size(const T& x)
{
	return 1;
}

template<typename T>
inline typename std::enable_if<has_size_method<T>::value, std::size_t>::type size(const T& x)
{
	return x.size();
}

template <typename T>
inline typename std::enable_if<!has_size_method<T>::value || std::is_same<T,std::string>::value, void>::type parse(std::istream& iss, T& x);
template <typename T>
inline typename std::enable_if<has_size_method<T>::value && is_iterable<T>::value && !std::is_same<T,std::string>::value, void>::type parse(std::istream& iss, T& x);
template <typename T>
inline typename std::enable_if<has_size_method<T>::value && !is_iterable<T>::value && !std::is_same<T,std::string>::value && (!has_cols_method<T>::value || !has_rows_method<T>::value), void>::type parse(std::istream& iss, T& x);
template <typename T>
inline typename std::enable_if<has_size_method<T>::value && !is_iterable<T>::value && !std::is_same<T,std::string>::value &&  has_cols_method<T>::value && has_rows_method<T>::value, void>::type parse(std::istream& iss, T& x);



template <typename T>
inline typename std::enable_if<!has_size_method<T>::value || std::is_same<T,std::string>::value, void>::type parse(std::istream& iss, T& x)
{
	iss >> x;
}

template <typename T>
inline typename std::enable_if<has_size_method<T>::value && is_iterable<T>::value && !std::is_same<T,std::string>::value, void>::type parse(std::istream& iss, T& x)
{
	for (auto& elem : x)
		parse(iss, elem);
}

template <typename T>
inline typename std::enable_if<has_size_method<T>::value && !is_iterable<T>::value && !std::is_same<T,std::string>::value && (!has_cols_method<T>::value || !has_rows_method<T>::value), void>::type parse(std::istream& iss, T& x)
{
	for (std::size_t i = 0u , end = size(x); i < end; ++i)
		parse(iss, x[i]);
}

template <typename T>
inline typename std::enable_if<has_size_method<T>::value && !is_iterable<T>::value && !std::is_same<T,std::string>::value &&  has_cols_method<T>::value && has_rows_method<T>::value, void>::type parse(std::istream& iss, T& x)
{
	for (std::size_t r = 0, rend = x.rows(); r < rend ; ++r)
		for (std::size_t c = 0, cend = x.cols(); c < cend ; ++c)
			parse(iss,x(r,c));
}


template <typename T, std::size_t Precision = 8ul>
inline typename std::enable_if<!has_size_method<T>::value, void>::type ostream_writer(std::ostream& o, const T& x, bool binary = false, bool little_endian = internal::cgogn_is_little_endian);
template <typename T, std::size_t Precision = 8ul>
inline typename std::enable_if<has_size_method<T>::value && !is_iterable<T>::value && (!has_rows_method<T>::value || !has_cols_method<T>::value), void>::type ostream_writer(std::ostream& o, const T& array, bool binary = false, bool little_endian = internal::cgogn_is_little_endian);
template <typename T, std::size_t Precision = 8ul>
inline typename std::enable_if<has_size_method<T>::value && !is_iterable<T>::value && has_rows_method<T>::value && has_cols_method<T>::value, void>::type ostream_writer(std::ostream& o, const T& array, bool binary = false, bool little_endian = internal::cgogn_is_little_endian);
template <typename T, std::size_t Precision = 8ul>
inline typename std::enable_if<has_size_method<T>::value && is_iterable<T>::value, void>::type ostream_writer(std::ostream& o, const T& array, bool binary = false, bool little_endian = internal::cgogn_is_little_endian);

template <typename T, std::size_t Precision>
inline typename std::enable_if<!has_size_method<T>::value, void>::type ostream_writer(std::ostream& o, const T& x, bool binary, bool little_endian)
{
	using numerical_type = typename fixed_precision<T, Precision>::type;
	if (binary)
	{
		numerical_type tmp = static_cast<numerical_type>(x);
		if (little_endian != internal::cgogn_is_little_endian)
			tmp = swap_endianness(tmp);
		save(o,&tmp,1ul);
	} else
		o << x;
}

template <typename T, std::size_t Precision>
inline typename std::enable_if<has_size_method<T>::value && !is_iterable<T>::value && (!has_rows_method<T>::value || !has_cols_method<T>::value), void>::type ostream_writer(std::ostream& o, const T& array, bool binary, bool little_endian)
{
	using nested_type = typename std::remove_const<typename std::remove_reference<decltype(array[0])>::type>::type;
	const std::size_t size = array.size();
	for(std::size_t i = 0ul ; i < size -1ul; ++i)
	{
		ostream_writer<nested_type, Precision>(o, array[i], binary, little_endian);
		if (!binary)
			o << " ";
	}
	ostream_writer<nested_type, Precision>(o, array[size-1ul], binary,little_endian);
}

template <typename T, std::size_t Precision>
inline typename std::enable_if<has_size_method<T>::value && is_iterable<T>::value, void>::type ostream_writer(std::ostream& o, const T& array, bool binary, bool little_endian)
{
	using nested_type = typename std::remove_const<typename std::remove_reference<decltype(*(array.begin()))>::type>::type;
	const auto end = array.end();

	for (auto it = array.begin(); it != end; )
	{
		ostream_writer<nested_type,Precision>(o,(*it), binary, little_endian);
		++it;
		if ((!binary) && it != end)
			o << " ";
	}
}

template <typename T, std::size_t Precision>
inline typename std::enable_if<has_size_method<T>::value && !is_iterable<T>::value && has_rows_method<T>::value && has_cols_method<T>::value, void>::type ostream_writer(std::ostream& o, const T& array, bool binary, bool little_endian)
{
	using nested_type = typename std::remove_const<typename std::remove_reference<decltype(array(0,0))>::type>::type;
	for(std::size_t r = 0, rend = array.rows(); r < rend ; ++r)
		for(std::size_t c = 0, cend = array.cols(); c < cend ; ++c)
		{
			ostream_writer<nested_type,Precision>(o, array(r,c), binary, little_endian);
			if ((!binary) && !((r == rend -1ul) && (c == cend -1ul)))
				o << " ";
		}
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
CGOGN_CORE_API bool known_size<std::string>(std::string const* /*src*/);

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
CGOGN_CORE_API void load<std::string>(std::istream& istream, std::string* dest, std::size_t quantity);

template <>
CGOGN_CORE_API void save<std::string>(std::ostream& ostream, std::string const* src, std::size_t quantity);

template <>
CGOGN_CORE_API std::size_t data_length<std::string>(std::string const* src, std::size_t quantity);


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

} // namespace cgogn

#endif // CGOGN_CORE_UTILS_SERIALIZATION_H_

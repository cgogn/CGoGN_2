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

#ifndef CORE_UTILS_SERIALIZATION_H_
#define CORE_UTILS_SERIALIZATION_H_

#include <iostream>
#include <vector>
#include <list>
#include <array>

#include <core/utils/assert.h>
#include <core/utils/dll.h>

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
CGOGN_UTILS_API bool known_size<std::string>(std::string const* /*src*/);

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
CGOGN_UTILS_API void load<std::string>(std::istream& istream, std::string* dest, std::size_t quantity);

template <>
CGOGN_UTILS_API void save<std::string>(std::ostream& ostream, std::string const* src, std::size_t quantity);

template <>
CGOGN_UTILS_API std::size_t data_length<std::string>(std::string const* src, std::size_t quantity);


// loading n vectors
template <typename U>
void load(std::istream& istream, std::vector<U>* dest, std::size_t quantity)
{
	cgogn_assert(istream.good());
	cgogn_assert(dest != nullptr);

	for (std::size_t i = 0; i < quantity ; ++i)
	{
		unsigned int vecSize;
		istream.read(reinterpret_cast<char*>(&vecSize), sizeof(unsigned int));
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

	for (std::size_t i = 0; i < quantity ; ++i)
	{
		const unsigned int size = static_cast<unsigned int>(src[i].size());
		ostream.write(reinterpret_cast<const char *>(&size), sizeof(unsigned int));
		save(ostream, &(src[i][0]), size);
	}
}

// compute data length of vector
template <typename U>
std::size_t data_length(std::vector<U> const * src, std::size_t quantity)
{
	cgogn_assert(src != nullptr);

	std::size_t total = 0;
	for (std::size_t i = 0; i < quantity ; ++i)
	{
		total += sizeof(unsigned int);// for size
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

	for (std::size_t i = 0; i < quantity ; ++i)
	{
		unsigned int listSize;
		istream.read(reinterpret_cast<char*>(&listSize), sizeof(unsigned int));
		std::vector<U> temp;
		temp.resize(listSize);
		load(istream, &(temp[0]), listSize);
		for(auto&& x : temp)
		{
			dest[i].emplace_back(std::move(x));
		}
	}
}

// saving n lists
template <typename U>
void save(std::ostream& ostream, std::list<U> const* src, std::size_t quantity)
{
	cgogn_assert(ostream.good());
	cgogn_assert(src != nullptr);

	for (std::size_t i = 0; i < quantity ; ++i)
	{
		const unsigned int size = static_cast<unsigned int>(src[i].size());
		ostream.write(reinterpret_cast<const char *>(&size), sizeof(unsigned int));
		for (const auto& elem : src[i])
			save(ostream, &elem, 1);
	}
}

// compute data length of list
template <typename U>
std::size_t data_length(std::list<U> const* src, std::size_t quantity)
{
	cgogn_assert(src != nullptr);

	std::size_t total = 0;
	for (std::size_t i = 0; i < quantity ; ++i)
	{
		total += sizeof(unsigned int); // for size
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
	for (std::size_t i = 0; i < quantity ; ++i)
	{
		load(istream, &(dest[i][0]), size);
	}
}

template <typename U, std::size_t size>
void save(std::ostream& ostream, std::array<U, size> const* src, std::size_t quantity)
{
	cgogn_assert(ostream.good());
	cgogn_assert(src);

	for (std::size_t i = 0; i < quantity ; ++i)
	{
		save(ostream, &(src[i][0]), size);
	}
}

template <typename U, std::size_t size>
std::size_t data_length(std::array<U, size>const* src, std::size_t quantity)
{
	cgogn_assert(src != nullptr);
	std::size_t total = 0u;
	for (std::size_t i = 0u; i < quantity ; ++i)
	{
		for (const auto& elem : src[i])
		{
			total += data_length(&elem,1);
		}
	}
	return total;
}

} // namespace serialization

} // namespace cgogn

#endif // CORE_UTILS_SERIALIZATION_H_

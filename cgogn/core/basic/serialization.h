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
#ifndef CORE_SERIALIZATION_H_
#define CORE_SERIALIZATION_H_

#include <iostream>
#include <vector>
#include <list>
#include <cassert>

namespace cgogn
{
namespace serialization
{

template<typename T>
void load(std::istream& istream, T * dest, std::size_t quantity)
{
	assert(dest != nullptr);
	istream.read(reinterpret_cast<char*>(dest), static_cast<std::streamsize>(quantity*sizeof(T)));
}

template<typename T>
void save(std::ostream& ostream, T const * src, std::size_t quantity)
{
	assert(src != nullptr);
	ostream.write(reinterpret_cast<const char *>(src), static_cast<std::streamsize>(quantity*sizeof(T)));
}

// first step : declare all overrides of load and save
template<typename U>
void load(std::istream& istream, std::vector<U>* dest, std::size_t quantity);
template<typename U>
void save(std::ostream& ostream, std::vector<U> const * src, std::size_t quantity);
template<typename U>
void load(std::istream& istream, std::list<U>* dest, std::size_t quantity);
template<typename U>
void save(std::ostream& ostream, std::list<U> const * src, std::size_t quantity);


// loading n vectors
template<typename U>
void load(std::istream& istream, std::vector<U>* dest, std::size_t quantity)
{
	assert(dest != nullptr);
	for (std::size_t i = 0; i < quantity ; ++i)
	{
		unsigned int vecSize;
		istream.read(reinterpret_cast<char*>(&vecSize), sizeof(unsigned int));
		dest[i].resize(vecSize);
		load(istream, &(dest[i][0]), vecSize);
	}
}

// saving n vectors
template<typename U>
void save(std::ostream& ostream, std::vector<U> const * src, std::size_t quantity)
{
	assert(src != nullptr);
	for (std::size_t i = 0; i < quantity ; ++i)
	{
		const unsigned int size = static_cast<unsigned int>(src[i].size());
		ostream.write(reinterpret_cast<const char *>(&size), sizeof(unsigned int));
		save(ostream, &(src[i][0]), size);
	}
}

// loading n lists
template<typename U>
void load(std::istream& istream, std::list<U>* dest, std::size_t quantity)
{
	assert(dest != nullptr);
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
template<typename U>
void save(std::ostream& ostream, std::list<U> const * src, std::size_t quantity)
{
	assert(src != nullptr);

	for (std::size_t i = 0; i < quantity ; ++i)
	{
		const unsigned int size = static_cast<unsigned int>(src[i].size());
		ostream.write(reinterpret_cast<const char *>(&size), sizeof(unsigned int));
		for (const auto& elem : src[i])
			save(ostream, &elem, 1);
	}
}

}
}

#endif // CORE_SERIALIZATION_H_


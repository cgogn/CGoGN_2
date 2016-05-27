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

#ifndef CGOGN_CORE_UTILS_BUFFERS_H_
#define CGOGN_CORE_UTILS_BUFFERS_H_

#include <vector>
#include <type_traits>

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/basic/dart.h>
#include <cgogn/core/basic/cell.h>

namespace cgogn
{

template <typename T>
class Buffers
{
	using value_type = T;
	static const uint32 DEFAULT_SIZE = 128u;
	static const uint32 SHRINK_SIZE  = 1024u;

protected:

	std::vector<std::vector<T>*> buffers_;

public:

	~Buffers()
	{
		for (auto i : buffers_)
			delete i;
	}

	inline std::vector<T>* buffer()
	{
		if (buffers_.empty())
		{
			std::vector<T>* v = new std::vector<T>;
			v->reserve(DEFAULT_SIZE);
			return v;
		}

		std::vector<T>* v = buffers_.back();
		buffers_.pop_back();
		return v;
	}

	inline void release_buffer(std::vector<T>* b)
	{
		if (b->capacity() > SHRINK_SIZE)
		{
			b->resize(DEFAULT_SIZE);
			b->shrink_to_fit();
		}

		b->clear();
		buffers_.push_back(b);
	}
};

template <>
class Buffers<Dart>
{
	using value_type = Dart;
	static const uint32 DEFAULT_SIZE = 128u;
	static const uint32 SHRINK_SIZE  = 1024u;

protected:

	std::vector<std::vector<Dart>*> buffers_;

public:

	~Buffers()
	{
		for (auto i : buffers_)
			delete i;
	}

	inline std::vector<Dart>* buffer()
	{
		if (buffers_.empty())
		{
			std::vector<Dart>* v = new std::vector<Dart>;
			v->reserve(DEFAULT_SIZE);
			return v;
		}

		std::vector<Dart>* v = buffers_.back();
		buffers_.pop_back();
		return v;
	}

	inline void release_buffer(std::vector<Dart>* b)
	{
		if (b->capacity() > SHRINK_SIZE)
		{
			b->resize(DEFAULT_SIZE);
			b->shrink_to_fit();
		}

		b->clear();
		buffers_.push_back(b);
	}

	template <typename CELL>
	inline std::vector<CELL>* cell_buffer()
	{
		static_assert(sizeof(CELL) == sizeof(Dart), "Cannot cast dart buffer in buffer of ??");
		return reinterpret_cast<std::vector<CELL>*>(buffer());
	}

	template <typename CELL>
	inline void release_cell_buffer(std::vector<CELL>* b)
	{
		static_assert(sizeof(CELL) == sizeof(Dart), "Cannot cast dart buffer in buffer of ??");
		release_buffer(reinterpret_cast<std::vector<Dart>*>(b));
	}
};

} // namespace cgogn

#endif // CGOGN_CORE_UTILS_BUFFERS_H_

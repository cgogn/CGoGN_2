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

#include <cgogn/core/utils/serialization.h>

namespace cgogn
{

namespace serialization
{

template <>
CGOGN_CORE_API bool known_size<std::string>(std::string const* /*src*/)
{
	return false;
}

// load string
template <>
CGOGN_CORE_API void load<std::string>(std::istream& istream, std::string* dest, std::size_t quantity)
{
	cgogn_assert(istream.good());
	cgogn_assert(dest != nullptr);

	char buffer[2048];
	for (std::size_t i = 0; i < quantity; ++i)
	{
		uint32 size;
		istream.read(reinterpret_cast<char*>(&size), sizeof(uint32));
		cgogn_assert(size < 2048);
		istream.read((buffer), size);
		dest[i].resize(size);
		for (uint32 j=0; j<size; ++j)
			dest[i][j] = buffer[j];
	}
}

//save string
template <>
CGOGN_CORE_API void save<std::string>(std::ostream& ostream, std::string const* src, std::size_t quantity)
{
	cgogn_assert(ostream.good());
	cgogn_assert(src != nullptr);

	for (std::size_t i = 0; i < quantity; ++i)
	{
		const uint32 size = uint32(src[i].length());
		ostream.write(reinterpret_cast<const char *>(&size), sizeof(uint32));
		const char* str = src[i].c_str();
		ostream.write(str, size);
	}
}

// compute data length of string
template <>
CGOGN_CORE_API std::size_t data_length<std::string>(std::string const* src, std::size_t quantity)
{
	cgogn_assert(src != nullptr);

	std::size_t total = 0;
	for (std::size_t i = 0; i < quantity; ++i)
	{
		total += sizeof(uint32); // for size
		total += src[i].length();
	}
	return total;
}

} // namespace serialization

} // namespace cgogn

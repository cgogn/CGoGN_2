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

#include <cgogn/core/container/chunk_array_gen.h>
#include <cgogn/core/utils/serialization.h>

namespace cgogn
{

void ChunkArrayGen::add_external_ref(ChunkArrayGen** ref)
{
	cgogn_message_assert(*ref == this, "ChunkArrayGen add_external_ref on other ChunkArrayGen");
	external_refs_.push_back(ref);
}

void ChunkArrayGen::remove_external_ref(ChunkArrayGen** ref)
{
	cgogn_message_assert(*ref == this, "ChunkArrayGen remove_external_ref on other ChunkArrayGen");
	auto it = std::find(external_refs_.begin(), external_refs_.end(), ref);
	cgogn_message_assert(it != external_refs_.end(), "ChunkArrayGen external ref not found");
	std::swap(*it, external_refs_.back());
	external_refs_.pop_back();
}

void ChunkArrayGen::skip(std::istream& fs)
{
	std::size_t chunk_bytes;
	serialization::load(fs, &chunk_bytes, 1);
	uint32 nb_lines;
	serialization::load(fs, &nb_lines, 1);
	fs.ignore(std::streamsize(chunk_bytes), EOF);
}

} // namespace cgogn

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

#define CGOGN_IO_DLL_EXPORT

#include <istream>
#include <iostream>

#ifdef CGOGN_WITH_ZLIB
#include <zlib.h>
#endif

#include <cgogn/core/utils/logger.h>
#include <cgogn/core/utils/string.h>
#include <cgogn/io/io_utils.h>

namespace cgogn
{

namespace io
{

#ifdef CGOGN_WITH_ZLIB
CGOGN_IO_API std::vector<unsigned char> zlib_decompress(const char* input, DataType header_type)
{

	std::uint64_t nb_blocks = UINT64_MAX;
	std::uint64_t uncompressed_block_size = UINT64_MAX;
	std::uint64_t last_block_size = UINT64_MAX;
	std::vector<uint32> compressed_size;

	uint32 word_size = 4u;
	std::vector<unsigned char> header_data;
	if (header_type == DataType::UINT64)
	{
		word_size = 8u;
		// we read the first 3 uint64
		header_data = base64_decode(input, 0, 32);
		nb_blocks = *reinterpret_cast<const std::uint64_t*>(&header_data[0]);
		uncompressed_block_size = *reinterpret_cast<const std::uint64_t*>(&header_data[8]);
		last_block_size = *reinterpret_cast<const std::uint64_t*>(&header_data[16]);
		compressed_size.resize(nb_blocks);
	} else
	{
		header_data = base64_decode(input, 0, 24);
		nb_blocks = *reinterpret_cast<const uint32*>(&header_data[0]);
		uncompressed_block_size = *reinterpret_cast<const uint32*>(&header_data[4]);
		last_block_size = *reinterpret_cast<const uint32*>(&header_data[8]);
		compressed_size.resize(nb_blocks);
	}

	std::size_t header_end = 4ul * word_size;
	std::size_t length = nb_blocks * word_size *4ul;
	while ((length % 12ul != 0ul))
		++length;
	length/=3ul;
	header_data = base64_decode(input, header_end, length);
	if (header_type == DataType::UINT64)
	{
		for (uint32 i = 0; i < nb_blocks; ++i)
			compressed_size[i] = uint32(*reinterpret_cast<const std::size_t*>(&header_data[8u * i]));
	} else
	{
		for (uint32 i = 0; i < nb_blocks; ++i)
			compressed_size[i] = uint32(*reinterpret_cast<const uint32*>(&header_data[4u * i]));
	}

	std::vector<unsigned char> data = base64_decode(input, header_end +length);
	std::vector<unsigned char> res(uncompressed_block_size*(nb_blocks-1u) + last_block_size);

	// zlib init
	int ret;
	z_stream zstream;
	zstream.zalloc = Z_NULL;
	zstream.zfree = Z_NULL;
	zstream.opaque = Z_NULL;
	uint32 in_data_it = 0u;
	uint32 out_data_it = 0u;
	for (uint32 i = 0; i < nb_blocks; ++i)
	{
		ret = inflateInit(&zstream);
		zstream.avail_in = compressed_size[i];
		zstream.next_in = &data[in_data_it];
		zstream.avail_out = uint32( (i == nb_blocks - 1u) ? last_block_size : uncompressed_block_size );
		zstream.next_out = &res[out_data_it];
		ret = inflate(&zstream, Z_NO_FLUSH);
		ret = inflateEnd(&zstream);
		in_data_it += compressed_size[i];
		out_data_it += uint32(uncompressed_block_size);
	}
	return res;
}
#endif

CGOGN_IO_API std::vector<unsigned char> base64_decode(const char* input, std::size_t begin, std::size_t length)
{
	const static char padCharacter('=');

	// needed if begin = 0
	while (std::isspace(*input))
		++input;

	for (std::size_t i = 0ul ; i < begin ;)
	{
		if (!std::isspace(*input))
			++i;
		++input;
	}

	const char* end = input;
	std::size_t i = 0ul;
	for ( ; i < length && (*end != '\0') ;)
	{
		if (!std::isspace(*end))
			++i;
		++end;
	}
	while (std::isspace(*(end-1)))
		--end;

	if (i % 4ul) //Sanity check
	{
		cgogn_log_error("base64_decode") << "The given length (" << length << ") is not a multiple of 4. This is not valid.";
		std::exit(EXIT_FAILURE);
	}

	size_t padding = 0;
	if (length)
	{
		if (*(end-1) == padCharacter)
			padding++;
		if (*(end-2) == padCharacter)
			padding++;
	}
	//Setup a vector to hold the result
	std::vector<unsigned char> decoded_chars;
	decoded_chars.reserve(((i/4ul)*3ul) - padding);
	long int temp=0; //Holds decoded quanta
	const char* cursor = input;
	while (cursor != end)
	{
		cgogn_assert(!std::isspace(*cursor));
		cgogn_assert(!std::isspace(*(cursor+1)));
		cgogn_assert(!std::isspace(*(cursor+2)));
		cgogn_assert(!std::isspace(*(cursor+3)));
		for (size_t quantumPosition = 0; quantumPosition < 4; quantumPosition++)
		{
			temp <<= 6;
			if       (*cursor >= 0x41 && *cursor <= 0x5A) // This area will need tweaking if
				temp |= *cursor - 0x41;		              // you are using an alternate alphabet
			else if  (*cursor >= 0x61 && *cursor <= 0x7A)
				temp |= *cursor - 0x47;
			else if  (*cursor >= 0x30 && *cursor <= 0x39)
				temp |= *cursor + 0x04;
			else if  (*cursor == 0x2B)
				temp |= 0x3E; //change to 0x2D for URL alphabet
			else if  (*cursor == 0x2F)
				temp |= 0x3F; //change to 0x5F for URL alphabet
			else if  (*cursor == padCharacter) //pad
			{
				switch( end - cursor )
				{
					case 1: //One pad character
						decoded_chars.push_back((temp >> 16) & 0x000000FF);
						decoded_chars.push_back((temp >> 8 ) & 0x000000FF);
						return decoded_chars;
					case 2: //Two pad characters
						decoded_chars.push_back((temp >> 10) & 0x000000FF);
						return decoded_chars;
					default:
						cgogn_log_error("base64_decode") << "Invalid Padding.";
						std::exit(EXIT_FAILURE);
				}
			} else
			{
				cgogn_log_error("base64_decode") << "Non-Valid Character.";
				std::exit(EXIT_FAILURE);
			}
			cursor++;
		}
		decoded_chars.push_back((temp >> 16) & 0x000000FF);
		decoded_chars.push_back((temp >> 8 ) & 0x000000FF);
		decoded_chars.push_back((temp      ) & 0x000000FF);
	}
	return decoded_chars;
}

CGOGN_IO_API FileType get_file_type(const std::string& filename)
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
	if (extension == "vtp")
		return FileType::FileType_VTP;
	if (extension == "meshb" || extension == "mesh")
		return FileType::FileType_MESHB;
	if (extension == "msh")
		return FileType::FileType_MSH;
	if (extension == "node" || extension == "ele")
		return FileType::FileType_TETGEN;
	if (extension == "nas" || extension == "bdf")
		return FileType::FileType_NASTRAN;
	if (extension == "tet")
		return FileType::FileType_AIMATSHAPE;

	return FileType::FileType_UNKNOWN;
}

CGOGN_IO_API DataType get_data_type(const std::string& type_name)
{
	if (type_name == name_of_type(float32()))
		return DataType::FLOAT;
	else if (type_name == name_of_type(float64()))
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

CharArrayBuffer::~CharArrayBuffer() {}

IMemoryStream::~IMemoryStream() {}

} // namespace io

} // namespace cgogn


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

#include <zlib.h>

#include <core/utils/string.h>
#include <io/io_utils.h>

namespace cgogn
{

namespace io
{

CGOGN_IO_API void zlib_decompress(std::string& input, DataType header_type)
{
	input.erase(std::remove(input.begin(), input.end(), ' '), input.end());
	input.erase(std::remove(input.begin(), input.end(), '\n'), input.end());
	input.erase(std::remove(input.begin(), input.end(), '\t'), input.end());

	std::uint64_t nb_blocks = UINT64_MAX;
	std::uint64_t uncompressed_block_size = UINT64_MAX;
	std::uint64_t last_block_size = UINT64_MAX;
	std::vector<unsigned int> compressed_size;

	unsigned int word_size = 4u;
	std::string header;
	std::vector<unsigned char> header_data;
	if (header_type == DataType::UINT64)
	{
		word_size = 8u;
		// we read the first 3 uint64
		header = input.substr(0, 24);
		header_data = base64_decode(header);
		nb_blocks = *reinterpret_cast<const std::uint64_t*>(&header_data[0]);
		uncompressed_block_size = *reinterpret_cast<const std::uint64_t*>(&header_data[8]);
		last_block_size = *reinterpret_cast<const std::uint64_t*>(&header_data[16]);
		compressed_size.resize(nb_blocks);
	} else
	{
		header = input.substr(0, 12);
		header_data = base64_decode(header);
		nb_blocks = *reinterpret_cast<const unsigned int*>(&header_data[0]);
		uncompressed_block_size = *reinterpret_cast<const unsigned int*>(&header_data[4]);
		last_block_size = *reinterpret_cast<const unsigned int*>(&header_data[8]);
		compressed_size.resize(nb_blocks);
	}

	std::size_t header_end = 4ul *word_size;
std:size_t length = nb_blocks* word_size *4ul;
	while ((length % 12ul != 0ul))
		++length;
	length/=3ul;
	header = input.substr(header_end, length);
	header_data = base64_decode(header);
	if (header_type == DataType::UINT64)
	{
		for (unsigned int i = 0; i < nb_blocks; ++i)
			compressed_size[i] = *reinterpret_cast<const std::size_t*>(&header_data[8u*i]);
	} else
	{
		for (unsigned int i = 0; i < nb_blocks; ++i)
			compressed_size[i] = *reinterpret_cast<const unsigned int*>(&header_data[4u*i]);
	}

	std::string data_str(input.substr(header_end +length, std::string::npos));

	std::vector<unsigned char> data = base64_decode(data_str);
	std::vector<unsigned char> res;
	res.resize(uncompressed_block_size*(nb_blocks-1u) + last_block_size);

	// zlib init
	int ret;
	z_stream zstream;
	zstream.zalloc = Z_NULL;
	zstream.zfree = Z_NULL;
	zstream.opaque = Z_NULL;
	unsigned int in_data_it = 0u;
	unsigned int out_data_it = 0u;
	for (unsigned int i = 0; i < nb_blocks; ++i)
	{
		ret = inflateInit(&zstream);
		zstream.avail_in = compressed_size[i];
		zstream.next_in = &data[in_data_it];
		zstream.avail_out = (i == nb_blocks -1u) ? last_block_size : uncompressed_block_size;
		zstream.next_out = &res[out_data_it];
		ret = inflate(&zstream, Z_NO_FLUSH);
		ret = inflateEnd(&zstream);
		in_data_it += compressed_size[i];
		out_data_it+=uncompressed_block_size;
	}
	input = std::string(reinterpret_cast<char*>(&res[0]), res.size());
}

CGOGN_IO_API std::vector<unsigned char> base64_decode(std::string& input)
{
	const static char padCharacter('=');
	if (input.length() % 4) //Sanity check
		throw std::runtime_error("Non-Valid base64!");
	size_t padding = 0;
	if (input.length())
	{
		if (input[input.length()-1] == padCharacter)
			padding++;
		if (input[input.length()-2] == padCharacter)
			padding++;
	}
	//Setup a vector to hold the result
	std::vector<unsigned char> decoded_chars;
	decoded_chars.reserve(((input.length()/4)*3) - padding);
	long int temp=0; //Holds decoded quanta
	std::basic_string<char>::const_iterator cursor = input.begin();
	while (cursor < input.end())
	{
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
				switch( input.end() - cursor )
				{
					case 1: //One pad character
						decoded_chars.push_back((temp >> 16) & 0x000000FF);
						decoded_chars.push_back((temp >> 8 ) & 0x000000FF);
						return decoded_chars;
					case 2: //Two pad characters
						decoded_chars.push_back((temp >> 10) & 0x000000FF);
						return decoded_chars;
					default:
						throw std::runtime_error("Invalid Padding in Base 64!");
				}
			}  else
				throw std::runtime_error("Non-Valid Character in Base 64!");
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
	return FileType::FileType_UNKNOWN;
}

CGOGN_IO_API DataType get_data_type(const std::string& type_name)
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

} // namespace io

} // namespace cgogn


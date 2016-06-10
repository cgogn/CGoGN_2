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
#include <map>

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

CGOGN_IO_API FileType file_type(const std::string& filename)
{
	const std::string& ext = to_lower(extension(filename));
	static const std::map<std::string, FileType> file_type_map{
		{"off", FileType_OFF},
		{"obj", FileType_OBJ},
		{"stl", FileType_STL},
		{"ply", FileType_PLY},
		{"vtk", FileType_VTK_LEGACY},
		{"vtu", FileType_VTU},
		{"vtp", FileType_VTP},
		{"meshb", FileType_MESHB},
		{"mesh", FileType_MESHB},
		{"msh", FileType_MSH},
		{"node", FileType_TETGEN},
		{"ele", FileType_TETGEN},
		{"nas", FileType_NASTRAN},
		{"bdf", FileType_NASTRAN},
		{"tet", FileType_AIMATSHAPE}
	};

	const auto it = file_type_map.find(ext);
	if ( it != file_type_map.end())
		return it->second;
	else
		return FileType::FileType_UNKNOWN;
}

CGOGN_IO_API DataType data_type(const std::string& type_name)
{
	static const std::map<std::string, DataType> data_type_map{
		{name_of_type(float32()), DataType::FLOAT},
		{name_of_type(float64()), DataType::DOUBLE},
		{name_of_type(char()), DataType::CHAR},
		{name_of_type(int8()), DataType::INT8},
		{name_of_type(uint8()), DataType::UINT8},
		{name_of_type(int16()), DataType::INT16},
		{name_of_type(uint16()), DataType::UINT16},
		{name_of_type(int32()), DataType::INT32},
		{name_of_type(uint32()), DataType::UINT32},
		{name_of_type(int64()), DataType::INT64},
		{name_of_type(uint64()), DataType::UINT64}
	};

	const auto it = data_type_map.find(type_name);
	if ( it != data_type_map.end())
		return it->second;
	else
		return DataType::UNKNOWN;
}

CharArrayBuffer::~CharArrayBuffer() {}

IMemoryStream::~IMemoryStream() {}

CGOGN_IO_API bool file_exists(const std::string& filename)
{
	return std::ifstream(filename).good();
}

CGOGN_IO_API std::unique_ptr<std::ofstream> create_file(const std::string& filename, bool binary)
{
	std::unique_ptr<std::ofstream> output;

	std::ios_base::openmode open_mode = std::ios_base::out;
	if (binary)
		open_mode |= std::ios::binary;

	std::string new_filename(filename);
	if (file_exists(new_filename))
	{
		uint32 i{1u};
		const std::string base_name = remove_extension(filename);
		do
		{
			new_filename = base_name + "(" + std::to_string(i++)  + ")." + extension(filename);
		} while (file_exists(new_filename));
		cgogn_log_warning("create_file")  << "The output filename has been changed to \"" << new_filename << "\"";
	}
	output = cgogn::make_unique<std::ofstream>(new_filename, open_mode);
	if (!output->good())
		cgogn_log_warning("create_file")  << "Error while opening the file \"" << filename << "\"";
	return output;
}

} // namespace io

} // namespace cgogn


﻿/*******************************************************************************
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


#include <istream>
#include <iostream>
#include <map>

#include <zlib.h>

#include <cgogn/core/utils/logger.h>
#include <cgogn/core/utils/string.h>
#include <cgogn/io/io_utils.h>

namespace cgogn
{

namespace io
{

CGOGN_IO_API std::vector<std::vector<unsigned char>> zlib_compress(const unsigned char* input, std::size_t size, std::size_t chunk_size)
{
	chunk_size = std::min(size, chunk_size);
	std::size_t buffer_size{0ul};
	std::vector<std::vector<unsigned char>> res;
	res.reserve(64);

	// zlib init
	z_stream zstream;
	int32 ret;
	unused_parameters(ret);// release warning
	zstream.zalloc = Z_NULL;
	zstream.zfree = Z_NULL;
	zstream.opaque = Z_NULL;

	do
	{
		ret = deflateInit(&zstream, Z_BEST_COMPRESSION);
		cgogn_assert(ret == Z_OK);

		buffer_size = static_cast<std::size_t>(deflateBound(&zstream, static_cast<uLong>(std::min(chunk_size, size))));

		zstream.avail_in = static_cast<uInt>(std::min(chunk_size, size));
		size -= zstream.avail_in;
		zstream.next_in = input;
		input += zstream.avail_in;
		res.emplace_back(buffer_size);
		zstream.avail_out = static_cast<uInt>(res.back().size());
		zstream.next_out = &(res.back()[0]);
		cgogn_assert(zstream.next_out  != nullptr);
		ret = deflate(&zstream, Z_FINISH);
		cgogn_assert(ret == Z_STREAM_END);
		res.back().resize(buffer_size - zstream.avail_out);
	} while (size > 0ul);

	/* clean up and return */
	(void)deflateEnd(&zstream);

	return res;
}

CGOGN_IO_API std::vector<unsigned char> zlib_decompress(const char* input, DataType header_type, std::size_t input_length)
{

	uint64 nb_blocks = UINT64_MAX;
	uint64 uncompressed_block_size = UINT64_MAX;
	uint64 last_block_size = UINT64_MAX;
	std::vector<uint32> compressed_size;

	const uint32 word_size = (header_type == DataType::UINT64)? 8u:4u;
	std::vector<unsigned char> header_data;
	header_data = base64_decode(input, 4*word_size);
	if (!header_data.empty())
	{
		if (header_type == DataType::UINT64)
		{
			nb_blocks = *reinterpret_cast<const std::uint64_t*>(&header_data[0]);
			uncompressed_block_size = *reinterpret_cast<const std::uint64_t*>(&header_data[8]);
			last_block_size = *reinterpret_cast<const std::uint64_t*>(&header_data[16]);
		} else {
			nb_blocks = *reinterpret_cast<const uint32*>(&header_data[0]);
			uncompressed_block_size = *reinterpret_cast<const uint32*>(&header_data[4]);
			last_block_size = *reinterpret_cast<const uint32*>(&header_data[8]);
		}
		compressed_size.resize(nb_blocks);
	} else {
		cgogn_log_warning("zlib_decompress") << "Unable to decode the header.";
		return std::vector<unsigned char>();
	}

	const uint64 header_end = 4ul * word_size;
	uint64 length = (nb_blocks * word_size * 4ul + 2ul) / 3ul; // round up of 4/3 *( nb_blocks * word_size)
	length = ((length + 3ul)/4ul)*4ul; // next multiple of 4

	header_data = base64_decode(input + header_end, length);
	if (header_data.empty())
	{
		cgogn_log_warning("zlib_decompress") << "Unable to decode the header.";
		return std::vector<unsigned char>();
	}

	if (header_type == DataType::UINT64)
		for (uint32 i = 0; i < nb_blocks; ++i)
			compressed_size[i] = uint32(*reinterpret_cast<const uint64*>(&header_data[8u * i]));
	else
		for (uint32 i = 0; i < nb_blocks; ++i)
			compressed_size[i] = uint32(*reinterpret_cast<const uint32*>(&header_data[4u * i]));


	const char* const data_begin = input + (header_end + length);
	std::vector<unsigned char> data = base64_decode(data_begin, input_length -(length + header_end));
	std::vector<unsigned char> res(uncompressed_block_size*(nb_blocks-1u) + last_block_size);

	if (data.empty()) // base64_decode failed
	{
		cgogn_log_warning("zlib_decompress") << "Unable to decode the data.";
		return std::vector<unsigned char>();
	}

	// zlib init
	z_stream zstream;
	int32 ret;
	unused_parameters(ret);// release warning
	zstream.zalloc = Z_NULL;
	zstream.zfree = Z_NULL;
	zstream.opaque = Z_NULL;
	uint32 in_data_it = 0u;
	uint32 out_data_it = 0u;
	for (uint32 i = 0; i < nb_blocks; ++i)
	{
		ret = inflateInit(&zstream);
		cgogn_assert(ret == Z_OK);
		zstream.avail_in = compressed_size[i];
		zstream.next_in = &data[in_data_it];
		zstream.avail_out = uint32( (i == nb_blocks - 1u) ? last_block_size : uncompressed_block_size );
		zstream.next_out = &res[out_data_it];
		ret = inflate(&zstream, Z_FINISH);
		cgogn_assert(ret == Z_STREAM_END);
		ret = inflateEnd(&zstream);
		in_data_it += compressed_size[i];
		out_data_it += uint32(uncompressed_block_size);
	}

	return res;
}

CGOGN_IO_API std::vector<char> base64_encode(const char* input_buffer, std::size_t buffer_size)
{

	const static char encode_lookup[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

	int i = 0;
	int j = 0;
	char char_array_3[3];
	char char_array_4[4];

	std::vector<char> res;
	res.reserve(((buffer_size + 2)/3) * 4);

	while (buffer_size--)
	{
		cgogn_assert(*input_buffer != '\0');
		char_array_3[i++] = *(input_buffer++);
		if (i == 3)
		{
			char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
			char_array_4[1] = char(((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4));
			char_array_4[2] = char(((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6));
			char_array_4[3] = char_array_3[2] & 0x3f;

			for(i = 0; i <4 ; ++i)
				res.push_back(encode_lookup[uint32(char_array_4[i])]);
			i = 0;
		}
	}

	if (i != 0)
	{
		for(j = i; j < 3; j++)
			char_array_3[j] = '\0';

		char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
		char_array_4[1] = char(((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4));
		char_array_4[2] = char(((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6));
		char_array_4[3] = char_array_3[2] & 0x3f;

		for (j = 0; (j < i + 1); j++)
			res.push_back(encode_lookup[uint32(char_array_4[j])]);

		while((i++ < 3))
			res.push_back('=');
	}

	return res;
}


CGOGN_IO_API std::vector<unsigned char> base64_decode(const char* input, std::size_t length)
{
	const std::locale locale;

	if (!input || length == 0)
	{
		cgogn_log_warning("base64_decode") << "Nothing to decode.";
		return std::vector<unsigned char>();
	}

	if (length % 4ul) //Sanity check
	{
		cgogn_log_warning("base64_decode") << "The given length (" << length << ") is not a multiple of 4. This is not valid.";
		return std::vector<unsigned char>();
	}

	const char* const end = input + length;

	//Setup a vector to hold the result
	std::vector<unsigned char> decoded_chars;
	decoded_chars.reserve(((length/4ul)*3ul));
	cgogn::int32 temp = 0; //Holds decoded quanta
	const char* cursor = input;
	while (cursor != end)
	{
		cgogn_assert(!std::isspace(*cursor, locale));
		cgogn_assert(!std::isspace(*(cursor+1), locale));
		cgogn_assert(!std::isspace(*(cursor+2), locale));
		cgogn_assert(!std::isspace(*(cursor+3), locale));
		for (size_t quantumPosition = 0; quantumPosition < 4; quantumPosition++)
		{
			const char c = *cursor;
			temp <<= 6;
			if       (c >= 'A' && c <= 'Z')
				temp |= c - 'A';
			else if  (c >= 'a' && c <= 'z')
				temp |= (c - 'a' + 26);
			else if  (c >= '0' && c <= '9')
				temp |= (c -'0' + 52);
			else if  (c == '+')
				temp |= 62;
			else if  (c == '/')
				temp |= 63;
			else if  (c == '=') //pad
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
						cgogn_log_warning("base64_decode") << "Invalid Padding.";
						return std::vector<unsigned char>();
				}
			} else {
				cgogn_log_warning("base64_decode") << "Non-Valid Character.";
				return std::vector<unsigned char>();
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
		{"2dm", FileType_2DM},
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
		{"tet", FileType_AIMATSHAPE},
		{"tetmesh", FileType_TETMESH}
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

CGOGN_IO_API std::unique_ptr<std::ofstream> create_file(const std::string& filename, bool binary, bool overwrite)
{
	std::unique_ptr<std::ofstream> output;

	std::ios_base::openmode open_mode = std::ios_base::out;
	if (binary)
		open_mode |= std::ios::binary;

	std::string new_filename(filename);
	if (file_exists(new_filename))
	{
		if (overwrite)
		{
			std::remove(filename.c_str());
			cgogn_log_info("create_file")  << "The file \"" << new_filename << "\" has been deleted.";
		} else {
			uint32 i{1u};
			const std::string base_name = remove_extension(filename);
			do
			{
				new_filename = base_name + "-" + std::to_string(i++) + std::string(".") + extension(filename);
			} while (file_exists(new_filename));
			cgogn_log_warning("create_file")  << "The output filename has been changed to \"" << new_filename << "\"";
		}
	}
	output = cgogn::make_unique<std::ofstream>(new_filename, open_mode);
	if (!output->good())
		cgogn_log_warning("create_file")  << "Error while opening the file \"" << filename << "\"";
	return output;
}

CGOGN_IO_API std::istream& getline_safe(std::istream& is, std::string& str)
{
	str.clear();
	std::istream::sentry se(is, true); // http://en.cppreference.com/w/cpp/io/basic_istream/sentry
	std::streambuf* sb = is.rdbuf();

	while (true)
	{
		const auto c = sb->sbumpc();
		switch (c)
		{
		case '\n':
			return is;
		case '\r':
			if (sb->sgetc() == '\n')
				sb->sbumpc();
			return is;
		case EOF: // Also handle the case when the last line has no line ending
			if (str.empty())
				is.setstate(std::ios::eofbit);
			return is;
		default:
			str.push_back(static_cast<char>(c));
		}
	}
}

ExportOptions::ExportOptions() :
	filename_(),
	position_attributes_(),
	attributes_to_export_(),
	binary_(false),
	compress_(false),
	overwrite_(true)
{}

ExportOptions::ExportOptions(const ExportOptions& eo) :
	filename_(eo.filename_),
	position_attributes_(eo.position_attributes_),
	attributes_to_export_(eo.attributes_to_export_),
	cell_filter_(eo.cell_filter_),
	binary_(eo.binary_),
	compress_(eo.compress_),
	overwrite_(eo.overwrite_)
{}

ExportOptions::ExportOptions(ExportOptions&& eo) :
	filename_(std::move(eo.filename_)),
	position_attributes_(std::move(eo.position_attributes_)),
	attributes_to_export_(std::move(eo.attributes_to_export_)),
	cell_filter_(std::move(eo.cell_filter_)),
	binary_(eo.binary_),
	compress_(eo.compress_),
	overwrite_(eo.overwrite_)
{}

ExportOptions ExportOptions::create()
{
	return ExportOptions();
}

} // namespace io

} // namespace cgogn


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

#define CGOGN_IO_FORMATS_VTK_CPP_

#include <cgogn/io/formats/vtk.h>
#include <cctype>

namespace cgogn
{

namespace io
{


CGOGN_IO_EXPORT std::string  vtk_data_type_to_cgogn_name_of_type(const std::string& vtk_type_str)
{
	const std::string& data_type = to_lower(vtk_type_str);
	static const std::map<std::string, std::string> type_map{
		{"char", name_of_type(int8())},
		{"int8", name_of_type(int8())},
		{"unsigned_char", name_of_type(uint8())},
		{"uint8", name_of_type(uint8())},
		{"short", name_of_type(int16())},
		{"int16", name_of_type(int16())},
		{"unsigned_short", name_of_type(uint16())},
		{"uint16", name_of_type(uint16())},
		{"int", name_of_type(int32())},
		{"int32", name_of_type(int32())},
		{"unsigned_int", name_of_type(uint32())},
		{"uint32", name_of_type(uint32())},
		{"long", name_of_type(int64())},
		{"int64", name_of_type(int64())},
		{"unsigned_long", name_of_type(uint64())},
		{"uint64", name_of_type(uint64())},
		{"float", name_of_type(float32())},
		{"float32", name_of_type(float32())},
		{"double", name_of_type(float64())},
		{"float64", name_of_type(float64())}
	};

	const auto it = type_map.find(data_type);
	if ( it != type_map.end())
		return it->second;
	cgogn_log_error("vtk_data_type_to_cgogn_name_of_type") << "Unknown vtk type \"" << vtk_type_str << "\".";
	return std::string();
}

CGOGN_IO_EXPORT std::string cgogn_name_of_type_to_vtk_xml_data_type(const std::string& cgogn_type)
{
	static const std::map<std::string, std::string> type_map{
		{name_of_type(int8()), "Int8"},
		{name_of_type(uint8()), "UInt8"},
		{name_of_type(int16()), "Int16"},
		{name_of_type(uint16()), "UInt16"},
		{name_of_type(int32()), "Int32"},
		{name_of_type(uint32()), "UInt32"},
		{name_of_type(int64()), "Int64"},
		{name_of_type(uint64()), "UInt64"},
		{name_of_type(float32()), "Float32"},
		{name_of_type(float64()), "Float64"}
	};

	const auto it = type_map.find(cgogn_type);
	if ( it != type_map.end())
		return it->second;

	cgogn_log_error("cgogn_name_of_type_to_vtk_xml_data_type") << "Unknown cgogn type \"" << cgogn_type << "\".";
	return std::string();
}

CGOGN_IO_EXPORT std::string cgogn_name_of_type_to_vtk_legacy_data_type(const std::string& cgogn_type)
{
	static const std::map<std::string, std::string> type_map{
		{name_of_type(bool()), "bit"},
		{name_of_type(int8()), "char"},
		{name_of_type(uint8()), "unsigned_char"},
		{name_of_type(int16()), "short"},
		{name_of_type(uint16()), "unsigned_short"},
		{name_of_type(int32()), "int"},
		{name_of_type(uint32()), "unsigned_int"},
		{name_of_type(int64()), "long"},
		{name_of_type(uint64()), "unsigned_long"},
		{name_of_type(float32()), "float"},
		{name_of_type(float64()), "double"}
	};

	const auto it = type_map.find(cgogn_type);
	if ( it != type_map.end())
		return it->second;

	cgogn_log_error("cgogn_name_of_type_to_vtk_legacy_data_type") << "Unknown cgogn type \"" << cgogn_type << "\".";
	return std::string();
}

CGOGN_IO_EXPORT std::vector<unsigned char> read_binary_xml_data(const char* data_str, bool is_compressed, DataType header_type)
{
	const char* begin = data_str;

	// remove spaces at the beginning
	while(std::isspace(*begin))
		++begin;


	// remove spaces at the end
	std::size_t len = std::strlen(begin);
	const char* end = begin + len -1u;
	while(std::isspace(*end))
	{
		--end;
		--len;
	}

	if (!is_compressed)
	{
		std::vector<unsigned char> decode = base64_decode(begin, len);

		if (!decode.empty())
		{
			auto vec_begin = decode.begin() + (header_type == DataType::UINT32 ? 4u : 8u);
			auto vec_end = decode.end();
			return std::vector<unsigned char>(vec_begin, vec_end);
		} else
			return std::vector<unsigned char>();
	}
	else
		return zlib_decompress(begin, header_type, len);
}

CGOGN_IO_EXPORT void write_binary_xml_data(std::ostream& output, const char* data_str, std::size_t size, bool compress)
{
	std::vector<char> data;
	std::vector<uint32> header;
	if (!compress)
	{
		header.push_back(static_cast<uint32>(size));
		data.resize(sizeof(uint32) + size);

		std::memcpy(&data[0], reinterpret_cast<const char*>(&header[0]), sizeof(uint32)* header.size());
		std::memcpy(&data[sizeof(uint32)], data_str, size);
	}
	else
	{
		const std::size_t uncompressed_chunk_size = std::min(size, std::size_t(1048576));
		const std::vector<std::vector<unsigned char>>& compressed_blocks = zlib_compress(reinterpret_cast<const unsigned char*>(data_str), size, uncompressed_chunk_size);
		std::size_t compressed_size{0ul};
		const std::size_t last_block_size = (compressed_blocks.size() == 1ul) ? uncompressed_chunk_size : size % uncompressed_chunk_size;

		header.push_back(static_cast<uint32>(compressed_blocks.size()));
		header.push_back(static_cast<uint32>(uncompressed_chunk_size));
		header.push_back(static_cast<uint32>(last_block_size));
		for (const auto& block : compressed_blocks)
		{
			header.push_back(static_cast<uint32>(block.size()));
			compressed_size += block.size();
		}

		const auto& encoded_header = base64_encode(reinterpret_cast<char*>(&header[0]), header.size() * sizeof(uint32));
		output.write(&encoded_header[0], encoded_header.size());

		data.resize(compressed_size);
		char* data_ptr = &data[0];
		for (const auto& block : compressed_blocks)
		{
			const char* src = reinterpret_cast<const char*>(&block[0]);
			std::memcpy(data_ptr, src, block.size());
			data_ptr += block.size();
		}
	}

	const auto& encoded_data = base64_encode(&data[0], data.size());
	output.write(&encoded_data[0], encoded_data.size());
}

} // namespace io

} // namespace cgogn

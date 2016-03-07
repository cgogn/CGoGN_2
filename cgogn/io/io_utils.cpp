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

#include <core/utils/string.h>
#include <io/io_utils.h>

namespace cgogn
{

namespace io
{

CGOGN_IO_API std::vector<unsigned char> base64_decode(std::string& input)
{
	input.erase(std::remove(input.begin(), input.end(), ' '), input.end());
	input.erase(std::remove(input.begin(), input.end(), '\n'), input.end());
	input.erase(std::remove(input.begin(), input.end(), '\t'), input.end());

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


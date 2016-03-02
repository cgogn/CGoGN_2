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

#ifndef IO_VTK_CELL_TYPES_H_
#define IO_VTK_CELL_TYPES_H_

namespace cgogn
{

enum VTK_CELL_TYPES
{
	VTK_VERTEX = 1,
	VTK_POLY_VERTEX = 2,
	VTK_LINE = 3,
	VTK_POLY_LINE = 4,
	VTK_TRIANGLE = 5,
	VTK_TRIANGLE_STRIP = 6,
	VTK_POLYGON = 7,
	VTK_PIXEL = 8,
	VTK_QUAD = 9,

	VTK_TETRA = 10,
	VTK_VOXEL = 11,
	VTK_HEXAHEDRON = 12,
	VTK_WEDGE = 13,
	VTK_PYRAMID = 14,

	VTK_QUADRATIC_EDGE = 21,
	VTK_QUADRATIC_TRIANGLE = 22,
	VTK_QUADRATIC_QUAD = 23,
	VTK_QUADRATIC_TETRA = 24,
	VTK_QUADRATIC_HEXAHEDRON = 25
};

/**
 * @brief vtk_data_type_to_cgogn_name_of_type : convert the type names we can find in VTK files to the one we use in cgogn.
 * @param vtk_type_str a typename extracted from a vtk file
 * @return a typename string that can be match with some cgogn::name_of_type
 */
inline std::string vtk_data_type_to_cgogn_name_of_type(const std::string& vtk_type_str)
{
	const std::string& data_type = to_lower(vtk_type_str);
	if (data_type == "char" || data_type == "int8")
		return name_of_type(std::int8_t());
	if (data_type == "unsigned_char" || data_type == "uint8")
		return name_of_type(std::uint8_t());
	if (data_type == "short" || data_type == "int16")
		return name_of_type(std::int16_t());
	if (data_type == "unsigned_short" || data_type == "uint16")
		return name_of_type(std::uint16_t());
	if (data_type == "int" || data_type == "int32")
		return name_of_type(std::int32_t());
	if (data_type == "unsigned_int" || data_type == "uint32")
		return name_of_type(std::uint32_t());
	if (data_type == "long" || data_type == "int64")
		return name_of_type(std::int64_t());
	if (data_type == "unsigned_long" || data_type == "uint64")
		return name_of_type(std::uint64_t());
	if (data_type == "float"  || data_type == "float32")
		return name_of_type(float());
	if (data_type == "double" || data_type == "float64")
		return name_of_type(double());

	std::cerr << "vtk_data_type_to_cgogn_name_of_type : unknown vtk type : " << vtk_type_str << std::endl;
	return std::string();
}

} // namespace cgogn
#endif // IO_VTK_CELL_TYPES_H_

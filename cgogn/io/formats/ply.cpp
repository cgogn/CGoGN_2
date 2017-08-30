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

#define CGOGN_IO_FORMATS_PLY_CPP_

#include <cgogn/io/formats/ply.h>

namespace cgogn
{

namespace io
{

template class CGOGN_IO_API PlySurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_API PlySurfaceImport<CMap2, Eigen::Vector3f>;
template class CGOGN_IO_API PlySurfaceImport<CMap2, geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_API PlySurfaceImport<CMap2, geometry::Vec_T<std::array<float32, 3>>>;

template class CGOGN_IO_API PlySurfaceExport<CMap2>;

CGOGN_IO_API std::string cgogn_name_of_type_to_ply_data_type(const std::string& cgogn_type)
{
	static const std::map<std::string, std::string> type_map{
		{name_of_type(int8()), "int8"},
		{name_of_type(uint8()), "uint8"},
		{name_of_type(int16()), "int16"},
		{name_of_type(uint16()), "uint16"},
		{name_of_type(int32()), "int"},
		{name_of_type(uint32()), "uint"},
		{name_of_type(float32()), "float32"},
		{name_of_type(float64()), "float64"}
	};

	const auto it = type_map.find(cgogn_type);
	if ( it != type_map.end())
		return it->second;

	cgogn_log_error("cgogn_name_of_type_to_ply_data_type") << "Unknown cgogn type \"" << cgogn_type << "\".";
	return std::string();
}

} // namespace io

} // namespace cgogn

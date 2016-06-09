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
#define CGOGN_IO_VTK_IO_CPP_

#include <cgogn/io/vtk_io.h>

namespace cgogn
{
namespace io
{
template class CGOGN_IO_API VtkIO<DefaultMapTraits::CHUNK_SIZE,1, Eigen::Vector3d>;
template class CGOGN_IO_API VtkIO<DefaultMapTraits::CHUNK_SIZE,1, Eigen::Vector3f>;
template class CGOGN_IO_API VtkIO<DefaultMapTraits::CHUNK_SIZE,1, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_API VtkIO<DefaultMapTraits::CHUNK_SIZE,1, geometry::Vec_T<std::array<float32,3>>>;

template class CGOGN_IO_API VtkVolumeImport<DefaultMapTraits, Eigen::Vector3d>;
template class CGOGN_IO_API VtkVolumeImport<DefaultMapTraits, Eigen::Vector3f>;
template class CGOGN_IO_API VtkVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_API VtkVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float32,3>>>;

template class CGOGN_IO_API VtkVolumeExport<CMap3<DefaultMapTraits>>;
template class CGOGN_IO_API VtkSurfaceExport<CMap2<DefaultMapTraits>>;

CGOGN_IO_API std::string  vtk_data_type_to_cgogn_name_of_type(const std::string& vtk_type_str)
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

CGOGN_IO_API std::string cgogn_name_of_type_to_vtk_data_type(const std::string& cgogn_type)
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

	cgogn_log_error("cgogn_name_of_type_to_vtk_data_type") << "Unknown cgogn type \"" << cgogn_type << "\".";
	return std::string();
}

} // namespace io
} // namespace cgogn

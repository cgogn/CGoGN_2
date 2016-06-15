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

#ifndef CGOGN_IO_MAP_EXPORT_H_
#define CGOGN_IO_MAP_EXPORT_H_

#include <string>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <climits>

#include <cgogn/core/utils/logger.h>
#include <cgogn/core/utils/endian.h>
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/algos/normal.h>
#include <cgogn/geometry/algos/ear_triangulation.h>
#include <cgogn/io/vtk_io.h>
#include <cgogn/io/msh_io.h>
#include <cgogn/io/nastran_io.h>
#include <cgogn/io/tet_io.h>
#include <cgogn/io/off_io.h>
#include <cgogn/io/obj_io.h>
#include <cgogn/io/stl_io.h>
#include <cgogn/io/ply_io.h>

namespace cgogn
{

namespace io
{

template <typename MAP>
inline std::unique_ptr<SurfaceExport<MAP>> new_surface_export(const std::string& filename);

template <typename MAP>
inline std::unique_ptr<VolumeExport<MAP>> new_volume_export(const std::string& filename);

template <class MAP>
inline void export_surface(MAP& map2, const ExportOptions& options);

template <class MAP>
inline void export_volume(MAP& map3, const ExportOptions& options);

template <class MAP>
inline void export_surface(MAP& map2, const ExportOptions& options)
{
	static_assert(MAP::DIMENSION == 2,"export_surface is designed for 2D maps.");
	auto se = new_surface_export<MAP>(options.filename_);
	if (se)
		se->export_file(map2,options);
}

template <class MAP>
inline void export_volume(MAP& map3, const ExportOptions& options)
{
	static_assert(MAP::DIMENSION == 3,"export_volume is designed for 3D maps.");
	auto ve = new_volume_export<MAP>(options.filename_);
	if (ve)
		ve->export_file(map3,options);
}

template <typename MAP>
inline std::unique_ptr<SurfaceExport<MAP> > new_surface_export(const std::string& filename)
{
	const FileType ft = file_type(filename);
	switch (ft)
	{
		case FileType::FileType_OFF:		return make_unique<OffSurfaceExport<MAP>>();
		case FileType::FileType_OBJ:		return make_unique<ObjSurfaceExport<MAP>>();
		case FileType::FileType_STL:		return make_unique<StlSurfaceExport<MAP>>();
		case FileType::FileType_VTK_LEGACY:
		case FileType::FileType_VTP:		return make_unique<VtkSurfaceExport<MAP>>();
		case FileType::FileType_PLY:		return make_unique<PlySurfaceExport<MAP>>();
		default:
			cgogn_log_warning("new_surface_export") << "SurfaceExport does not handle files with extension \"" << extension(filename) << "\".";
			return std::unique_ptr<SurfaceExport<MAP>>();
	}
}

template <typename MAP>
inline std::unique_ptr<VolumeExport<MAP> > new_volume_export(const std::string& filename)
{
	const FileType ft = file_type(filename);
	switch (ft)
	{
		case FileType::FileType_VTK_LEGACY:
		case FileType::FileType_VTU:		return make_unique<VtkVolumeExport<MAP>>();
		case FileType::FileType_MSH:		return make_unique<MshVolumeExport<MAP>>();
		case FileType::FileType_NASTRAN:	return make_unique<NastranVolumeExport<MAP>>();
		case FileType::FileType_AIMATSHAPE:	return make_unique<TetVolumeExport<MAP>>();
		default:
			cgogn_log_warning("new_volume_export") << "VolumeExport does not handle files with extension \"" << extension(filename) << "\".";
			return std::unique_ptr<VolumeExport<MAP>>();
	}
}

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_MAP_EXPORT_H_

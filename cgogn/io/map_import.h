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

#ifndef IO_MAP_IMPORT_H_
#define IO_MAP_IMPORT_H_

#include <string>
#include <memory>

#include <core/cmap/cmap2.h>
#include <core/cmap/cmap3.h>

#include <io/surface_import.h>
#include <io/volume_import.h>
#include <io/vtk_io.h>
#include <io/off_io.h>
#include <io/obj_io.h>
#include <io/ply_io.h>
#include <io/lm6_io.h>

namespace cgogn
{

namespace io
{

template <typename MAP_TRAITS, typename VEC3>
inline std::unique_ptr<SurfaceImport<MAP_TRAITS>> newSurfaceImport(const std::string& filename);

template <typename MAP_TRAITS, typename VEC3>
inline std::unique_ptr<VolumeImport<MAP_TRAITS>> newVolumeImport(const std::string& filename);

template <typename VEC3, class MAP_TRAITS>
inline void import_surface(cgogn::CMap2<MAP_TRAITS>& cmap2, const std::string& filename);

template <typename VEC3, class MAP_TRAITS>
inline void import_volume(cgogn::CMap3<MAP_TRAITS>& cmap3, const std::string& filename);




template <typename VEC3, class MAP_TRAITS>
inline void import_surface(cgogn::CMap2<MAP_TRAITS>& cmap2, const std::string& filename)
{
	auto si = newSurfaceImport<MAP_TRAITS, VEC3>(filename);
	si->import_file(filename);
	si->create_map(cmap2);
}

template <typename VEC3, class MAP_TRAITS>
inline void import_volume(cgogn::CMap3<MAP_TRAITS>& cmap3, const std::string& filename)
{
	auto si = newVolumeImport<MAP_TRAITS, VEC3>(filename);
	si->import_file(filename);
	si->create_map(cmap3);
}

template <typename MAP_TRAITS, typename VEC3>
inline std::unique_ptr<SurfaceImport<MAP_TRAITS>> newSurfaceImport(const std::string& filename)
{
	const FileType file_type = get_file_type(filename);
	switch (file_type)
	{
		case FileType::FileType_OFF : return make_unique<OffSurfaceImport<MAP_TRAITS, VEC3>>();
		case FileType::FileType_VTK_LEGACY:
		case FileType::FileType_VTU:
		case FileType::FileType_VTP: return make_unique<VtkSurfaceImport<MAP_TRAITS, VEC3>>();
		case FileType::FileType_OBJ: return make_unique<ObjSurfaceImport<MAP_TRAITS, VEC3>>();
		case FileType::FileType_PLY: return make_unique<PlySurfaceImport<MAP_TRAITS, VEC3>>();
		default:
			std::cerr << "SurfaceImport does not handle files with extension \"" << get_extension(filename) << "\"." << std::endl;
			return std::unique_ptr<SurfaceImport<MAP_TRAITS>> ();
	}
}

template <typename MAP_TRAITS, typename VEC3>
inline std::unique_ptr<VolumeImport<MAP_TRAITS> > newVolumeImport(const std::string& filename)
{
	const FileType file_type = get_file_type(filename);
	switch (file_type)
	{
		case FileType::FileType_VTK_LEGACY:
		case FileType::FileType_VTU:	return make_unique<VtkVolumeImport<MAP_TRAITS, VEC3>>();
		case FileType::FileType_MESHB:	return make_unique<LM6VolumeImport<MAP_TRAITS, VEC3>>();
		default:
			std::cerr << "VolumeImport does not handle files with extension \"" << get_extension(filename) << "\"." << std::endl;
			return std::unique_ptr<VolumeImport<MAP_TRAITS>> ();
	}
}
} // namespace io

} // namespace cgogn

#endif // IO_MAP_IMPORT_H_

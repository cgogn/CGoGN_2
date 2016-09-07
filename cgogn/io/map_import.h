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

#ifndef CGOGN_IO_MAP_IMPORT_H_
#define CGOGN_IO_MAP_IMPORT_H_

#include <string>
#include <memory>

#include <cgogn/core/utils/logger.h>
#include <cgogn/core/cmap/cmap2.h>
#include <cgogn/core/cmap/cmap3.h>

#include <cgogn/io/surface_import.h>
#include <cgogn/io/volume_import.h>
#include <cgogn/io/vtk_io.h>
#include <cgogn/io/off_io.h>
#include <cgogn/io/obj_io.h>
#include <cgogn/io/ply_io.h>
#include <cgogn/io/lm6_io.h>
#include <cgogn/io/msh_io.h>
#include <cgogn/io/tetgen_io.h>
#include <cgogn/io/nastran_io.h>
#include <cgogn/io/tet_io.h>
#include <cgogn/io/stl_io.h>

namespace cgogn
{

namespace io
{

template <typename MAP_TRAITS, typename VEC3>
inline std::unique_ptr<SurfaceFileImport<MAP_TRAITS>> newSurfaceImport(const std::string& filename);

template <typename MAP_TRAITS, typename VEC3>
inline std::unique_ptr<VolumeFileImport<MAP_TRAITS>> newVolumeImport(const std::string& filename);

template <typename VEC3, typename MAP2>
inline void import_surface(MAP2& cmap2, const std::string& filename);

template <typename VEC3, typename MAP3>
inline void import_volume(MAP3& cmap3, const std::string& filename);




template <typename VEC3, typename MAP2>
inline void import_surface(MAP2& cmap2, const std::string& filename)
{
	auto si = newSurfaceImport<typename MAP2::Traits, VEC3>(filename);
	if (si)
	{
		if (si->import_file(filename))
			si->create_map(cmap2);
	}
}

template <typename VEC3, typename MAP3>
inline void import_volume(MAP3& cmap3, const std::string& filename)
{
	auto si = newVolumeImport<typename MAP3::Traits, VEC3>(filename);
	if (si)
	{
		if (si->import_file(filename))
			si->create_map(cmap3);
	}
}

template <typename MAP_TRAITS, typename VEC3>
inline std::unique_ptr<SurfaceFileImport<MAP_TRAITS>> newSurfaceImport(const std::string& filename)
{
	const FileType ft = file_type(filename);
	switch (ft)
	{
		case FileType::FileType_OFF : return make_unique<OffSurfaceImport<MAP_TRAITS, VEC3>>();
		case FileType::FileType_VTK_LEGACY:
		case FileType::FileType_VTU:
		case FileType::FileType_VTP: return make_unique<VtkSurfaceImport<MAP_TRAITS, VEC3>>();
		case FileType::FileType_OBJ: return make_unique<ObjSurfaceImport<MAP_TRAITS, VEC3>>();
		case FileType::FileType_PLY: return make_unique<PlySurfaceImport<MAP_TRAITS, VEC3>>();
		case FileType::FileType_STL: return make_unique<StlSurfaceImport<MAP_TRAITS, VEC3>>();
		default:
			cgogn_log_warning("newSurfaceImport") << "SurfaceImport does not handle files with extension \"" << extension(filename) << "\".";
			return std::unique_ptr<SurfaceFileImport<MAP_TRAITS>> ();
	}
}

template <typename MAP_TRAITS, typename VEC3>
inline std::unique_ptr<VolumeFileImport<MAP_TRAITS> > newVolumeImport(const std::string& filename)
{
	const FileType ft = file_type(filename);
	switch (ft)
	{
		case FileType::FileType_VTK_LEGACY:
		case FileType::FileType_VTU:		return make_unique<VtkVolumeImport<MAP_TRAITS, VEC3>>();
		case FileType::FileType_MESHB:		return make_unique<LM6VolumeImport<MAP_TRAITS, VEC3>>();
		case FileType::FileType_MSH:		return make_unique<MshVolumeImport<MAP_TRAITS, VEC3>>();
		case FileType::FileType_TETGEN:		return make_unique<TetgenVolumeImport<MAP_TRAITS, VEC3>>();
		case FileType::FileType_NASTRAN:	return make_unique<NastranVolumeImport<MAP_TRAITS, VEC3>>();
		case FileType::FileType_AIMATSHAPE:	return make_unique<TetVolumeImport<MAP_TRAITS, VEC3>>();
		default:
			cgogn_log_warning("VolumeImport") << "VolumeImport does not handle files with extension \"" << extension(filename) << "\".";
			return std::unique_ptr<VolumeFileImport<MAP_TRAITS>> ();
	}
}

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_MAP_IMPORT_H_

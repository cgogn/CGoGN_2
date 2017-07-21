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
#include <cgogn/io/2dm_io.h>
#include <cgogn/io/ply_io.h>
#include <cgogn/io/lm6_io.h>
#include <cgogn/io/msh_io.h>
#include <cgogn/io/tetgen_io.h>
#include <cgogn/io/nastran_io.h>
#include <cgogn/io/tet_io.h>
#include <cgogn/io/stl_io.h>
#include <cgogn/io/tetmesh_io.h>

namespace cgogn
{

namespace io
{

template <typename VEC3, typename MAP>
inline std::unique_ptr<SurfaceFileImport<MAP, VEC3>> newSurfaceImport(MAP& map, const std::string& filename);

template <typename VEC3, typename MAP>
inline std::unique_ptr<VolumeFileImport<MAP, VEC3>> newVolumeImport(MAP& map, const std::string& filename);


template <typename VEC3, typename MAP>
inline void import_surface(MAP& map, const std::string& filename)
{
	auto si = newSurfaceImport<VEC3>(map, filename);
	if (si)
	{
		if (si->import_file(filename))
			si->create_map();
	}
}

template <typename VEC3, typename MAP>
inline void import_volume(MAP& map, const std::string& filename)
{
	auto si = newVolumeImport<VEC3>(map, filename);
	if (si)
	{
		if (si->import_file(filename))
			si->create_map();
	}
}

template <typename VEC3, typename MAP>
inline std::unique_ptr<SurfaceFileImport<MAP, VEC3>> newSurfaceImport(MAP& map, const std::string& filename)
{
	const FileType ft = file_type(filename);
	switch (ft)
	{
		case FileType::FileType_OFF : return make_unique<OffSurfaceImport<MAP, VEC3>>(map);
		case FileType::FileType_VTK_LEGACY:
		case FileType::FileType_VTU:
		case FileType::FileType_VTP: return make_unique<VtkSurfaceImport<MAP, VEC3>>(map);
		case FileType::FileType_OBJ: return make_unique<ObjSurfaceImport<MAP, VEC3>>(map);
		case FileType::FileType_2DM: return make_unique<SMS2DMSurfaceImport<MAP, VEC3>>(map);
		case FileType::FileType_PLY: return make_unique<PlySurfaceImport<MAP, VEC3>>(map);
		case FileType::FileType_STL: return make_unique<StlSurfaceImport<MAP, VEC3>>(map);
		case FileType::FileType_MSH: return make_unique<MshSurfaceImport<MAP, VEC3>>(map);
		case FileType::FileType_MESHB: return make_unique<LM6SurfaceImport<MAP, VEC3>>(map);
		default:
			cgogn_log_warning("newSurfaceImport") << "SurfaceImport does not handle files with extension \"" << extension(filename) << "\".";
			return std::unique_ptr<SurfaceFileImport<MAP, VEC3>>();
	}
}

template <typename VEC3, typename MAP>
inline std::unique_ptr<VolumeFileImport<MAP, VEC3>> newVolumeImport(MAP& map, const std::string& filename)
{
	const FileType ft = file_type(filename);
	switch (ft)
	{
		case FileType::FileType_VTK_LEGACY:
		case FileType::FileType_VTU:		return make_unique<VtkVolumeImport<MAP, VEC3>>(map);
		case FileType::FileType_MESHB:		return make_unique<LM6VolumeImport<MAP, VEC3>>(map);
		case FileType::FileType_MSH:		return make_unique<MshVolumeImport<MAP, VEC3>>(map);
		case FileType::FileType_TETGEN:		return make_unique<TetgenVolumeImport<MAP, VEC3>>(map);
		case FileType::FileType_NASTRAN:	return make_unique<NastranVolumeImport<MAP, VEC3>>(map);
		case FileType::FileType_AIMATSHAPE:	return make_unique<TetVolumeImport<MAP, VEC3>>(map);
		case FileType::FileType_TETMESH:	return make_unique<TetMeshVolumeImport<MAP, VEC3>>(map);
		default:
			cgogn_log_warning("VolumeImport") << "VolumeImport does not handle files with extension \"" << extension(filename) << "\".";
			return std::unique_ptr<VolumeFileImport<MAP, VEC3>>();
	}
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_MAP_IMPORT_CPP_))
extern template CGOGN_IO_API void import_surface<Eigen::Vector3f>(CMap2&, const std::string&);
extern template CGOGN_IO_API void import_surface<Eigen::Vector3d>(CMap2&, const std::string&);
extern template CGOGN_IO_API void import_volume<Eigen::Vector3f>(CMap3&, const std::string&);
extern template CGOGN_IO_API void import_volume<Eigen::Vector3d>(CMap3&, const std::string&);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_MAP_IMPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_MAP_IMPORT_H_

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

#include <cgogn/io/formats/vtk.h>
#include <cgogn/io/formats/off.h>
#include <cgogn/io/formats/obj.h>
#include <cgogn/io/formats/stl.h>
#include <cgogn/io/formats/ply.h>
#include <cgogn/io/formats/cg.h>
#include <cgogn/io/formats/cskel.h>
#include <cgogn/io/formats/skel.h>
#include <cgogn/io/formats/tet.h>
#include <cgogn/io/formats/msh.h>
#include <cgogn/io/formats/nastran.h>
#include <cgogn/io/formats/tetmesh.h>

namespace cgogn
{

namespace io
{

template <typename MAP>
inline std::unique_ptr<GraphExport<MAP>> new_graph_export(const std::string& filename)
{
	const FileType ft = file_type(filename);
	switch(ft) {
		case FileType::FileType_SKEL:		return make_unique<SkelGraphExport<MAP>>();
		case FileType::FileType_VTK_LEGACY:	return make_unique<VtkGraphExport<MAP>>();
		case FileType::FileType_CG:			return make_unique<CgGraphExport<MAP>>();
		case FileType::FileType_CSKEL:		return make_unique<CskelGraphExport<MAP>>();
		case FileType::FileType_OBJ:		return make_unique<ObjGraphExport<MAP>>();
		default:
			cgogn_log_warning("new_graph_export") << "GraphExport does not handle files with extension \"" << extension(filename) << "\".";
			return std::unique_ptr<GraphExport<MAP>>();
	}
}

template <typename MAP>
inline std::unique_ptr<SurfaceExport<MAP>> new_surface_export(const std::string& filename)
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
		case FileType::FileType_MSH:		return make_unique<MshSurfaceExport<MAP>>();
		default:
			cgogn_log_warning("new_surface_export") << "SurfaceExport does not handle files with extension \"" << extension(filename) << "\".";
			return std::unique_ptr<SurfaceExport<MAP>>();
	}
}

template <typename MAP>
inline std::unique_ptr<VolumeExport<MAP>> new_volume_export(const std::string& filename)
{
	const FileType ft = file_type(filename);
	switch (ft)
	{
		case FileType::FileType_VTK_LEGACY:
		case FileType::FileType_VTU:		return make_unique<VtkVolumeExport<MAP>>();
		case FileType::FileType_MSH:		return make_unique<MshVolumeExport<MAP>>();
		case FileType::FileType_NASTRAN:	return make_unique<NastranVolumeExport<MAP>>();
		case FileType::FileType_AIMATSHAPE:	return make_unique<TetVolumeExport<MAP>>();
		case FileType::FileType_TETMESH:	return make_unique<TetMeshVolumeExport<MAP>>();
		default:
			cgogn_log_warning("new_volume_export") << "VolumeExport does not handle files with extension \"" << extension(filename) << "\".";
			return std::unique_ptr<VolumeExport<MAP>>();
	}
}

template <typename MAP>
inline void export_graph(MAP& map, const ExportOptions& options)
{
	auto se = new_graph_export<MAP>(options.filename_);
	if (se)
		se->export_file(map, options);
}

template <class MAP>
inline void export_surface(MAP& map2, const ExportOptions& options)
{
	static_assert(MAP::DIMENSION == 2, "export_surface is designed for 2D maps.");
	auto se = new_surface_export<MAP>(options.filename_);
	if (se)
		se->export_file(map2, options);
}

template <class MAP>
inline void export_volume(MAP& map3, const ExportOptions& options)
{
	static_assert(MAP::DIMENSION == 3, "export_volume is designed for 3D maps.");
	auto ve = new_volume_export<MAP>(options.filename_);
	if (ve)
		ve->export_file(map3, options);
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_MAP_EXPORT_CPP_))
extern template CGOGN_IO_API void export_graph(UndirectedGraph& , const ExportOptions&);
extern template CGOGN_IO_API void export_surface(CMap2& , const ExportOptions&);
extern template CGOGN_IO_API void export_volume(CMap3& , const ExportOptions&);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_MAP_EXPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_MAP_EXPORT_H_

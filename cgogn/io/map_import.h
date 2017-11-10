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
#include <cgogn/core/graph/undirected_graph.h>

#include <cgogn/io/surface_import.h>
#include <cgogn/io/volume_import.h>
#include <cgogn/io/graph_import.h>

#include <cgogn/io/formats/cg.h>
#include <cgogn/io/formats/cskel.h>
#include <cgogn/io/formats/dot.h>
#include <cgogn/io/formats/off.h>
#include <cgogn/io/formats/obj.h>
#include <cgogn/io/formats/2dm.h>
#include <cgogn/io/formats/ply.h>
#include <cgogn/io/formats/lm6.h>
#include <cgogn/io/formats/msh.h>
#include <cgogn/io/formats/tetgen.h>
#include <cgogn/io/formats/nastran.h>
#include <cgogn/io/formats/tet.h>
#include <cgogn/io/formats/skel.h>
#include <cgogn/io/formats/stl.h>
#include <cgogn/io/formats/tetmesh.h>
#include <cgogn/io/formats/vtk.h>

namespace cgogn
{

namespace io
{

template <typename VEC3>
inline std::unique_ptr<GraphFileImport> new_graph_import(const std::string& filename)
{
	const FileType ft = file_type(filename);
	switch (ft)
	{
		case FileType::FileType_SKEL:		return make_unique<SkelGraphImport<VEC3>>();
		case FileType::FileType_VTK_LEGACY:	return make_unique<VtkGraphImport<VEC3>>();
		case FileType::FileType_CG:			return make_unique<CgGraphImport<VEC3>>();
		case FileType::FileType_CSKEL:		return make_unique<CskelGraphImport<VEC3>>();
		case FileType::FileType_DOT:		return make_unique<DotGraphImport<VEC3>>();
		case FileType::FileType_OBJ:		return make_unique<ObjGraphImport<VEC3>>();
		default:
			cgogn_log_warning("GraphImport") << "GraphImport does not handle files with extension \"" << extension(filename) << "\".";
			return std::unique_ptr<GraphFileImport> ();
	}
}

template <typename VEC3, typename MAP>
inline std::unique_ptr<SurfaceFileImport<MAP>> new_surface_import(MAP& map, const std::string& filename)
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
			cgogn_log_warning("SurfaceImport") << "SurfaceImport does not handle files with extension \"" << extension(filename) << "\".";
			return std::unique_ptr<SurfaceFileImport<MAP>>();
	}
}

template <typename VEC3, typename MAP>
inline std::unique_ptr<VolumeFileImport<MAP>> new_volume_import(MAP& map, const std::string& filename)
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
			return std::unique_ptr<VolumeFileImport<MAP>>();
	}
}

template <typename VEC3, typename MAP>
inline void import_graph(MAP& map, const std::string& filename)
{
	auto si = new_graph_import<VEC3>(filename);

	if(si)
		if(si->import_file(filename))
			si->create(map);
}

template <typename VEC3, typename MAP>
inline void import_surface(MAP& map, const std::string& filename)
{
	auto si = new_surface_import<VEC3>(map, filename);
	if (si)
		if (si->import_file(filename))
			si->create_map();
}

template <typename VEC3, typename MAP>
inline void import_volume(MAP& map, const std::string& filename)
{
	auto si = new_volume_import<VEC3>(map, filename);
	if (si)
		if (si->import_file(filename))
			si->create_map();
}


#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_MAP_IMPORT_CPP_))
extern template CGOGN_IO_API void import_graph<Eigen::Vector3f>(UndirectedGraph&, const std::string&);
extern template CGOGN_IO_API void import_graph<Eigen::Vector3d>(UndirectedGraph&, const std::string&);
extern template CGOGN_IO_API void import_surface<Eigen::Vector3f>(CMap2&, const std::string&);
extern template CGOGN_IO_API void import_surface<Eigen::Vector3d>(CMap2&, const std::string&);
extern template CGOGN_IO_API void import_volume<Eigen::Vector3f>(CMap3&, const std::string&);
extern template CGOGN_IO_API void import_volume<Eigen::Vector3d>(CMap3&, const std::string&);
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_MAP_IMPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_MAP_IMPORT_H_

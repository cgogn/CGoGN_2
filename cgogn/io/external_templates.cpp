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
#define CGOGN_IO_EXTERNAL_TEMPLATES_CPP_


#include <cgogn/io/map_export.h>
#include <cgogn/io/map_import.h>
#include <cgogn/io/surface_export.h>
#include <cgogn/io/surface_import.h>
#include <cgogn/io/volume_export.h>
#include <cgogn/io/volume_import.h>

namespace cgogn
{
namespace io
{

template class CGOGN_IO_API GraphExport<UndirectedGraph>;

template CGOGN_IO_API void export_graph(UndirectedGraph& , const ExportOptions&);
template CGOGN_IO_API void export_surface(CMap2& , const ExportOptions&);
template CGOGN_IO_API void export_volume(CMap3& , const ExportOptions&);

template CGOGN_IO_API void import_graph<Eigen::Vector3f>(UndirectedGraph&, const std::string&);
template CGOGN_IO_API void import_graph<Eigen::Vector3d>(UndirectedGraph&, const std::string&);
template CGOGN_IO_API void import_surface<Eigen::Vector3f>(CMap2&, const std::string&);
template CGOGN_IO_API void import_surface<Eigen::Vector3d>(CMap2&, const std::string&);
template CGOGN_IO_API void import_volume<Eigen::Vector3f>(CMap3&, const std::string&);
template CGOGN_IO_API void import_volume<Eigen::Vector3d>(CMap3&, const std::string&);




template class CGOGN_IO_API SurfaceExport<CMap2>;

template class CGOGN_IO_API SurfaceImport<CMap2>;
template class CGOGN_IO_API SurfaceFileImport<CMap2>;

template class CGOGN_IO_API VolumeExport<CMap3>;

template class CGOGN_IO_API VolumeImport<CMap3>;
template class CGOGN_IO_API VolumeFileImport<CMap3>;

/// FORMAT

template class CGOGN_IO_API MshIO<Eigen::Vector3d>;
template class CGOGN_IO_API MshIO<Eigen::Vector3f>;
template class CGOGN_IO_API MshIO<geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_API MshIO<geometry::Vec_T<std::array<float32,3>>>;

template class CGOGN_IO_API MshSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_API MshSurfaceImport<CMap2, Eigen::Vector3f>;

template class CGOGN_IO_API MshVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_API MshVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_API MshVolumeImport<CMap3, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_API MshVolumeImport<CMap3, geometry::Vec_T<std::array<float32,3>>>;

template class CGOGN_IO_API MshSurfaceExport<CMap2>;
template class CGOGN_IO_API MshVolumeExport<CMap3>;

template class CGOGN_IO_API SMS2DMSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_API SMS2DMSurfaceImport<CMap2, Eigen::Vector3f>;
template class CGOGN_IO_API SMS2DMSurfaceImport<CMap2, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_API SMS2DMSurfaceImport<CMap2, geometry::Vec_T<std::array<float32,3>>>;

template class CGOGN_IO_API CgGraphImport<Eigen::Vector3d>;
template class CGOGN_IO_API CgGraphImport<Eigen::Vector3f>;
template class CGOGN_IO_API CgGraphImport<geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_API CgGraphImport<geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_API CgGraphExport<UndirectedGraph>;

template class CGOGN_IO_API CskelGraphImport<Eigen::Vector3d>;
template class CGOGN_IO_API CskelGraphImport<Eigen::Vector3f>;
template class CGOGN_IO_API CskelGraphImport<geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_API CskelGraphImport<geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_API CskelGraphExport<UndirectedGraph>;

template class CGOGN_IO_API DotGraphImport<Eigen::Vector3d>;
template class CGOGN_IO_API DotGraphImport<Eigen::Vector3f>;
template class CGOGN_IO_API DotGraphImport<geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_API DotGraphImport<geometry::Vec_T<std::array<float32,3>>>;

template class CGOGN_IO_API MeshbVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_API MeshbVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_API MeshbVolumeImport<CMap3, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_API MeshbVolumeImport<CMap3, geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_API MeshbSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_API MeshbSurfaceImport<CMap2, Eigen::Vector3f>;

template class CGOGN_IO_API NastranIO<Eigen::Vector3d>;
template class CGOGN_IO_API NastranIO<Eigen::Vector3f>;
template class CGOGN_IO_API NastranIO<geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_API NastranIO<geometry::Vec_T<std::array<float32, 3>>>;
template class CGOGN_IO_API NastranVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_API NastranVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_API NastranVolumeImport<CMap3, geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_API NastranVolumeImport<CMap3, geometry::Vec_T<std::array<float32, 3>>>;
template class CGOGN_IO_API NastranVolumeExport<CMap3>;

template class CGOGN_IO_API ObjSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_API ObjSurfaceImport<CMap2, Eigen::Vector3f>;
template class CGOGN_IO_API ObjSurfaceImport<CMap2, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_API ObjSurfaceImport<CMap2, geometry::Vec_T<std::array<float32,3>>>;

template class CGOGN_IO_API ObjGraphImport<Eigen::Vector3d>;
template class CGOGN_IO_API ObjGraphImport<Eigen::Vector3f>;
template class CGOGN_IO_API ObjGraphImport<geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_API ObjGraphImport<geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_API ObjSurfaceExport<CMap2>;
template class CGOGN_IO_API ObjGraphExport<UndirectedGraph>;

template class CGOGN_IO_API OffSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_API OffSurfaceImport<CMap2, Eigen::Vector3f>;
template class CGOGN_IO_API OffSurfaceImport<CMap2, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_API OffSurfaceImport<CMap2, geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_API OffSurfaceExport<CMap2>;

template class CGOGN_IO_API PlySurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_API PlySurfaceImport<CMap2, Eigen::Vector3f>;
template class CGOGN_IO_API PlySurfaceImport<CMap2, geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_API PlySurfaceImport<CMap2, geometry::Vec_T<std::array<float32, 3>>>;
template class CGOGN_IO_API PlySurfaceExport<CMap2>;

template class CGOGN_IO_API SkelGraphImport<Eigen::Vector3d>;
template class CGOGN_IO_API SkelGraphImport<Eigen::Vector3f>;
template class CGOGN_IO_API SkelGraphImport<geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_API SkelGraphImport<geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_API SkelGraphExport<UndirectedGraph>;

template class CGOGN_IO_API StlSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_API StlSurfaceImport<CMap2, Eigen::Vector3f>;
template class CGOGN_IO_API StlSurfaceImport<CMap2, geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_API StlSurfaceImport<CMap2, geometry::Vec_T<std::array<float32, 3>>>;
template class CGOGN_IO_API StlSurfaceExport<CMap2>;

template class CGOGN_IO_API TetVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_API TetVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_API TetVolumeImport<CMap3, geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_API TetVolumeImport<CMap3, geometry::Vec_T<std::array<float32, 3>>>;
template class CGOGN_IO_API TetVolumeExport<CMap3>;

template class CGOGN_IO_API TetgenVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_API TetgenVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_API TetgenVolumeImport<CMap3, geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_API TetgenVolumeImport<CMap3, geometry::Vec_T<std::array<float32, 3>>>;

template class CGOGN_IO_API TetMeshVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_API TetMeshVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_API TetMeshVolumeImport<CMap3, geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_API TetMeshVolumeImport<CMap3, geometry::Vec_T<std::array<float32, 3>>>;
template class CGOGN_IO_API TetMeshVolumeExport<CMap3>;

template class CGOGN_IO_API VtkIO<1, Eigen::Vector3d>;
template class CGOGN_IO_API VtkIO<1, Eigen::Vector3f>;
template class CGOGN_IO_API VtkSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_API VtkSurfaceImport<CMap2, Eigen::Vector3f>;
template class CGOGN_IO_API VtkVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_API VtkVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_API VtkVolumeExport<CMap3>;
template class CGOGN_IO_API VtkSurfaceExport<CMap2>;
template class CGOGN_IO_API VtkGraphImport<Eigen::Vector3d>;
template class CGOGN_IO_API VtkGraphImport<Eigen::Vector3f>;
template class CGOGN_IO_API VtkGraphExport<UndirectedGraph>;

} // namespace io

} // namespace cgogn




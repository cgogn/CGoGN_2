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

#include <cgogn/io/formats/msh.h>
#include <cgogn/io/formats/cg.h>
#include <cgogn/io/formats/cskel.h>
#include <cgogn/io/formats/dot.h>
#include <cgogn/io/formats/lin.h>
#include <cgogn/io/formats/off.h>
#include <cgogn/io/formats/obj.h>
#include <cgogn/io/formats/2dm.h>
#include <cgogn/io/formats/plo.h>
#include <cgogn/io/formats/ply.h>
#include <cgogn/io/formats/meshb.h>
#include <cgogn/io/formats/tetgen.h>
#include <cgogn/io/formats/nastran.h>
#include <cgogn/io/formats/tet.h>
#include <cgogn/io/formats/skel.h>
#include <cgogn/io/formats/stl.h>
#include <cgogn/io/formats/tetmesh.h>
#include <cgogn/io/formats/ts.h>
#include <cgogn/io/formats/vtk.h>
#include <cgogn/io/surface_import.h>
#include <cgogn/io/volume_import.h>

namespace cgogn
{
namespace io
{

template class CGOGN_IO_EXPORT PointSetImport<CMap0>;
template class CGOGN_IO_EXPORT PointSetFileImport<CMap0>;
template class CGOGN_IO_EXPORT SurfaceImport<CMap2>;
template class CGOGN_IO_EXPORT VolumeImport<CMap3>;
template class CGOGN_IO_EXPORT VolumeFileImport<CMap3>;
template class CGOGN_IO_EXPORT SurfaceFileImport<CMap2>;
template class CGOGN_IO_EXPORT PolylineImport<CMap1>;
template class CGOGN_IO_EXPORT PolylineFileImport<CMap1>;

template class CGOGN_IO_EXPORT MshIO<Eigen::Vector3f>;
template class CGOGN_IO_EXPORT MshIO<Eigen::Vector3d>;
template class CGOGN_IO_EXPORT MshIO<geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_EXPORT MshIO<geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_EXPORT MshSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT MshSurfaceImport<CMap2, Eigen::Vector3f>;

template class CGOGN_IO_EXPORT MshVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT MshVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT MshVolumeImport<CMap3, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_EXPORT MshVolumeImport<CMap3, geometry::Vec_T<std::array<float32,3>>>;

template class CGOGN_IO_EXPORT MshSurfaceExport<CMap2>;
template class CGOGN_IO_EXPORT MshVolumeExport<CMap3>;

template class CGOGN_IO_EXPORT SMS2DMSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT SMS2DMSurfaceImport<CMap2, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT SMS2DMSurfaceImport<CMap2, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_EXPORT SMS2DMSurfaceImport<CMap2, geometry::Vec_T<std::array<float32,3>>>;

template class CGOGN_IO_EXPORT CgGraphImport<Eigen::Vector3d>;
template class CGOGN_IO_EXPORT CgGraphImport<Eigen::Vector3f>;
template class CGOGN_IO_EXPORT CgGraphImport<geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_EXPORT CgGraphImport<geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_EXPORT CgGraphExport<UndirectedGraph>;

template class CGOGN_IO_EXPORT CskelGraphImport<Eigen::Vector3d>;
template class CGOGN_IO_EXPORT CskelGraphImport<Eigen::Vector3f>;
template class CGOGN_IO_EXPORT CskelGraphImport<geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_EXPORT CskelGraphImport<geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_EXPORT CskelGraphExport<UndirectedGraph>;

template class CGOGN_IO_EXPORT DotGraphImport<Eigen::Vector3d>;
template class CGOGN_IO_EXPORT DotGraphImport<Eigen::Vector3f>;
template class CGOGN_IO_EXPORT DotGraphImport<geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_EXPORT DotGraphImport<geometry::Vec_T<std::array<float32,3>>>;

template class CGOGN_IO_EXPORT MeshbVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT MeshbVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT MeshbVolumeImport<CMap3, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_EXPORT MeshbVolumeImport<CMap3, geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_EXPORT MeshbSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT MeshbSurfaceImport<CMap2, Eigen::Vector3f>;

template class CGOGN_IO_EXPORT NastranIO<Eigen::Vector3d>;
template class CGOGN_IO_EXPORT NastranIO<Eigen::Vector3f>;
template class CGOGN_IO_EXPORT NastranIO<geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_EXPORT NastranIO<geometry::Vec_T<std::array<float32, 3>>>;
template class CGOGN_IO_EXPORT NastranVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT NastranVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT NastranVolumeImport<CMap3, geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_EXPORT NastranVolumeImport<CMap3, geometry::Vec_T<std::array<float32, 3>>>;
template class CGOGN_IO_EXPORT NastranVolumeExport<CMap3>;

template class CGOGN_IO_EXPORT ObjSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT ObjSurfaceImport<CMap2, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT ObjSurfaceImport<CMap2, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_EXPORT ObjSurfaceImport<CMap2, geometry::Vec_T<std::array<float32,3>>>;

template class CGOGN_IO_EXPORT ObjGraphImport<Eigen::Vector3d>;
template class CGOGN_IO_EXPORT ObjGraphImport<Eigen::Vector3f>;
template class CGOGN_IO_EXPORT ObjGraphImport<geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_EXPORT ObjGraphImport<geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_EXPORT ObjSurfaceExport<CMap2>;
template class CGOGN_IO_EXPORT ObjGraphExport<UndirectedGraph>;

template class CGOGN_IO_EXPORT OffSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT OffSurfaceImport<CMap2, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT OffSurfaceImport<CMap2, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_EXPORT OffSurfaceImport<CMap2, geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_EXPORT OffSurfaceExport<CMap2>;

template class CGOGN_IO_EXPORT PloPointSetImport<CMap0, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT PloPointSetImport<CMap0, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT PloPointSetImport<CMap0, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_EXPORT PloPointSetImport<CMap0, geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_EXPORT PloPointSetExport<CMap0>;

template class CGOGN_IO_EXPORT PlySurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT PlySurfaceImport<CMap2, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT PlySurfaceImport<CMap2, geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_EXPORT PlySurfaceImport<CMap2, geometry::Vec_T<std::array<float32, 3>>>;
template class CGOGN_IO_EXPORT PlySurfaceExport<CMap2>;

template class CGOGN_IO_EXPORT SkelGraphImport<Eigen::Vector3d>;
template class CGOGN_IO_EXPORT SkelGraphImport<Eigen::Vector3f>;
template class CGOGN_IO_EXPORT SkelGraphImport<geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_EXPORT SkelGraphImport<geometry::Vec_T<std::array<float32,3>>>;
template class CGOGN_IO_EXPORT SkelGraphExport<UndirectedGraph>;

template class CGOGN_IO_EXPORT StlSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT StlSurfaceImport<CMap2, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT StlSurfaceImport<CMap2, geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_EXPORT StlSurfaceImport<CMap2, geometry::Vec_T<std::array<float32, 3>>>;
template class CGOGN_IO_EXPORT StlSurfaceExport<CMap2>;

template class CGOGN_IO_EXPORT TetVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT TetVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT TetVolumeImport<CMap3, geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_EXPORT TetVolumeImport<CMap3, geometry::Vec_T<std::array<float32, 3>>>;
template class CGOGN_IO_EXPORT TetVolumeExport<CMap3>;

template class CGOGN_IO_EXPORT TetgenVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT TetgenVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT TetgenVolumeImport<CMap3, geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_EXPORT TetgenVolumeImport<CMap3, geometry::Vec_T<std::array<float32, 3>>>;

template class CGOGN_IO_EXPORT TetMeshVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT TetMeshVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT TetMeshVolumeImport<CMap3, geometry::Vec_T<std::array<float64, 3>>>;
template class CGOGN_IO_EXPORT TetMeshVolumeImport<CMap3, geometry::Vec_T<std::array<float32, 3>>>;
template class CGOGN_IO_EXPORT TetMeshVolumeExport<CMap3>;

template class CGOGN_IO_EXPORT VtkIO<1, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT VtkIO<1, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT VtkSurfaceImport<CMap2, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT VtkSurfaceImport<CMap2, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT VtkVolumeImport<CMap3, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT VtkVolumeImport<CMap3, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT VtkVolumeExport<CMap3>;
template class CGOGN_IO_EXPORT VtkSurfaceExport<CMap2>;
template class CGOGN_IO_EXPORT VtkGraphImport<Eigen::Vector3d>;
template class CGOGN_IO_EXPORT VtkGraphImport<Eigen::Vector3f>;
template class CGOGN_IO_EXPORT VtkGraphExport<UndirectedGraph>;

template class CGOGN_IO_EXPORT LinPolylineImport<CMap1, Eigen::Vector3d>;
template class CGOGN_IO_EXPORT LinPolylineImport<CMap1, Eigen::Vector3f>;
template class CGOGN_IO_EXPORT LinPolylineImport<CMap1, geometry::Vec_T<std::array<float64,3>>>;
template class CGOGN_IO_EXPORT LinPolylineImport<CMap1, geometry::Vec_T<std::array<float32,3>>>;

} // namespace io
} // namespace cgogn

#include <cgogn/io/map_import.h>

namespace cgogn
{
namespace io
{

template CGOGN_IO_EXPORT void import_point_set<Eigen::Vector3f>(CMap0&, const std::string&);
template CGOGN_IO_EXPORT void import_point_set<Eigen::Vector3d>(CMap0&, const std::string&);
template CGOGN_IO_EXPORT void import_polyline<Eigen::Vector3f>(CMap1&, const std::string&);
template CGOGN_IO_EXPORT void import_polyline<Eigen::Vector3d>(CMap1&, const std::string&);
template CGOGN_IO_EXPORT void import_graph<Eigen::Vector3f>(UndirectedGraph&, const std::string&);
template CGOGN_IO_EXPORT void import_graph<Eigen::Vector3d>(UndirectedGraph&, const std::string&);
template CGOGN_IO_EXPORT void import_surface<Eigen::Vector3f>(CMap2&, const std::string&);
template CGOGN_IO_EXPORT void import_surface<Eigen::Vector3d>(CMap2&, const std::string&);
template CGOGN_IO_EXPORT void import_volume<Eigen::Vector3f>(CMap3&, const std::string&);
template CGOGN_IO_EXPORT void import_volume<Eigen::Vector3d>(CMap3&, const std::string&);

} // namespace io
} // namespace cgogn


#include <cgogn/io/map_export.h>

namespace cgogn
{
namespace io
{

template class CGOGN_IO_EXPORT PointSetExport<CMap0>;
template class CGOGN_IO_EXPORT SurfaceExport<CMap2>;
template class CGOGN_IO_EXPORT VolumeExport<CMap3>;
template class CGOGN_IO_EXPORT GraphExport<UndirectedGraph>;
template CGOGN_IO_EXPORT void export_point_set(CMap0& , const ExportOptions&);
template CGOGN_IO_EXPORT void export_polyline(CMap1& , const ExportOptions&);
template CGOGN_IO_EXPORT void export_graph(UndirectedGraph& , const ExportOptions&);
template CGOGN_IO_EXPORT void export_surface(CMap2& , const ExportOptions&);
template CGOGN_IO_EXPORT void export_volume(CMap3& , const ExportOptions&);

} // namespace io
} // namespace cgogn

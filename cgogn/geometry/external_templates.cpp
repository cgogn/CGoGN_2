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
#define CGOGN_GEOMETRY_EXTERNAL_TEMPLATES_CPP_


#include <cgogn/geometry/types/eigen.h>
#include <cgogn/geometry/types/vec.h>

namespace cgogn
{
namespace geometry
{

template class CGOGN_GEOMETRY_EXPORT Vec_T<std::array<float32,3>>;
template class CGOGN_GEOMETRY_EXPORT Vec_T<std::array<float64,3>>;

} // namespace geometry
} // namespace cgogn

#include <cgogn/geometry/types/obb.h>

namespace cgogn
{
namespace geometry
{

template class CGOGN_GEOMETRY_EXPORT OBB<Eigen::Vector3d>;
template class CGOGN_GEOMETRY_EXPORT OBB<Eigen::Vector3f>;

} // namespace geometry
} // namespace cgogn

#include <cgogn/geometry/types/aabb.h>

namespace cgogn
{
namespace geometry
{

template class CGOGN_GEOMETRY_EXPORT AABB<Eigen::Vector3d>;
template class CGOGN_GEOMETRY_EXPORT AABB<Eigen::Vector3f>;
template class CGOGN_GEOMETRY_EXPORT AABB<Vec_T<std::array<float32,3>>>;
template class CGOGN_GEOMETRY_EXPORT AABB<Vec_T<std::array<float64,3>>>;

} // namespace geometry
} // namespace cgogn

#include <cgogn/geometry/algos/angle.h>

namespace cgogn
{
namespace geometry
{

template CGOGN_GEOMETRY_EXPORT float32 angle_between_face_normals(const CMap2&, const Cell<Orbit::PHI2>,const CMap2::VertexAttribute<Eigen::Vector3f>&);
template CGOGN_GEOMETRY_EXPORT float64 angle_between_face_normals(const CMap2&, const Cell<Orbit::PHI2>,const CMap2::VertexAttribute<Eigen::Vector3d>&);
template CGOGN_GEOMETRY_EXPORT void compute_angle_between_face_normals(const CMap2&, const CMap2::VertexAttribute<Eigen::Vector3f>&, Attribute<float32, Orbit::PHI2>&);
template CGOGN_GEOMETRY_EXPORT void compute_angle_between_face_normals(const CMap2&, const CMap2::VertexAttribute<Eigen::Vector3d>&, Attribute<float64, Orbit::PHI2>&);


} // namespace geometry
} // namespace cgogn


#include <cgogn/geometry/algos/selection.h>

namespace cgogn
{
namespace geometry
{

template class CGOGN_GEOMETRY_EXPORT Collector_OneRing<Eigen::Vector3f, CMap2>;
template class CGOGN_GEOMETRY_EXPORT Collector_OneRing<Eigen::Vector3d, CMap2>;
template class CGOGN_GEOMETRY_EXPORT Collector_OneRing<Eigen::Vector3f, CMap3>;
template class CGOGN_GEOMETRY_EXPORT Collector_OneRing<Eigen::Vector3d, CMap3>;
template class CGOGN_GEOMETRY_EXPORT Collector_WithinSphere<Eigen::Vector3f, CMap2>;
template class CGOGN_GEOMETRY_EXPORT Collector_WithinSphere<Eigen::Vector3d, CMap2>;
template class CGOGN_GEOMETRY_EXPORT Collector_WithinSphere<Eigen::Vector3f, CMap3>;
template class CGOGN_GEOMETRY_EXPORT Collector_WithinSphere<Eigen::Vector3d, CMap3>;


} // namespace geometry
} // namespace cgogn

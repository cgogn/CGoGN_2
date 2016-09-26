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

#define CGOGN_GEOMETRY_ALGOS_ANGLE_CPP_

#include <cgogn/geometry/algos/angle.h>

namespace cgogn
{

namespace geometry
{

template CGOGN_GEOMETRY_API float32 angle_between_face_normals<Eigen::Vector3f, CMap2<DefaultMapTraits>>(const CMap2<DefaultMapTraits>&,const Cell<Orbit::PHI2>,const CMap2<DefaultMapTraits>::VertexAttribute<Eigen::Vector3f>&);
template CGOGN_GEOMETRY_API float64 angle_between_face_normals<Eigen::Vector3d, CMap2<DefaultMapTraits>>(const CMap2<DefaultMapTraits>&,const Cell<Orbit::PHI2>,const CMap2<DefaultMapTraits>::VertexAttribute<Eigen::Vector3d>&);
template CGOGN_GEOMETRY_API void compute_angle_between_face_normals<Eigen::Vector3f, CMap2<DefaultMapTraits>>(const CMap2<DefaultMapTraits>&,const CMap2<DefaultMapTraits>::VertexAttribute<Eigen::Vector3f>&, CMap2<DefaultMapTraits>::Attribute<float32, Orbit::PHI2>&);
template CGOGN_GEOMETRY_API void compute_angle_between_face_normals<Eigen::Vector3d, CMap2<DefaultMapTraits>>(const CMap2<DefaultMapTraits>&,const CMap2<DefaultMapTraits>::VertexAttribute<Eigen::Vector3d>&, CMap2<DefaultMapTraits>::Attribute<float64, Orbit::PHI2>&);

} // namespace geometry
} // namespace cgogn

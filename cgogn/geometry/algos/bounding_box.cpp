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

#define CGOGN_GEOMETRY_ALGO_BOUNDING_BOX_CPP_

#include <cgogn/geometry/algos/bounding_box.h>

namespace cgogn
{

namespace geometry
{

template CGOGN_GEOMETRY_API void compute_AABB<CMap2<DefaultMapTraits>::VertexAttribute<Eigen::Vector3f>>(const CMap2<DefaultMapTraits>::VertexAttribute<Eigen::Vector3f>&, AABB<Eigen::Vector3f>& bb);
template CGOGN_GEOMETRY_API void compute_AABB<CMap2<DefaultMapTraits>::VertexAttribute<Eigen::Vector3d>>(const CMap2<DefaultMapTraits>::VertexAttribute<Eigen::Vector3d>&, AABB<Eigen::Vector3d>& bb);
template CGOGN_GEOMETRY_API void compute_AABB<CMap3<DefaultMapTraits>::VertexAttribute<Eigen::Vector3f>>(const CMap3<DefaultMapTraits>::VertexAttribute<Eigen::Vector3f>&, AABB<Eigen::Vector3f>& bb);
template CGOGN_GEOMETRY_API void compute_AABB<CMap3<DefaultMapTraits>::VertexAttribute<Eigen::Vector3d>>(const CMap3<DefaultMapTraits>::VertexAttribute<Eigen::Vector3d>&, AABB<Eigen::Vector3d>& bb);


template CGOGN_GEOMETRY_API void compute_OBB<CMap2<DefaultMapTraits>::VertexAttribute<Eigen::Vector3f>>(const CMap2<DefaultMapTraits>::VertexAttribute<Eigen::Vector3f>&, OBB<Eigen::Vector3f>& bb);
template CGOGN_GEOMETRY_API void compute_OBB<CMap2<DefaultMapTraits>::VertexAttribute<Eigen::Vector3d>>(const CMap2<DefaultMapTraits>::VertexAttribute<Eigen::Vector3d>&, OBB<Eigen::Vector3d>& bb);
template CGOGN_GEOMETRY_API void compute_OBB<CMap3<DefaultMapTraits>::VertexAttribute<Eigen::Vector3f>>(const CMap3<DefaultMapTraits>::VertexAttribute<Eigen::Vector3f>&, OBB<Eigen::Vector3f>& bb);
template CGOGN_GEOMETRY_API void compute_OBB<CMap3<DefaultMapTraits>::VertexAttribute<Eigen::Vector3d>>(const CMap3<DefaultMapTraits>::VertexAttribute<Eigen::Vector3d>&, OBB<Eigen::Vector3d>& bb);

} // namespace geometry
} // namespace cgogn

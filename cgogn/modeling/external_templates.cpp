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
#define CGOGN_MODELING_EXTERNAL_TEMPLATES_CPP_

#include <cgogn/modeling/algos/loop.h>
#include <cgogn/modeling/algos/catmull_clark.h>
#include <cgogn/modeling/algos/doo_sabin.h>
#include <cgogn/modeling/algos/decimation.h>
#include <cgogn/modeling/algos/pliant_remeshing.h>
#include <cgogn/modeling/algos/refinements.h>
#include <cgogn/modeling/cgogn_modeling_export.h>

namespace cgogn
{
namespace modeling
{

template CGOGN_MODELING_EXPORT void loop(CMap2&, CMap2::VertexAttribute<Eigen::Vector3f>&);
template CGOGN_MODELING_EXPORT void loop(CMap2&, CMap2::VertexAttribute<Eigen::Vector3d>&);
template CGOGN_MODELING_EXPORT void loop(CMap3&, CMap3::VertexAttribute<Eigen::Vector3f>&);
template CGOGN_MODELING_EXPORT void loop(CMap3&, CMap3::VertexAttribute<Eigen::Vector3d>&);
template CGOGN_MODELING_EXPORT CMap2::Vertex quadrangule_face<CMap2>(CMap2&, CMap2::Face);
template CGOGN_MODELING_EXPORT CMap3::Vertex quadrangule_face<CMap3>(CMap3&, CMap3::Face);
template CGOGN_MODELING_EXPORT void catmull_clark(CMap2&, CMap2::VertexAttribute<Eigen::Vector3f>&);
template CGOGN_MODELING_EXPORT void catmull_clark(CMap2&, CMap2::VertexAttribute<Eigen::Vector3d>&);
template CGOGN_MODELING_EXPORT void doo_sabin(CMap2&, CMap2::VertexAttribute<Eigen::Vector3f>&);
template CGOGN_MODELING_EXPORT void doo_sabin(CMap2&, CMap2::VertexAttribute<Eigen::Vector3d>&);
template CGOGN_MODELING_EXPORT void decimate(CMap2&, CMap2::VertexAttribute<Eigen::Vector3f>&, EdgeTraversorType, EdgeApproximatorType, uint32);
template CGOGN_MODELING_EXPORT void decimate(CMap2&, CMap2::VertexAttribute<Eigen::Vector3d>&, EdgeTraversorType, EdgeApproximatorType, uint32);
template CGOGN_MODELING_EXPORT void pliant_remeshing(CMap2&, CMap2::VertexAttribute<Eigen::Vector3f>&);
template CGOGN_MODELING_EXPORT void pliant_remeshing(CMap2&, CMap2::VertexAttribute<Eigen::Vector3d>&);
template CGOGN_MODELING_EXPORT CMap2::Vertex triangule(CMap2&, CMap2::Face);
template CGOGN_MODELING_EXPORT CMap3::Vertex triangule(CMap3&, CMap3::Face);
template CGOGN_MODELING_EXPORT void triangule(CMap2&, CMap2::VertexAttribute<Eigen::Vector3f>&);
template CGOGN_MODELING_EXPORT void triangule(CMap2&, CMap2::VertexAttribute<Eigen::Vector3d>&);

} // nampespace modeling
} // namespace cgogn

#include <cgogn/modeling/tiling/tiling.h>
#include <cgogn/modeling/tiling/square_cylinder.h>
#include <cgogn/modeling/tiling/square_grid.h>


namespace cgogn
{
namespace modeling
{

template class CGOGN_MODELING_EXPORT Tiling<CMap2>;
template class CGOGN_MODELING_EXPORT Tiling<CMap3>;
template class CGOGN_MODELING_EXPORT SquareCylinder<CMap2>;
template class CGOGN_MODELING_EXPORT SquareGrid<CMap2>;

} // nampespace modeling
} // namespace cgogn


#include <cgogn/modeling/tiling/square_tore.h>
#include <cgogn/modeling/tiling/triangular_cylinder.h>
#include <cgogn/modeling/tiling/triangular_grid.h>


namespace cgogn
{
namespace modeling
{

template class CGOGN_MODELING_EXPORT SquareTore<CMap2>;
template class CGOGN_MODELING_EXPORT TriangularCylinder<CMap2>;
template class CGOGN_MODELING_EXPORT TriangularGrid<CMap2>;

} // nampespace modeling
} // namespace cgogn

#include <cgogn/modeling/tiling/triangular_tore.h>

namespace cgogn
{
namespace modeling
{

template class CGOGN_MODELING_EXPORT TriangularTore<CMap2>;

} // nampespace modeling
} // namespace cgogn

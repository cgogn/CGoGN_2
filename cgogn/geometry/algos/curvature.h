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

#ifndef CGOGN_GEOMETRY_ALGOS_CURVATURE_H_
#define CGOGN_GEOMETRY_ALGOS_CURVATURE_H_

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/algos/selection.h>
#include <cgogn/core/utils/masks.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3, typename MAP>
void curvature(
	const MAP& map,
	const Cell<Orbit::PHI21> v,
	typename vector_traits<VEC3>::Scalar radius,
	const typename MAP::template Attribute<VEC3, Orbit::PHI21>& position,
	const typename MAP::template Attribute<VEC3, Orbit::PHI21>& normal,
	const typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI2>& edge_angle,
	const typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI2>& edge_area,
	typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI21>& kmax,
	typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI21>& kmin,
	typename MAP::template Attribute<VEC3, Orbit::PHI21>& Kmax,
	typename MAP::template Attribute<VEC3, Orbit::PHI21>& Kmin,
	typename MAP::template Attribute<VEC3, Orbit::PHI21>& Knormal
)
{
	using Scalar = typename vector_traits<VEC3>::Scalar;
	using Vertex2 = Cell<Orbit::PHI21>;
	using Edge2 = Cell<Orbit::PHI2>;

	// collect the normal cycle tensor
	geometry::Collector<VEC3> neighborhood(map);
	neigh.collect_within_sphere<Edge2>(v, radius, position);
	Eigen::Matrix3d tensor(0);
	map.foreach_cell([&] (Edge2 e)
	{
		std::pair<Vertex2, Vertex2> vv = map.vertices(e);
		const VEC3& p1 = position[vv.first];
		const VEC3& p2 = position[vv.second];
		Eigen::Vector3d ev = Eigen::Vector3d(p2[0], p2[1], p2[2]) - Eigen::Vector3d(p1[0], p1[1], p1[2]);
		tensor += (ev * ev.transpose()) * edge_angle[e] * (Scalar(1) / edge_length(map, e, position));
	},
	neighborhood);

	// project the tensor
	Eigen::Matrix3d proj;
	proj.setIdentity();
	Eigen::Vector3d nv = normal[v];
	proj -= nv * nv.transpose();
	tensor = proj * tensor * proj;

	// solve eigen problem
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(tensor);
	const Eigen::Vector3d& ev = solver.eigenvalues();
	const Eigen::Matrix3d& evec = solver.eigenvectors();

	// sort eigen components : ev[inormal] has minimal absolute value ; kmin = ev[imin] <= ev[imax] = kmax
	uint32 inormal = 0, imin, imax;
	if (fabs(ev[1]) < fabs(ev[inormal])) inormal = 1;
	if (fabs(ev[2]) < fabs(ev[inormal])) inormal = 2;
	imin = (inormal + 1) % 3;
	imax = (inormal + 2) % 3;
	if (ev[imax] < ev[imin]) { uint32 tmp = imin; imin = imax; imax = tmp; }

	// set curvatures from sorted eigen components
	// warning : Kmin and Kmax are switched w.r.t. kmin and kmax
	// normal direction : minimal absolute eigen value
	Knormal[0] = evec(0, inormal);
	Knormal[1] = evec(1, inormal);
	Knormal[2] = evec(2, inormal);
	if (Knormal * normal < 0) Knormal *= -1; // change orientation
	// min curvature
	kmin = ev[imin] ;
	Kmin[0] = evec(0, imax);
	Kmin[1] = evec(1, imax);
	Kmin[2] = evec(2, imax);
	// max curvature
	kmax = ev[imax] ;
	Kmax[0] = evec(0, imin);
	Kmax[1] = evec(1, imin);
	Kmax[2] = evec(2, imin);
}

template <typename VEC3, typename MAP, typename MASK>
void curvature(
	const MAP& map,
	const MASK& mask,
	typename vector_traits<VEC3>::Scalar radius,
	const typename MAP::template Attribute<VEC3, Orbit::PHI21>& position,
	const typename MAP::template Attribute<VEC3, Orbit::PHI21>& normal,
	const typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI2>& edge_angle,
	const typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI2>& edge_area,
	typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI21>& kmax,
	typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI21>& kmin,
	typename MAP::template Attribute<VEC3, Orbit::PHI21>& Kmax,
	typename MAP::template Attribute<VEC3, Orbit::PHI21>& Kmin,
	typename MAP::template Attribute<VEC3, Orbit::PHI21>& Knormal
)
{
	map.parallel_foreach_cell([&] (Cell<Orbit::PHI21> v, uint32)
	{
		curvature(map, v, radius, position, normal, edge_angle, edge_area, kmax, kmin, Kmax, Kmin, Knormal);
	},
	mask);
}

template <typename VEC3, typename MAP>
void curvature(
	const MAP& map,
	typename vector_traits<VEC3>::Scalar radius,
	const typename MAP::template Attribute<VEC3, Orbit::PHI21>& position,
	const typename MAP::template Attribute<VEC3, Orbit::PHI21>& normal,
	const typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI2>& edge_angle,
	const typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI2>& edge_area,
	typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI21>& kmax,
	typename MAP::template Attribute<typename vector_traits<VEC3>::Scalar, Orbit::PHI21>& kmin,
	typename MAP::template Attribute<VEC3, Orbit::PHI21>& Kmax,
	typename MAP::template Attribute<VEC3, Orbit::PHI21>& Kmin,
	typename MAP::template Attribute<VEC3, Orbit::PHI21>& Knormal
)
{
	curvature<VEC3>(map, CellFilters(), radius, position, normal, edge_angle, edge_area, kmax, kmin, Kmax, Kmin, Knormal);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_CURVATURE_H_

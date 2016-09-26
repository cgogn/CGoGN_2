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
#include <cgogn/geometry/algos/length.h>
#include <cgogn/geometry/functions/intersection.h>
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
	using Face = typename MAP::Face;

	// collect the normal cycle tensor
	geometry::Collector_WithinSphere<VEC3, MAP> neighborhood(map, radius, position);
	neighborhood.collect(v);

	Eigen::Matrix3d tensor;
	tensor.setZero();

	neighborhood.foreach_cell([&] (Edge2 e)
	{
		std::pair<Vertex2, Vertex2> vv = map.vertices(e);
		const VEC3& p1 = position[vv.first];
		const VEC3& p2 = position[vv.second];
		Eigen::Vector3d ev = Eigen::Vector3d(p2[0], p2[1], p2[2]) - Eigen::Vector3d(p1[0], p1[1], p1[2]);
		tensor += (ev * ev.transpose()) * edge_angle[e] * (Scalar(1) / ev.norm());
	});

	neighborhood.foreach_border([&] (Dart d)
	{
		std::pair<Vertex2, Vertex2> vv = map.vertices(Edge2(d));
		const VEC3& p1 = position[vv.first];
		const VEC3& p2 = position[vv.second];
		Eigen::Vector3d ev = Eigen::Vector3d(p2[0], p2[1], p2[2]) - Eigen::Vector3d(p1[0], p1[1], p1[2]);
		Scalar alpha;
		geometry::intersection_sphere_segment<VEC3>(position[v], radius, position[Vertex2(d)], position[Vertex2(map.phi1(d))], alpha);
		tensor += (ev * ev.transpose()) * edge_angle[Edge2(d)] * (Scalar(1) / ev.norm()) * alpha;
	});

	tensor /= neighborhood.area(position);

	const VEC3& normal_v = normal[v];
	Eigen::Vector3d e_normal_v(normal_v[0], normal_v[1], normal_v[2]);

	// project the tensor
	Eigen::Matrix3d proj;
	proj.setIdentity();
	proj -= e_normal_v * e_normal_v.transpose();
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
	if (ev[imax] < ev[imin]) { std::swap(imin, imax); }

	// set curvatures from sorted eigen components
	// warning : Kmin and Kmax are switched w.r.t. kmin and kmax

	// normal direction : minimal absolute eigen value
	VEC3& Knormal_v = Knormal[v];
	Knormal_v[0] = evec(0, inormal);
	Knormal_v[1] = evec(1, inormal);
	Knormal_v[2] = evec(2, inormal);
	if (Knormal_v.dot(normal_v) < 0)
		Knormal_v *= Scalar(-1); // change orientation

	// min curvature
	kmin[v] = ev[imin];
	VEC3& Kmin_v = Kmin[v];
	Kmin_v[0] = evec(0, imax);
	Kmin_v[1] = evec(1, imax);
	Kmin_v[2] = evec(2, imax);

	// max curvature
	kmax[v] = ev[imax];
	VEC3& Kmax_v = Kmax[v];
	Kmax_v[0] = evec(0, imin);
	Kmax_v[1] = evec(1, imin);
	Kmax_v[2] = evec(2, imin);
}

template <typename VEC3, typename MAP, typename MASK>
void compute_curvature(
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
		curvature<VEC3>(map, v, radius, position, normal, edge_angle, edge_area, kmax, kmin, Kmax, Kmin, Knormal);
	},
	mask);
}

template <typename VEC3, typename MAP>
void compute_curvature(
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
	compute_curvature<VEC3>(map, CellFilters(), radius, position, normal, edge_angle, edge_area, kmax, kmin, Kmax, Kmin, Knormal);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_CURVATURE_H_

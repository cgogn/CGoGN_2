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

#ifndef CGOGN_GEOMETRY_ALGO_BOUNDING_BOX_H_
#define CGOGN_GEOMETRY_ALGO_BOUNDING_BOX_H_

#include <cgogn/geometry/types/aabb.h>
#include <cgogn/geometry/types/obb.h>
#include <cgogn/core/cmap/cmap3.h>

namespace cgogn
{

namespace geometry
{

template <typename ATTR>
void compute_AABB(const ATTR& attr, AABB<array_data_type<ATTR>>& bb)
{
	bb.reset();
	for(const auto& p : attr)
		bb.add_point(p);
}

template <typename ATTR>
void pca(const ATTR& attr)
{
	unused_parameters(attr);// temporary for warning
	//mean centering data
	//TODO
}

template <typename ATTR>
void compute_OBB(const ATTR& attr, OBB<array_data_type<ATTR>>& bb)
{
	using Vec = array_data_type<ATTR>;
	using Scalar = typename Vec::Scalar;
	using Mat3 = Eigen::Matrix<Scalar, 3, 3>;

	bb.reset();

	// compute the mean of the dataset (centroid)
	Vec mean;
	mean.setZero();
	uint32 count = 0;
	for (const auto& p : attr)
	{
		mean += Vec(p[0], p[1], p[2]);
		++count;
	}
	mean /= count;

	// compute covariance matrix
	Mat3 covariance;
	covariance.setZero();

	for (const auto& p : attr)
	{
		Eigen::Matrix<Scalar, 4, 1> point;
		point[0] = p[0] - mean[0];
		point[1] = p[1] - mean[1];
		point[2] = p[2] - mean[2];

		covariance(1, 1) += point[1] * point[1];
		covariance(1, 2) += point[1] * point[2];

		covariance(2, 2) += point[2] * point[2];

		point *= point[0];

		covariance(0, 0) += point[0];
		covariance(0, 1) += point[1];
		covariance(0, 2) += point[2];
	}

	covariance(1, 0) = covariance(0, 1);
	covariance(2, 0) = covariance(0, 2);
	covariance(2, 1) = covariance(1, 2);

	covariance /= count;

	// Extract axes (i.e. eigenvectors) from covariance matrix.
	Eigen::SelfAdjointEigenSolver<Mat3> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	auto eivecs = eigen_solver.eigenvectors();

	// Compute the size of the obb
	Vec ex;
	Vec t;
	for (const auto& p : attr)
	{
		t = eivecs.transpose() * (p - mean);

		t(0) = std::abs(t(0));
		t(1) = std::abs(t(1));
		t(2) = std::abs(t(2));

		if (t(0) > ex(0))
			ex(0) = t(0);
		if (t(1) > ex(1))
			ex(1) = t(1);
		if (t(2) > ex(2))
			ex(2) = t(2);
	}

	bb.axis(eivecs);
	bb.center(mean);
	bb.extension(ex);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGO_BOUNDING_BOX_H_

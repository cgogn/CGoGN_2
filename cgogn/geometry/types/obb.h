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

#ifndef CGOGN_GEOMETRY_TYPES_OBB_H_
#define CGOGN_GEOMETRY_TYPES_OBB_H_

#include <array>

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/basic/cell.h>
#include <cgogn/core/cmap/attribute.h>
#include <cgogn/core/utils/masks.h>

#include <cgogn/geometry/dll.h>
#include <cgogn/geometry/types/geometry_traits.h>

namespace cgogn
{

namespace geometry
{

/**
 * Object Bounding Box
 * http://jamesgregson.blogspot.fr/2011/03/latex-test.html
 */
template <typename VEC_T>
class OBB
{
public:

	using Vec = VEC_T;
	using Scalar = typename vector_traits<Vec>::Scalar;
	using Self = OBB<Vec>;

	/**
	 * \brief Dimension of the dataset
	 */
	static const uint32 dim_ = vector_traits<Vec>::SIZE;

	using Matrix = Eigen::Matrix<Scalar, dim_, dim_>;

private:

	/**
	 * \brief Translation component of the transformation
	 */
	Vec pos_;

	/**
	 * \brief Bounding box extents
	 */
	Vec extension_;

	/**
	 * \brief The axes defining the OBB
	 */
	Matrix rotation_;

	/**
	 * \brief If the OBB is initiliazed or not
	 */
	bool initialized_;

public:

	/**********************************************/
	/*                CONSTRUCTORS                */
	/**********************************************/

	OBB() :
		initialized_(false)
	{}

	// reinitialize the oriented bounding box
	void reset()
	{
		initialized_ = false;
	}

	Vec diag() const
	{
		Vec r(rotation_.col(0));
		Vec u(rotation_.col(1));
		Vec f(rotation_.col(2));

		Vec max = pos_ - r*extension_[0] - u*extension_[1] - f*extension_[2];
		Vec min = pos_ + r*extension_[0] + u*extension_[1] + f*extension_[2];

		return max - min;
	}

	inline Scalar diag_size() const
	{
		return Scalar(diag().norm());
	}

	/**
	 * @brief Returns the volume of the OBB.
	 * It is measure of how tight the fit is.
	 * Better OBBs wil have smaller volumes.
	 * @return
	 */
	inline Scalar volume()
	{
		return Scalar(8.0) * extension_[0] * extension_[1] * extension_[2];
	}

	inline bool is_valid()
	{
		std::array<Vec, 8> cvertices;
		corner_vertices(cvertices);

		for(auto& ve : cvertices)
			if(ve.hasNaN())
				return false;

		return true;
	}

	void corner_vertices(std::array<Vec, 8>& e)
	{
		Vec r(rotation_.col(0));
		Vec u(rotation_.col(1));
		Vec f(rotation_.col(2));

		e[0] = pos_ - r*extension_[0] - u*extension_[1] - f*extension_[2];
		e[1] = pos_ - r*extension_[0] - u*extension_[1] + f*extension_[2];
		e[2] = pos_ - r*extension_[0] + u*extension_[1] - f*extension_[2];
		e[3] = pos_ - r*extension_[0] + u*extension_[1] + f*extension_[2];

		e[4] = pos_ + r*extension_[0] - u*extension_[1] - f*extension_[2];
		e[5] = pos_ + r*extension_[0] - u*extension_[1] + f*extension_[2];
		e[6] = pos_ + r*extension_[0] + u*extension_[1] - f*extension_[2];
		e[7] = pos_ + r*extension_[0] + u*extension_[1] + f*extension_[2];
	}

	/**
	 * \brief Build an OBB from an attribute of points.
	 * This method just forms the covariance matrix and hands
	 * it to the build_from_covariance_matrix() method
	 * which handles fitting the box to the points
	 * \param[in] attr A container that supports iterators of \p vector
	 */
	template <typename ATTR>
	inline void build(const ATTR& attr)
	{
		static_assert(std::is_same<array_data_type<ATTR>, Vec>::value, "Wrong attribute type");

		// compute the mean of the dataset (centroid)
		Vec mean;
		cgogn::geometry::set_zero(mean);
		uint32 count = 0;
		for (const auto& p : attr)
		{
			mean += p;
			++count;
		}
		mean /= count;

		// compute covariance matrix
		Matrix covariance;
		covariance.setZero();

		for (uint32 j = 0; j < dim_; j++)
		{
			for (uint32 k = 0; k < dim_; k++)
			{
				for (const auto& p : attr)
					covariance(j,k) += (p[j] - mean[j]) * (p[k] - mean[k]);

				covariance(j,k) /= count - 1;
			}
		}

		build_from_covariance_matrix(covariance, attr);
	}

	/**
	 * \brief Build an OBB from the vertices of a map
	 * This method just forms the covariance matrix and hands
	 * it to the build_from_covariance_matrix() method
	 * which handles fitting the box to the points
	 * \param[in] attr An attribut of \p vector
	 * \param[in] map The map to browse
	 */
	template <typename MAP>
	inline void build_from_vertices(const MAP& map, const typename MAP::template VertexAttribute<VEC_T>& attr)
	{
		using Vertex = typename MAP::Vertex;

		// compute the mean of the dataset (centroid)
		Vec mean;
		cgogn::geometry::set_zero(mean);
		uint32 count = 0;

		CellCache<MAP> cache(map);
		cache.template build<Vertex>();

		map.foreach_cell(
			[&] (Vertex v)
			{
				mean += attr[v];
				++count;
			},
			cache
		);
		mean /= count;

		// compute covariance matrix
		Matrix covariance;
		covariance.setZero();

		for (uint32 j = 0; j < dim_; j++)
		{
			for (uint32 k = 0; k < dim_; k++)
			{
				map.foreach_cell(
					[&] (Vertex v)
					{
						covariance(j,k) += (attr[v][j] - mean[j]) * (attr[v][k] - mean[k]);
					},
					cache
				);

				covariance(j,k) /= count - 1;
			}
		}

		build_from_covariance_matrix(covariance, map, attr);
	}

	/**
	 * \brief Build an OBB from the vertices of a map of a given connected component
	 * This method just forms the covariance matrix and hands
	 * it to the build_from_covariance_matrix() method
	 * which handles fitting the box to the points
	 * \param[in] attr An attribut of \p vector
	 * \param[in] map The map to browse
	 * \param[in] cc The connected component
	 */
	template <typename MAP>
	inline void build_from_vertices(const MAP& map, const typename MAP::template VertexAttribute<VEC_T>& attr, typename MAP::ConnectedComponent cc)
	{
		using Vertex = typename MAP::Vertex;

		// compute the mean of the dataset (centroid)
		Vec mean;
		cgogn::geometry::set_zero(mean);
		uint32 count = 0;
		std::vector<Vertex> incident_to_cc;
		incident_to_cc.reserve(1024);
		map.foreach_incident_vertex(cc, [&] (Vertex v)
		{
			mean += attr[v];
			incident_to_cc.push_back(v);
			++count;
		});
		mean /= count;

		// compute covariance matrix
		Matrix covariance;
		covariance.setZero();

		for(int j = 0; j < dim_; j++)
			for(int k = 0; k < dim_; k++)
			{
				for(Vertex v : incident_to_cc)
					covariance(j,k) += (attr[v][j] - mean[j]) * (attr[v][k] - mean[k]);

				covariance(j,k) /= count - 1;
			}

		build_from_covariance_matrix(covariance, map, attr);
	}

	/**
	 * \brief Builds an OBB from triangles of a map of a given connected component.
	 *  Forms the covariance matrix for the triangles, then uses the
	 * method build_from_covariance_matrix() method to fit the box.
	 */
	template <typename MAP>
	inline void build_from_triangles(const MAP& map, const typename MAP::template VertexAttribute<VEC_T>& attr, typename MAP::ConnectedComponent cc)
	{
		using Vertex = typename MAP::Vertex;
		using Face = typename MAP::Face;

		Scalar Ai = 0.0, Am = 0.0;

		Vec mu, mui;
		cgogn::geometry::set_zero(mu);
		cgogn::geometry::set_zero(mui);

		Scalar cxx = 0.0, cxy = 0.0, cxz = 0.0, cyy = 0.0, cyz = 0.0, czz = 0.0;

		// loop over the triangles this time to find the
		// mean location
		map.foreach_incident_face(cc, [&] (Face f)
		{
			Vec p = attr[Vertex(f.dart)];
			Vec q = attr[Vertex(map.phi1(f.dart))];
			Vec r = attr[Vertex(map.phi_1(f.dart))];

			mui = (p + q + r) / 3.0 ;

			Ai = (q-p).cross(r-p).norm() / 2.0;

			mu += mui * Ai;
			Am += Ai;

			// these bits set the c terms to Am*E[xx], Am*E[xy], Am*E[xz]....
			cxx += ( 9.0*mui[0]*mui[0] + p[0]*p[0] + q[0]*q[0] + r[0]*r[0])*(Ai/12.0);
			cxy += ( 9.0*mui[0]*mui[1] + p[0]*p[1] + q[0]*q[1] + r[0]*r[1])*(Ai/12.0);
			cxz += ( 9.0*mui[0]*mui[2] + p[0]*p[2] + q[0]*q[2] + r[0]*r[2])*(Ai/12.0);
			cyy += ( 9.0*mui[1]*mui[1] + p[1]*p[1] + q[1]*q[1] + r[1]*r[1])*(Ai/12.0);
			cyz += ( 9.0*mui[1]*mui[2] + p[1]*p[2] + q[1]*q[2] + r[1]*r[2])*(Ai/12.0);
		});

		// divide out the Am fraction from the average position and
		// covariance terms
		mu /= Am;
		cxx /= Am; cxy /= Am; cxz /= Am; cyy /= Am; cyz /= Am; czz /= Am;

		// now subtract off the E[x]*E[x], E[x]*E[y], ... terms
		cxx -= mu[0]*mu[0]; cxy -= mu[0]*mu[1]; cxz -= mu[0]*mu[2];
		cyy -= mu[1]*mu[1]; cyz -= mu[1]*mu[2]; czz -= mu[2]*mu[2];

		Matrix covariance;
		covariance.setZero();

		// now build the covariance matrix
		covariance(0,0)=cxx; covariance(0,1)=cxy; covariance(0,2)=cxz;
		covariance(1,0)=cxy; covariance(1,1)=cyy; covariance(1,2)=cyz;
		covariance(2,0)=cxz; covariance(1,2)=cyz; covariance(2,2)=czz;

		build_from_covariance_matrix(covariance, map, attr, cc);
	}

	static std::string cgogn_name_of_type()
	{
		return std::string("cgogn::geometry::OBB<") + name_of_type(Vec()) + std::string(">");
	}

private:

	template <typename ATTR>
	void build_from_covariance_matrix(Matrix& C, const ATTR& attr)
	{
		// Extract axes (i.e. eigenvectors) from covariance matrix.
		Eigen::SelfAdjointEigenSolver<Matrix> eigen_solver(C);
		Matrix eivecs = eigen_solver.eigenvectors();

		//TODO what follows works only for 3d vectors
		Vec r(eivecs.col(0));
		Vec u(eivecs.col(1));
		Vec f(eivecs.col(2));

		// Compute the size of the obb
		Vec max = Vec::Constant(dim_, std::numeric_limits<Scalar>::lowest());
		Vec min = Vec::Constant(dim_, std::numeric_limits<Scalar>::max());

		// now build the bounding box extents in the rotated frame
		for (const auto& p : attr)
		{
			Vec prime(r.dot(p), u.dot(p), f.dot(p));

			max = Vec(std::max(max[0], prime[0]), std::max(max[1], prime[1]), std::max(max[2], prime[2]));
			min = Vec(std::min(min[0], prime[0]), std::min(min[1], prime[1]), std::min(min[2], prime[2]));
		}

		// set the center of the OBB to be the average of the
		// minimum and maximum, and the extents be half of the
		// difference between the minimum and maximum

		rotation_ = eivecs;
		Vec center = max + min;
		center /= 2.0;
		pos_ = Vec(eivecs.row(0).dot(center), eivecs.row(1).dot(center), eivecs.row(2).dot(center));
		extension_ = Vec(std::abs(max[0] - min[0]) / 2.0 , std::abs(max[1] - min[1]) / 2.0, std::abs(max[2] - min[2]) / 2.0 );
	}

	template <typename MAP>
	void build_from_covariance_matrix(Matrix& C, const MAP& map, const typename MAP::template VertexAttribute<VEC_T>& attr)
	{
		using Vertex = typename MAP::Vertex;

		// Extract axes (i.e. eigenvectors) from covariance matrix.
		Eigen::SelfAdjointEigenSolver<Matrix> eigen_solver(C);
		Matrix eivecs = eigen_solver.eigenvectors();

		//TODO what follows works only for 3d vectors
		Vec r(eivecs.col(0));
		Vec u(eivecs.col(1));
		Vec f(eivecs.col(2));

		// Compute the size of the obb
		Vec max = Vec::Constant(dim_, std::numeric_limits<Scalar>::lowest());
		Vec min = Vec::Constant(dim_, std::numeric_limits<Scalar>::max());

		// now build the bounding box extents in the rotated frame
		map.foreach_cell([&] (Vertex v)
		{
			Vec prime(r.dot(attr[v]), u.dot(attr[v]), f.dot(attr[v]));

			max = Vec(std::max(max[0], prime[0]), std::max(max[1], prime[1]), std::max(max[2], prime[2]));
			min = Vec(std::min(min[0], prime[0]), std::min(min[1], prime[1]), std::min(min[2], prime[2]));
		});

		// set the center of the OBB to be the average of the
		// minimum and maximum, and the extents be half of the
		// difference between the minimum and maximum

		rotation_ = eivecs;
		Vec center = max + min;
		center /= 2.0;
		pos_ = Vec(eivecs.row(0).dot(center), eivecs.row(1).dot(center), eivecs.row(2).dot(center));
		extension_ = Vec(std::abs(max[0] - min[0]) / 2.0 , std::abs(max[1] - min[1]) / 2.0, std::abs(max[2] - min[2]) / 2.0 );
	}

	template <typename MAP>
	void build_from_covariance_matrix(Matrix& C, const MAP& map, const typename MAP::template VertexAttribute<VEC_T>& attr, typename MAP::ConnectedComponent cc)
	{
		using Vertex = typename MAP::Vertex;

		// Extract axes (i.e. eigenvectors) from covariance matrix.
		Eigen::SelfAdjointEigenSolver<Matrix> eigen_solver(C);
		Matrix eivecs = eigen_solver.eigenvectors();

		//TODO what follows works only for 3d vectors
		Vec r(eivecs.col(0));
		Vec u(eivecs.col(1));
		Vec f(eivecs.col(2));

		// Compute the size of the obb
		Vec max = Vec::Constant(dim_, std::numeric_limits<Scalar>::lowest());
		Vec min = Vec::Constant(dim_, std::numeric_limits<Scalar>::max());

		// now build the bounding box extents in the rotated frame
		map.foreach_incident_vertex(cc, [&] (Vertex c)
		{
			Vec prime(r.dot(attr[c]), u.dot(attr[c]), f.dot(attr[c]));

			max = Vec(std::max(max[0], prime[0]), std::max(max[1], prime[1]), std::max(max[2], prime[2]));
			min = Vec(std::min(min[0], prime[0]), std::min(min[1], prime[1]), std::min(min[2], prime[2]));

		});

		// set the center of the OBB to be the average of the
		// minimum and maximum, and the extents be half of the
		// difference between the minimum and maximum

		rotation_ = eivecs;
		Vec center = max + min;
		center /= 2.0;
		pos_ = Vec(eivecs.row(0).dot(center), eivecs.row(1).dot(center), eivecs.row(2).dot(center));
		extension_ = Vec(std::abs(max[0] - min[0]) / 2.0 , std::abs(max[1] - min[1]) / 2.0, std::abs(max[2] - min[2]) / 2.0 );
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_TYPES_OBB_CPP_))
extern template class CGOGN_GEOMETRY_API OBB<Eigen::Vector3d>;
extern template class CGOGN_GEOMETRY_API OBB<Eigen::Vector3f>;
//extern template class CGOGN_GEOMETRY_API OBB<Vec_T<std::array<float32, 3>>>;
//extern template class CGOGN_GEOMETRY_API OBB<Vec_T<std::array<float64,3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_TYPES_OBB_CPP_))

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_OBB_H_

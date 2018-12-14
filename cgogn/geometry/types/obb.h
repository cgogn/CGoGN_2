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
#include <tuple>
#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/basic/cell.h>
#include <cgogn/core/cmap/attribute.h>
#include <cgogn/core/utils/masks.h>

#include <cgogn/geometry/cgogn_geometry_export.h>
#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/types/aabb.h>

namespace cgogn
{

namespace geometry
{


/// ideas coming from
/// https://code.ill.fr/scientific-software/nsxtool/blob/master/nsxlib/geometry/OBB.h
template<typename T, uint32 D>
class OBB
{

public:
	//! Some useful typedefs;
	using Matrix = Eigen::Matrix<T,D,D>;
	using Vector = Eigen::Matrix<T,D,1>;
	using HomeMatrix = Eigen::Matrix<T,D+1,D+1>;
	using HomeVector = Eigen::Matrix<T,D+1,1>;

	//! Default constructor
	OBB()
	{}

	//! Copy constructor
	OBB(const OBB<T,D>& other)
	{
		transformation_inv_=other.transformation_inv_;
		eigen_values_=other.eigen_values_;
	}

	//! Construct a N-dimensional box from its center, semi-axes, and eigenvectors ()
	OBB(const Vector& center, const Vector& eigenvalues, const Matrix& eigenvectors) :
		eigen_values_(eigenvalues)
	{
		// Define the inverse scale matrix from the eigenvalues
		Eigen::DiagonalMatrix<T,D+1> Sinv;
		for (uint32 i=0; i<D; ++i)
			Sinv.diagonal()[i]=1.0/eigenvalues[i];
		Sinv.diagonal()[D]=1.0;

		// Now prepare the R^-1.T^-1 (rotation,translation)
		transformation_inv_=Eigen::Matrix<T,D+1,D+1>::Constant(0.0);
		transformation_inv_(D,D)=1.0;
		for (unsigned int i=0;i<D;++i)
			transformation_inv_.block(i,0,1,D)=eigenvectors.col(i).transpose().normalized();
		transformation_inv_.block(0,D,D,1)=-transformation_inv_.block(0,0,D,D)*center;

		// Finally compute (TRS)^-1 by left-multiplying (TR)^-1 by S^-1
		transformation_inv_=Sinv*transformation_inv_;
	}

	//! Construct a OBB from a AABB
	OBB(const AABB<Vector>& aabb)
	{
		transformation_inv_=Eigen::Matrix<T,D+1,D+1>::Identity();
		transformation_inv_.block(0,D,D,1)=-aabb.center();
	}

	//! Assignment
	OBB& operator=(const OBB<T,D>& other)
	{
		if(this!=&other)
		{
			eigen_values_=other.eigen_values_;
			transformation_inv_=other.transformation_inv_;
		}
		return *this;
	}

	//! The destructor.
	~OBB()
	{}

	//! Return the extents of the OBB
	const Vector& extents() const
	{
		return eigen_values_;
	}

	//! Return the inverse of the Mapping matrix (\f$ S^{-1}.R^{-1}.T^{-1} \f$)
	const HomeMatrix& inverse_transformation() const
	{
		return transformation_inv_;
	}

	//! Rotate the OBB.
	void rotate(const Matrix& eigenvectors)
	{
		// Reconstruct S
		Eigen::DiagonalMatrix<T,D+1> S;
		for(uint32 i=0; i<D; ++i)
			S.diagonal()[i]=eigen_values_[i];
		S.diagonal()[D]=1.0;

		transformation_inv_=S*transformation_inv_;

		// Construct the inverse of the new rotation matrix
		HomeMatrix Rnewinv=HomeMatrix::Zero();
		Rnewinv(D,D) = 1.0;
		for (uint32 i=0;i<D;++i)
			Rnewinv.block(i,0,1,D)=eigenvectors.col(i).transpose().normalized();
		transformation_inv_=Rnewinv*transformation_inv_;

		// Reconstruct Sinv
		for (uint32 i=0; i<D; ++i)
			S.diagonal()[i]=1.0/eigen_values_[i];
		S.diagonal()[D]=1.0;

		// Reconstruct the complete TRS inverse
		transformation_inv_ = S*transformation_inv_;
	}

	//! Scale isotropically the OBB.
	void scale(T value)
	{
		eigen_values_*=value;
		Eigen::DiagonalMatrix<T,D+1> Sinv;
		for (uint32 i=0; i<D; ++i)
			Sinv.diagonal()[i]=1.0/value;
		Sinv.diagonal()[D]=1.0;
		transformation_inv_=Sinv*transformation_inv_;
	}

	//! Scale anisotropically the OBB.
	void scale(const Vector& v)
	{
		eigen_values_=eigen_values_.cwiseProduct(v);
		Eigen::DiagonalMatrix<T,D+1> Sinv;
		for (uint32 i=0;i<D;++i)
			Sinv.diagonal()[i]=1.0/v[i];
		Sinv.diagonal()[D]=1.0;
		transformation_inv_=Sinv*transformation_inv_;
	}

	//! Translate the OBB.
	void translate(const Vector& t)
	{
		HomeMatrix tinv = HomeMatrix::Zero();
		for (uint32 i=0; i<D+1; ++i)
			tinv(i,i)=1.0;
		tinv.block(0,D,D,1)=-t;
		transformation_inv_=transformation_inv_*tinv;
	}

	std::tuple<Vector,Vector> bounds()
	{
		// Reconstruct S
		Eigen::DiagonalMatrix<T,D+1> S;
		for (uint32 i=0;i<D;++i)
			S.diagonal()[i]=eigen_values_[i];
		S.diagonal()[D]=1.0;

		// Reconstruct R from TRinv
		HomeMatrix TRinv=S*transformation_inv_;
		Matrix R=TRinv.block(0,0,D,D).transpose();

		// Extract T matrix from TRinv
		Vector Tmat=-R*TRinv.block(0,D,D,1);

		// Calculate the width of the bounding box
		Vector width=Vector::Constant(0.0);
		for (uint32 i=0;i<D;++i)
		{
			for (uint32 j=0;j<D;++j)
				width[i] += std::abs(eigen_values_[j]*R(j,i));
		}

		// Update the upper and lower bound of the AABB
		return {Tmat-width,Tmat+width};
	}

private:
	//! The inverse of the homogeneous transformation matrix.
	HomeMatrix transformation_inv_;

	//! The scale value.
	Vector eigen_values_;

public:
	// Macro to ensure that an OBB object can be dynamically allocated.
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

template <typename T, uint32 D>
typename OBB<T,D>::Vector diagonal(const OBB<T, D>& obb)
{
	typename OBB<T,D>::Vector min, max;
	std::tie(min, max) = obb.bounds();
	return max - min;
}


/// \brief Returns the volume of the OBB.
/// It is measure of how tight the fit is.
/// Better OBBs wil have smaller volumes.
/// \return
template <typename T, uint32 D>
T volume(const OBB<T, D>& obb)
{
	return std::pow(2., T(D)) * obb.extents().prod();
}

template <typename T, uint32 D=3>
void corner_vertices(const OBB<T, 3>& obb, std::array<typename OBB<T, 3>::Vector, 8>& e)
{
	using HomeMatrix = typename OBB<T, 3>::HomeMatrix;
	using Matrix = typename OBB<T, 3>::Matrix;
	using Vector = typename OBB<T, 3>::Vector;

	// Reconstruct S
	Vector eigen_values = obb.extents();
	Eigen::DiagonalMatrix<T,D+1> S;
	for (uint32 i=0;i<D;++i)
		S.diagonal()[i] = eigen_values[i];
	S.diagonal()[D]=1.0;

	// Reconstruct R from TRinv
	HomeMatrix TRinv = S * obb.inverse_transformation();
	Matrix R=TRinv.block(0,0,D,D).transpose();

	// Extract T matrix from TRinv
	Vector Tmat=-R*TRinv.block(0,D,D,1);

	e[0] = Tmat - R.col(0)*eigen_values[0] - R.col(1)*eigen_values[1] - R.col(2)*eigen_values[2];
	e[1] = Tmat - R.col(0)*eigen_values[0] - R.col(1)*eigen_values[1] + R.col(2)*eigen_values[2];
	e[2] = Tmat - R.col(0)*eigen_values[0] + R.col(1)*eigen_values[1] - R.col(2)*eigen_values[2];
	e[3] = Tmat - R.col(0)*eigen_values[0] + R.col(1)*eigen_values[1] + R.col(2)*eigen_values[2];

	e[4] = Tmat + R.col(0)*eigen_values[0] - R.col(1)*eigen_values[1] - R.col(2)*eigen_values[2];
	e[5] = Tmat + R.col(0)*eigen_values[0] - R.col(1)*eigen_values[1] + R.col(2)*eigen_values[2];
	e[6] = Tmat + R.col(0)*eigen_values[0] + R.col(1)*eigen_values[1] - R.col(2)*eigen_values[2];
	e[7] = Tmat + R.col(0)*eigen_values[0] + R.col(1)*eigen_values[1] + R.col(2)*eigen_values[2];
}

//! Check whether a point given as Homogeneous coordinate in the (D+1) dimension is inside the OBB.
template<typename T, uint32 D>
bool is_inside(const OBB<T,D>& other, const typename OBB<T,D>::HomeVector& point)
{
	typename OBB<T,D>::HomeVector p = other.inverse_transformation()*point;

	for(uint32 i=0; i<D; ++i)
	{
		if (p[i] < -1 || p[i] > 1)
			return false;
	}

	return true;
}

//! Returns true if the OBB collides with a AABB.
template<typename T, uint32 D>
bool collide(const OBB<T,D>& obb, const AABB<typename OBB<T,D>::Vector>& aabb)
{
	OBB<T,D> obb1(aabb);
	return collide<T,D>(obb,obb1);
}

//! Returns true if the OBB collides with an OBB.
template<typename T, uint32 D=2>
bool collide(const OBB<T,2> a, const OBB<T,2>& b)
{
	//Get the (TRS)^-1 matrices of the two OBBs
	const Eigen::Matrix<T,3,3>& trsinva=a.inverse_transformation();
	const Eigen::Matrix<T,3,3>& trsinvb=b.inverse_transformation();

	// Get the extent of the two OBBs
	const Eigen::Matrix<T,2,1>& eiga=a.extents();
	const Eigen::Matrix<T,2,1>& eigb=b.extents();

	// Reconstruct the S matrices for the two OBBs
	Eigen::DiagonalMatrix<T,3> sa;
	Eigen::DiagonalMatrix<T,3> sb;
	sa.diagonal() << eiga[0], eiga[1],1;
	sb.diagonal() << eigb[0], eigb[1],1;

	// Reconstruct the (TR)^-1 matrices for the two OBBs
	const Eigen::Matrix<T,3,3> trinva(sa*trsinva);
	const Eigen::Matrix<T,3,3> trinvb(sb*trsinvb);

	// Reconstruct R for the two OBBs
	Eigen::Matrix<T,2,2> ra(trinva.block(0,0,D,D).transpose());
	Eigen::Matrix<T,2,2> rb(trinvb.block(0,0,D,D).transpose());

	// Extract T matrix from TRinv
	Eigen::Matrix<T,2,1> ta=-ra*trinva.block(0,D,D,1);
	Eigen::Matrix<T,2,1> tb=-rb*trinvb.block(0,D,D,1);

	Eigen::Matrix<T,2,2> C=ra.transpose()*rb;
	Eigen::Matrix<T,2,2> Cabs=C.array().abs();

	// The difference vector between the centers of OBB2 and OBB1
	Eigen::Matrix<T,1,2> diff=(tb-ta).transpose();

	// If for one of the following 15 conditions, R<=(R0+R1) then the two OBBs collide.
	T R0, R, R1;

	// condition 1,2,3
	for (unsigned int i=0;i<D;++i)
	{
		R0=eiga[i];
		R1=(Cabs.block(i,0,1,D)*eigb)(0,0);
		R=std::abs((diff*ra.block(0,i,D,1))(0,0));
		if (R>(R0+R1))
			return false;
	}

	// condition 4,5,6
	for (unsigned int i=0;i<D;++i)
	{
		R0=(Cabs.block(i,0,1,D)*eiga)(0,0);
		R1=eigb[i];
		R=std::abs((diff*rb.block(0,i,D,1))(0,0));
		if (R>(R0+R1))
			return false;
	}

	return true;
}

//! Returns true if the OBB collides with an OBB.
template<typename T, uint32 D=3>
bool collide(const OBB<T,3> a, const OBB<T,3>& b)
{
	// Get the (TRS)^-1 matrices of the two OBBs
	const Eigen::Matrix<T,4,4>& trsinva=a.inverse_transformation();
	const Eigen::Matrix<T,4,4>& trsinvb=b.inverse_transformation();

	// Get the extent of the two OBBs
	const Eigen::Matrix<T,3,1>& eiga=a.extents();
	const Eigen::Matrix<T,3,1>& eigb=b.extents();

	// Reconstruct the S matrices for the two OBBs
	Eigen::DiagonalMatrix<T,4> sa;
	Eigen::DiagonalMatrix<T,4> sb;
	sa.diagonal() << eiga[0], eiga[1],eiga[2],1;
	sb.diagonal() << eigb[0], eigb[1],eigb[2],1;

	// Reconstruct the (TR)^-1 matrices for the two OBBs
	const Eigen::Matrix<T,4,4> trinva(sa*trsinva);
	const Eigen::Matrix<T,4,4> trinvb(sb*trsinvb);

	// Reconstruct R for the two OBBs
	Eigen::Matrix<T,3,3> ra(trinva.block(0,0,D,D).transpose());
	Eigen::Matrix<T,3,3> rb(trinvb.block(0,0,D,D).transpose());

	// Extract T matrix from TRinv
	Eigen::Matrix<T,3,1> ta=-ra*trinva.block(0,D,D,1);
	Eigen::Matrix<T,3,1> tb=-rb*trinvb.block(0,D,D,1);

	Eigen::Matrix<T,3,3> C=ra.transpose()*rb;
	Eigen::Matrix<T,3,3> Cabs=C.array().abs();

	// The difference vector between the centers of OBB2 and OBB1
	Eigen::Matrix<T,1,3> diff=(tb-ta).transpose();

	// If for one of the following 15 conditions, R<=(R0+R1) then the two OBBs collide.
	T R0, R, R1;

	// condition 1,2,3
	for (unsigned int i=0;i<D;++i)
	{
		R0=eiga[i];
		R1=(Cabs.block(i,0,1,D)*eigb)(0,0);
		R=std::abs((diff*ra.block(0,i,D,1))(0,0));
		if (R>(R0+R1))
			return false;
	}

	// condition 4,5,6
	for (unsigned int i=0;i<D;++i)
	{
		R0=(Cabs.block(i,0,1,D)*eiga)(0,0);
		R1=eigb[i];
		R=std::abs((diff*rb.block(0,i,D,1))(0,0));
		if (R>(R0+R1))
			return false;
	}

	T A0D((diff*ra.block(0,0,D,1))(0,0));
	T A1D((diff*ra.block(0,1,D,1))(0,0));
	T A2D((diff*ra.block(0,2,D,1))(0,0));

	// condition 7
	R0=eiga[1]*Cabs(2,0)+eiga[2]*Cabs(1,0);
	R1=eigb[1]*Cabs(0,2)+eigb[2]*Cabs(0,1);
	R=std::abs(C(1,0)*A2D-C(2,0)*A1D);
	if (R>(R0+R1))
		return false;

	// condition 8
	R0=eiga[1]*Cabs(2,1)+eiga[2]*Cabs(1,1);
	R1=eigb[0]*Cabs(0,2)+eigb[2]*Cabs(0,0);
	R=std::abs(C(1,1)*A2D-C(2,1)*A1D);
	if (R>(R0+R1))
		return false;

	// condition 9
	R0=eiga[1]*Cabs(2,2)+eiga[2]*Cabs(1,2);
	R1=eigb[0]*Cabs(0,1)+eigb[1]*Cabs(0,0);
	R=std::abs(C(1,2)*A2D-C(2,2)*A1D);
	if (R>(R0+R1))
		return false;

	// condition 10
	R0=eiga[0]*Cabs(2,0)+eiga[2]*Cabs(0,0);
	R1=eigb[1]*Cabs(1,2)+eigb[2]*Cabs(1,1);
	R=std::abs(C(2,0)*A0D-C(0,0)*A2D);
	if (R>(R0+R1))
		return false;

	// condition 11
	R0=eiga[0]*Cabs(2,1)+eiga[2]*Cabs(0,1);
	R1=eigb[0]*Cabs(1,2)+eigb[2]*Cabs(1,0);
	R=std::abs(C(2,1)*A0D-C(0,1)*A2D);
	if (R>(R0+R1))
		return false;

	// condition 12
	R0=eiga[0]*Cabs(2,2)+eiga[2]*Cabs(0,2);
	R1=eigb[0]*Cabs(1,1)+eigb[1]*Cabs(1,0);
	R=std::abs(C(2,2)*A0D-C(0,2)*A2D);
	if (R>(R0+R1))
		return false;

	// condition 13
	R0=eiga[0]*Cabs(1,0)+eiga[1]*Cabs(0,0);
	R1=eigb[1]*Cabs(2,2)+eigb[2]*Cabs(2,1);
	R=std::abs(C(0,0)*A1D-C(1,0)*A0D);
	if (R>(R0+R1))
		return false;

	// condition 14
	R0=eiga[0]*Cabs(1,1)+eiga[1]*Cabs(0,1);
	R1=eigb[0]*Cabs(2,2)+eigb[2]*Cabs(2,0);
	R=std::abs(C(0,1)*A1D-C(1,1)*A0D);
	if (R>(R0+R1))
		return false;

	// condition 15
	R0=eiga[0]*Cabs(1,2)+eiga[1]*Cabs(0,2);
	R1=eigb[0]*Cabs(2,1)+eigb[1]*Cabs(2,0);
	R=std::abs(C(0,2)*A1D-C(1,2)*A0D);
	if (R>(R0+R1))
		return false;

	return true;
}

//! Compute the intersection between the OBB and a given ray.
//! Return true if an intersection was found, false otherwise.
//! If the return value is true the intersection "times" will be stored
//! in t1 and t2 in such a way that from + t1*dir and from + t2*dir are
//! the two intersection points between the ray and this shape.
template<typename T, uint32 D>
bool ray_intersect(const OBB<T,D>& other,
				  const typename OBB<T,D>::Vector& from,
				  const typename OBB<T,D>::Vector& dir, T& t1, T& t2)
{
	using HomeVector = typename OBB<T,D>::HomeVector;
	using Vector = typename OBB<T,D>::Vector;
	HomeVector hFrom = other.inverse_transformation() * from.homogeneous();
	HomeVector hDir;
	hDir.segment(0,D) = dir;
	hDir[D] = 0.0;
	hDir = other.inverse_transformation()*hDir;

	//AABB<T,D> aabb(-Vector::Ones(), Vector::Ones());

	//return ray_intersect(aabb, hFrom.segment(0,D), hDir.segment(0,D), t1, t2);

	return true;
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_TYPES_OBB_CPP_))
extern template class CGOGN_GEOMETRY_EXPORT OBB<float, 2>;
extern template class CGOGN_GEOMETRY_EXPORT OBB<float, 3>;
extern template class CGOGN_GEOMETRY_EXPORT OBB<double, 2>;
extern template class CGOGN_GEOMETRY_EXPORT OBB<double, 3>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_TYPES_OBB_CPP_))

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_OBB_H_

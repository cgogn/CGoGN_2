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

#ifndef GEOMETRY_VEC_OPERATIONS_H_
#define GEOMETRY_VEC_OPERATIONS_H_

#include <cmath>
#include <type_traits>
#include <geometry/geometry_traits.h>
CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_ON
#include <Eigen/Dense>
CGOGN_PRAGMA_EIGEN_REMOVE_WARNINGS_OFF

namespace cgogn
{

namespace geometry
{

/*
 *  Beginning of declarations
 */
template<typename VEC_T>
VEC_T operator-(const VEC_T& v1);

template<typename VEC_T>
void operator+=(VEC_T& v1, const VEC_T& v2);

template<typename VEC_T>
void operator-=(VEC_T& v1, const VEC_T& v2);

template<typename VEC_T>
VEC_T operator+(const VEC_T& v1, const VEC_T& v2);

template<typename VEC_T>
VEC_T operator-(const VEC_T& v1, const VEC_T& v2);

template<typename VEC_T>
VEC_T operator*(const VEC_T& v1, const typename vector_traits<VEC_T>::Scalar r);

template<typename VEC_T>
VEC_T operator*( const typename vector_traits<VEC_T>::Scalar r, const VEC_T& v1);

template<typename VEC_T>
typename vector_traits<VEC_T>::Scalar operator*(const VEC_T& v1, const VEC_T& v2);

template<typename VEC_T>
VEC_T operator^(const VEC_T& v1, const VEC_T& v2);

template<typename VEC_T>
typename vector_traits<VEC_T>::Scalar norm2(const VEC_T& v);

template<typename VEC_T>
typename vector_traits<VEC_T>::Scalar norm(const VEC_T& v);

template<typename VEC_T>
void normalize(VEC_T& v);

/*
 *  End of declarations
 */


/*
 *  Beginning of Eigen specializations
 */


template<typename Scalar_, int Rows, int Options>
inline auto operator-(const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v1) -> Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>
{
	return v1.operator-();
}

template<typename Scalar_, int Rows, int Options>
inline void operator+=(Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v1, const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v2)
{
	v1.operator+=(v2);
}

template<typename Scalar_, int Rows, int Options>
inline void operator-=(Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v1, const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v2)
{
	v1.operator-=(v2);
}

template<typename Scalar_, int Rows, int Options>
inline Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1> operator+(const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v1, const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v2)
{
	return v1.operator+(v2);
}

template<typename Scalar_, int Rows, int Options>
inline Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1> operator-(const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v1, const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v2)
{
	return v1.operator-(v2);
}

template<typename Scalar_, int Rows, int Options>
inline Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1> operator*(const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v1, const Scalar_ r)
{
	return v1.operator*(r);
}

template<typename Scalar_, int Rows, int Options>
inline Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1> operator*(const Scalar_ r, const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v1)
{
	return v1.operator*(r);
}

template<typename Scalar_, int Rows, int Options>
inline Scalar_ operator*(const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v1, const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v2)
{
	return v1.dot(v2);
}

template<typename Scalar_, int Rows, int Options>
inline Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1> operator^(const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v1, const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v2)
{
	return v1.cross(v2);
}

template<typename Scalar_, int Rows, int Options>
Scalar_ norm2(const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v)
{
	return v.squaredNorm();
}

template<typename Scalar_, int Rows, int Options>
Scalar_ norm(const Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v)
{
	return v.norm();
}

template<typename Scalar_, int Rows, int Options>
void normalize(Eigen::Matrix<Scalar_,Rows,1,Options,Rows,1>& v)
{
	v.normalize();
}

/*
 *  End of Eigen specializations
 */


/*
 *  Beginning of definitions
 */

template<typename VEC_T>
VEC_T operator-(const VEC_T& v)
{
	VEC_T res(v);
	for (std::size_t i = 0ul; i < vector_traits<VEC_T>::SIZE; ++i)
	{
		res[i] = -res[i];
	}
	return res;
}

template<typename VEC_T>
void operator+=(VEC_T& v1, const VEC_T& v2)
{
	for (std::size_t i = 0ul; i < vector_traits<VEC_T>::SIZE; ++i)
	{
		v1[i] += v2[i];
	}
}

template<typename VEC_T>
void operator-=(VEC_T& v1, const VEC_T& v2)
{
	for (std::size_t i = 0ul; i < vector_traits<VEC_T>::SIZE; ++i)
	{
		v1[i] -= v2[i];
	}
}

template<typename VEC_T>
VEC_T operator+(const VEC_T& v1, const VEC_T& v2)
{
	VEC_T res;
	for (std::size_t i = 0ul; i < vector_traits<VEC_T>::SIZE; ++i)
	{
		res[i] = v1[i] + v2[i];
	}
	return res;
}

template<typename VEC_T>
VEC_T operator-(const VEC_T& v1, const VEC_T& v2)
{
	VEC_T res;
	for (std::size_t i = 0ul; i < vector_traits<VEC_T>::SIZE; ++i)
	{
		res[i] = v1[i] - v2[i];
	}
	return res;
}

template<typename VEC_T>
VEC_T operator*(const VEC_T& v1, const typename vector_traits<VEC_T>::Scalar r)
{
	VEC_T res(v1);
	for (std::size_t i = 0ul; i < vector_traits<VEC_T>::SIZE; ++i)
	{
		res[i] *= r;
	}
	return res;
}

template<typename VEC_T>
VEC_T operator*( const typename vector_traits<VEC_T>::Scalar r, const VEC_T& v1)
{
	return geometry::operator *(v1,r);
}

template<typename VEC_T>
typename vector_traits<VEC_T>::Scalar operator*(const VEC_T& v1, const VEC_T& v2)
{
	using Real = typename vector_traits<VEC_T>::Scalar;
	Real r{0};
	for (std::size_t i = 0ul; i < vector_traits<VEC_T>::SIZE; ++i)
	{
		r += v1[i] * v2[i];
	}
	return r;
}



template<typename VEC_T>
VEC_T operator^(const VEC_T& v1, const VEC_T& v2)
{
	static_assert(vector_traits<VEC_T>::SIZE == 3ul, "The size of the vector must be equal to 3.");
	VEC_T res;
	res[0] = v1[1] * v2[2] - v1[2] * v2[1];
	res[1] = v1[2] * v2[0] - v1[0] * v2[2];
	res[2] = v1[0] * v2[1] - v1[1] * v2[0];
	return res;
}

template<typename VEC_T>
typename vector_traits<VEC_T>::Scalar norm2(const VEC_T& v)
{
	using Real = typename vector_traits<VEC_T>::Scalar;
	Real r{0};
	for (std::size_t i = 0ul; i < vector_traits<VEC_T>::SIZE; ++i)
	{
		r += v[i]*v[i];
	}

	return r;
}

template<typename VEC_T>
typename vector_traits<VEC_T>::Scalar norm(const VEC_T& v)
{
	return std::sqrt(norm2(v));
}

template<typename VEC_T>
void normalize(VEC_T& v)
{
	using Real = typename vector_traits<VEC_T>::Scalar;
	const Real norm_value = norm(v);
	for (std::size_t i = 0ul; i < vector_traits<VEC_T>::SIZE; ++i)
	{
		v[i]/= norm_value;
	}
}




/*
 *  End of definitions
 */

} // namespace geometry
} // namespace cgogn

#endif // GEOMETRY_VEC_OPERATIONS_H_

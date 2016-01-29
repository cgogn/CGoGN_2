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

#ifndef GEOMETRY_VEC_H_
#define GEOMETRY_VEC_H_

#include <array>
#include <cmath>
#include <vector>
#include <initializer_list>

#include <core/utils/name_types.h>
#include <core/utils/assert.h>

#include <geometry/dll.h>

namespace cgogn
{

namespace geometry
{

/**
 * @brief The Vec_T class, designed to mimic Eigen's Vector interface.
 * The template parameter is the kind of container you want to use to store your vec (std::array)
 * The Container class must provide an iterator nested type and the following methods :
 *  -operator []
 *  -begin()
 *  -end()
 * Its size has to be known at compile time
 * The struct cgogn::geometry::vector_traits has to be specialized for the Container class (see geometry_traits.h).
 */
template<class Container>
class Vec_T
{
public:

	using Self = Vec_T<Container>;
	using Scalar = typename std::remove_cv< typename std::remove_reference<decltype(Container()[0ul])>::type >::type;

	Vec_T() = default;
	Vec_T(const Self&v) = default;
	Self& operator=(const Self& v) = default;

	template<typename...Args>
	inline Vec_T(Args... a) : data_{Scalar(a)...} {}

	inline Scalar& operator[](std::size_t i)
	{
		return data_[i];
	}

	inline const Scalar& operator[](std::size_t i) const
	{
		return data_[i];
	}

	inline bool operator==(const Self& v) const
	{
		auto it_this = data_.begin();
		for (auto c : v.data_)
			if ( *(it_this++) != c)
				return false;
		return true;
	}

	inline const Self operator-() const
	{
		Self res(*this);
		for(auto& c : res.data_)
			c = -c;
		return res;
	}

	inline void operator+=(const Self& v)
	{
		auto it_this = data_.begin();
		for (auto c : v.data_)
			*(it_this++) += c;
	}

	inline void operator-=(const Self& v)
	{
		auto it_this = data_.begin();
		for (auto c : v.data_)
			*(it_this++) -= c;
	}

	inline void operator/=(Scalar r)
	{
		for (auto& c : data_)
			c /= r;
	}

	inline void operator*=(Scalar r)
	{
		for (auto& c : data_)
			c *= r;
	}

	inline const Self operator+(const Self& v) const
	{
		Self res;
		auto it_res = res.data_.begin();
		auto it_v = v.data_.begin();
		for (auto c : data_)
			*(it_res++) = c + (*it_v++);
		return res;
	}

	inline const Self operator-(const Self& v) const
	{
		Self res;
		auto it_res = res.data_.begin();
		auto it_v = v.data_.begin();
		for (auto c : data_)
			*(it_res++) = c - *(it_v++);
		return res;
	}

	inline friend const Self operator*(const Self& v, Scalar r)
	{
		Self res(v);
		for (auto& c : res.data_)
			c *= r;
		return res;
	}

	inline friend const Self operator*(Scalar r, const Self& v)
	{
		return v * r;
	}

	inline friend const Self operator/(const Self& v, Scalar r)
	{
		Self res(v);
		for (auto& c : res.data_)
			c /= r;
		return res;
	}

	inline friend const Self operator/(Scalar r, const Self& v)
	{
		return v / r;
	}


	inline const Scalar dot(const Self& v) const
	{
		Scalar r{0};
		auto it_v = v.data_.begin();
		for (auto c : data_)
			r += c * (*it_v++);
		return r;
	}

	inline const Self cross(const Self& v) const
	{
		Self res;
		res[0] = data_[1] * v[2] - data_[2] * v[1];
		res[1] = data_[2] * v[0] - data_[0] * v[2];
		res[2] = data_[0] * v[1] - data_[1] * v[0];
		return res;
	}

	inline const Scalar squaredNorm() const
	{
		Scalar r{0};
		for (auto c : data_)
			r += c * c;
		return r;
	}

	inline const Scalar norm() const
	{
		return std::sqrt(this->squaredNorm());
	}

	inline void normalize()
	{
		const Scalar norm_value = this->norm();
		for (auto& c : data_)
			c /= norm_value;
	}

	inline void setZero()
	{
		for (auto& c : data_)
			c = 0;
	}

	static std::string cgogn_name_of_type()
	{
		return std::string("cgogn::geometry::Vec_T<") + cgogn::name_of_type(Container()) + std::string(">");
	}

	const Container& to_container() const
	{
		return data_;
	}

private:

	Container data_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(GEOMETRY_VEC_CPP_))
extern template class CGOGN_GEOMETRY_API Vec_T<std::array<double,3>>;
extern template class CGOGN_GEOMETRY_API Vec_T<std::array<float,3>>;
extern template class CGOGN_GEOMETRY_API Vec_T<std::vector<float>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(GEOMETRY_VEC_CPP_))

} // namespace geometry

} // namespace cgogn

#endif // GEOMETRY_VEC_H_

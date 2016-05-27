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

#ifndef CGOGN_GEOMETRY_TYPES_VEC_H_
#define CGOGN_GEOMETRY_TYPES_VEC_H_

#include <array>
#include <cmath>
#include <vector>
#include <initializer_list>

#include <cgogn/core/utils/name_types.h>
#include <cgogn/core/utils/assert.h>

#include <cgogn/geometry/dll.h>

namespace cgogn
{

namespace geometry
{

/**
 * @brief The Vec_T class, designed to mimic Eigen's Vector interface.
 * The template parameter is the kind of container you want to use to store your vec (std::array)
 * The Container class must provide an iterator nested type and the following methods :
 *  -operator[]
 *  -begin()
 *  -end()
 * Its size has to be known at compile time
 * The struct cgogn::geometry::vector_traits has to be specialized for the Container class (see geometry_traits.h).
 */
template <class Container>
class Vec_T
{
public:

	using Self = Vec_T<Container>;
	using Scalar = typename std::remove_cv< typename std::remove_reference<decltype(Container()[0ul])>::type >::type;

	inline Vec_T(Scalar a)
	{
		cgogn_message_assert(std::tuple_size<Container>::value >= 1, "wrong contructor, too many data");
		data_[0] = a;
	}

	inline Vec_T(Scalar a, Scalar b)
	{
		cgogn_message_assert(std::tuple_size<Container>::value >= 2, "wrong contructor, too many data");
		data_[0] = a; data_[1] = b;
	}

	inline Vec_T(Scalar a, Scalar b, Scalar c)
	{
		cgogn_message_assert(std::tuple_size<Container>::value >= 3, "wrong contructor, too many data");
		data_[0] = a; data_[1] = b; data_[2] = c;
	}

	inline Vec_T(Scalar a, Scalar b, Scalar c, Scalar d)
	{
		cgogn_message_assert(std::tuple_size<Container>::value >= 4, "wrong contructor, too many data");
		data_[0] = a; data_[1] = b; data_[2] = c; data_[3] = d;
	}

	inline Vec_T() : data_() {}

	//template <typename... Args>
	//inline Vec_T(Args... a) : data_({ { std::forward<Args>(a)... } })
	//{}

	Vec_T(const Self&v) = default;
	Self& operator=(const Self& v) = default;

	inline const Scalar* data() const
	{
		return data_.data();
	}

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

	inline friend std::ostream& operator<<(std::ostream& o, const Self& v)
	{
		o << "(";
		for (std::size_t i = 0ul ; i < std::tuple_size<Container>::value -1ul; ++i )
			o << v.data_[i] << ",";
		o << v.data_[std::tuple_size<Container>::value -1ul];
		o << ")";
		return o;
	}

	inline std::size_t size() const
	{
		return data_.size();
	}
	inline auto begin() const -> decltype(std::declval<const Container>().begin())
	{
		return data_.begin();
	}

	inline auto begin() -> decltype(std::declval<Container>().begin())
	{
		return data_.begin();
	}

	inline auto end() const -> decltype(std::declval<const Container>().end())
	{
		return data_.end();
	}

	inline auto end() -> decltype(std::declval<Container>().end())
	{
		return data_.end();
	}

private:

	Container data_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_TYPES_VEC_CPP_))
extern template class CGOGN_GEOMETRY_API Vec_T<std::array<float32,3>>;
extern template class CGOGN_GEOMETRY_API Vec_T<std::array<float64,3>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_TYPES_VEC_CPP_))

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_VEC_H_

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

#ifndef CGOGN_GEOMETRY_TYPES_AABB_H_
#define CGOGN_GEOMETRY_TYPES_AABB_H_

#include <type_traits>
#include <array>
#include <initializer_list>

#include <cgogn/core/utils/numerics.h>

#include <cgogn/geometry/cgogn_geometry_export.h>
#include <cgogn/geometry/types/geometry_traits.h>

namespace cgogn
{

namespace geometry
{

/**
 * Axis-Aligned Bounding Box
 */
template <typename VEC_T>
class AABB
{
public:

	using Vec = VEC_T;
	using Scalar = ScalarOf<Vec>;
	using Self = AABB<Vec>;
	static const uint32 dim_ = vector_traits<Vec>::SIZE;
	// https://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
	static const bool eigen_make_aligned = std::is_same<Eigen::AlignedVector3<Scalar>, Vec>::value;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(eigen_make_aligned)

private:

	Vec p_min_;
	Vec p_max_;

public:
	/**********************************************/
	/*                CONSTRUCTORS                */
	/**********************************************/

	AABB()
	{
		for(uint32 i = 0; i < dim_; ++i)
		{
			p_min_[i] = std::numeric_limits<Scalar>::max();
			p_max_[i] = std::numeric_limits<Scalar>::lowest();
		}
	}

	// initialize the bounding box with one first point
	AABB(const Vec& p) :
		p_min_(p),
		p_max_(p)
	{}

	// initialize the bounding box with bounding points
	AABB(const Vec& min, const Vec& max) :
		p_min_(min),
		p_max_(max)
	{}

	/**********************************************/
	/*                 ACCESSORS                  */
	/**********************************************/

	inline Vec& min()
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		return p_min_;
	}

	inline const Vec& min() const
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		return p_min_;
	}

	inline Vec& max()
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		return p_max_;
	}

	inline const Vec& max() const
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		return p_max_;
	}

	inline Scalar size(uint32 coord) const
	{
		cgogn_assert(is_initialized() && coord < dim_);
		return p_max_[coord] - p_min_[coord];
	}

	inline Scalar max_size() const
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		Scalar max = p_max_[0] - p_min_[0];
		for(uint32 i = 1; i < dim_; ++i)
		{
			Scalar size = p_max_[i] - p_min_[i];
			if(size > max)
				max = size;
		}
		return max;
	}

	inline Scalar min_size() const
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		Scalar min = p_max_[0] - p_min_[0];
		for(uint32 i = 1; i < dim_; ++i)
		{
			Scalar size = p_max_[i] - p_min_[i];
			if(size < min)
				min = size;
		}
		return min;
	}

	inline Vec diag() const
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		return p_max_ - p_min_;
	}

	inline Scalar diag_size()  const
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		return Scalar(diag().norm());
	}

	inline Vec center() const
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		Vec center = (p_max_ + p_min_) / Scalar(2);
		return center;
	}

	inline bool is_initialized() const
	{
		bool is_init = false;
		for(uint32 i = 0; !is_init && i < dim_; ++i)
		{
			is_init = p_min_[i] != std::numeric_limits<Scalar>::max() ||
					p_max_[i] != std::numeric_limits<Scalar>::lowest();
		}

		return is_init;
	}

	// reinitialize the axis-aligned bounding box
	inline void reset()
	{	
		for(uint32 i = 0; i < dim_; ++i)
		{
			p_min_[i] = std::numeric_limits<Scalar>::max();
			p_max_[i] = std::numeric_limits<Scalar>::lowest();
		}
	}

	// add a point to the axis-aligned bounding box
	inline void add_point(const Vec& p)
	{
		for(uint32 i = 0; i < dim_; ++i)
		{
			if(p[i] < p_min_[i])
				p_min_[i] = p[i];
			if(p[i] > p_max_[i])
				p_max_[i] = p[i];
		}
	}

	inline void add_point(std::initializer_list<Scalar> list)
	{
		cgogn_message_assert(list.size() <= dim_, "Dimension of point greaer tyhan Axis-Aligned Bounding box dimension");
		uint32 i = 0;
		for(const Scalar elem : list )
		{
			if(elem < p_min_[i])
				p_min_[i] = elem;
			if(elem > p_max_[i])
				p_max_[i] = elem;

			++i;
		}
	}

	// return true if bb intersects the axis-aligned bounding box
	bool intersects(const AABB<Vec>& bb) const
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		Vec bbmin = bb.min();
		Vec bbmax = bb.max();
		for(uint32 i = 0; i < dim_; ++i)
		{
			if(p_max_[i] < bbmin[i])
				return false;
			if(p_min_[i] > bbmax[i])
				return false;
		}
		return true;
	}

	// fusion with the given bounding box
	void fusion(const AABB<Vec>& bb)
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		Vec bbmin = bb.min();
		Vec bbmax = bb.max();
		for (uint32 i = 0; i < dim_; ++i)
		{
			if (bbmin[i] < p_min_[i])
				p_min_[i] = bbmin[i];
			if (bbmax[i] > p_max_[i])
				p_max_[i] = bbmax[i];
		}
	}

	// return true if the point belongs strictly to a bounding box
	bool contains(const Vec& p) const
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		for (uint32 i = 0; i < dim_; ++i)
		{
			if (p_min_[i] > p[i])
				return false;
			if (p[i] > p_max_[i])
				return false;
		}
		return true;
	}

	bool contains(std::initializer_list<Scalar> list) const
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		cgogn_message_assert(list.size() <= dim_, "Dimension of point greaer tyhan Axis-Aligned Bounding box dimension");
		uint32 i = 0;
		for(const Scalar elem : list )
		{
			if (p_min_[i] > elem)
				return false;
			if (elem > p_max_[i])
				return false;
			++i;
		}
		return true;
	}

	// return true if the bounding box belongs strictly to a bounding box
	bool contains(const AABB<Vec>& bb) const
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		return this->contains(bb.min()) && this->contains(bb.max());
	}

	// scale the bounding box
	void scale(Scalar size)
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		p_min_ *= size;
		p_max_ *= size;
	}

	// 0-centered scale of the bounding box
	void centered_scale(Scalar size)
	{
		cgogn_message_assert(is_initialized(), "Axis-Aligned Bounding box not initialized");
		Vec center = (p_min_ + p_max_) / Scalar(2);
		p_min_ = ((p_min_ - center) * size) + center;
		p_max_ = ((p_max_ - center) * size) + center;
	}

	/// \brief Test if a ray intersects an axis-aligned box
	/// \tparam VEC3 the domain of the box. Has to be of dimension 3
	template <bool B = true>
	auto ray_intersect(const Vec& P, const Vec& V) const
	-> typename std::enable_if<B && is_dim_of<Vec, 3>::value, bool>::type
	{
		if (!cgogn::almost_equal_relative(V[2], Scalar(0)))
		{
			Vec Q = P + ((p_min_[2] - P[2]) / V[2]) * V;
			if ((Q[0] < p_max_[0]) && (Q[0] > p_min_[0]) && (Q[1] < p_max_[1]) && (Q[1] > p_min_[1]))
				return true;
			Q = P + ((p_max_[2] - P[2]) / V[2]) * V;
			if ((Q[0] < p_max_[0]) && (Q[0] > p_min_[0]) && (Q[1] < p_max_[1]) && (Q[1] > p_min_[1]))
				return true;
		}

		if (!cgogn::almost_equal_relative(V[1], Scalar(0)))
		{
			Vec Q = P + ((p_min_[1] - P[1]) / V[1]) * V;
			if ((Q[0] < p_max_[0]) && (Q[0] > p_min_[0]) && (Q[2] < p_max_[2]) && (Q[2] > p_min_[2]))
				return true;
			Q = P + ((p_max_[1] - P[1]) / V[1]) * V;
			if ((Q[0] < p_max_[0]) && (Q[0] > p_min_[0]) && (Q[2] < p_max_[2]) && (Q[2] > p_min_[2]))
				return true;
		}

		if (!cgogn::almost_equal_relative(V[0], Scalar(0)))
		{
			Vec Q = P + ((p_min_[0] - P[0]) / V[0]) * V;
			if ((Q[1] < p_max_[1]) && (Q[1] > p_min_[1]) && (Q[2] < p_max_[2]) && (Q[2] > p_min_[2]))
				return true;
			Q = P + ((p_max_[0] - P[0]) / V[0]) * V;
			if ((Q[1] < p_max_[1]) && (Q[1] > p_min_[1]) && (Q[2] < p_max_[2]) && (Q[2] > p_min_[2]))
				return true;
		}

		return false;
	}

	static std::string cgogn_name_of_type()
	{
		return std::string("cgogn::geometry::AABB<") + name_of_type(Vec()) + std::string(">");
	}
};

template <typename VEC_T>
std::ostream& operator<<(std::ostream& out, const AABB<VEC_T>& bb)
{
	out << bb.min() << " " << bb.max();
	return out;
}

template <typename VEC_T>
std::istream& operator>>(std::istream& in, AABB<VEC_T>& bb)
{
	in >> bb.min() >> bb.max();
	return in;
}


template <typename VEC_T, typename std::enable_if<is_dim_of<VEC_T, 3>::value, VEC_T>::type* = nullptr>
VEC_T corner(const AABB<VEC_T>& aabb, uint32 id)
{
	VEC_T min = aabb.min();
	VEC_T max = aabb.max();

	switch(id)
	{
		case 0: return min; break;
		case 1: return VEC_T(min[0], min[1], max[2]); break;
		case 2: return VEC_T(min[0], max[1], min[2]); break;
		case 3: return VEC_T(min[0], max[1], max[2]); break;
		case 4: return VEC_T(max[0], min[1], min[2]); break;
		case 5: return VEC_T(max[0], min[1], max[2]); break;
		case 6: return VEC_T(max[0], max[1], min[2]); break;
		case 7: return max; break;
		default: cgogn_assert_not_reached("id not in range [0-7]"); break;
	}
}

/// \brief Computes the smallest axis-aligned box that encloses two AABBs.
/// \tparam VEC_T the domain of the box
/// \param[out] target the smallest axis-aligned box that encloses \p b1 and \p b2
/// \param[in] b1 first box
/// \param[in] b2 second box
template <typename VEC_T>
inline void aabb_union(AABB<VEC_T>& target, const AABB<VEC_T>& b1, const AABB<VEC_T>& b2)
{
	cgogn_assert(target.dim_ == b1.dim_);
	cgogn_assert(b1.dim_ == b2.dim_);

	for(uint32 i = 0; i < b1.dim_; ++i)
	{
		target.min(i, std::min(b1.min()[i], b2.min()[i]));
		target.max(i, std::max(b1.max()[i], b2.max()[i]));
	}
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_EXTERNAL_TEMPLATES_CPP_))
extern template class CGOGN_GEOMETRY_EXPORT AABB<Eigen::Vector3d>;
extern template class CGOGN_GEOMETRY_EXPORT AABB<Eigen::Vector3f>;
extern template class CGOGN_GEOMETRY_EXPORT AABB<Vec_T<std::array<float32, 3>>>;
extern template class CGOGN_GEOMETRY_EXPORT AABB<Vec_T<std::array<float64,3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_EXTERNAL_TEMPLATES_CPP_))

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_AABB_H_

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

#include <cgogn/core/utils/numerics.h>

#include <cgogn/geometry/dll.h>
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
	using Scalar = typename vector_traits<Vec>::Scalar;
	using Self = AABB<Vec>;
	static const uint32 dim_ = vector_traits<Vec>::SIZE;

private:

	bool initialized_;
	Vec p_min_;
	Vec p_max_;

public:
	/**********************************************/
	/*                CONSTRUCTORS                */
	/**********************************************/

	AABB() :
		initialized_(false)
	{}

	// initialize the bounding box with one first point
	AABB(const Vec& p) :
		initialized_(true),
		p_min_(p),
		p_max_(p)
	{}

	/**********************************************/
	/*                 ACCESSORS                  */
	/**********************************************/

	Vec& min()
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		return p_min_;
	}

	const Vec& min() const
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		return p_min_;
	}

	Vec& max()
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		return p_max_;
	}

	const Vec& max() const
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		return p_max_;
	}

	Scalar size(uint32 coord) const
	{
		cgogn_assert(initialized_ && coord < dim_);
		return p_max_[coord] - p_min_[coord];
	}

	Scalar max_size() const
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		Scalar max = p_max_[0] - p_min_[0];
		for(uint32 i = 1; i < dim_; ++i)
		{
			Scalar size = p_max_[i] - p_min_[i];
			if(size > max)
				max = size;
		}
		return max;
	}

	Scalar min_size() const
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		Scalar min = p_max_[0] - p_min_[0];
		for(uint32 i = 1; i < dim_; ++i)
		{
			Scalar size = p_max_[i] - p_min_[i];
			if(size < min)
				min = size;
		}
		return min;
	}

	Vec diag() const
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		return p_max_ - p_min_;
	}

	Scalar diag_size()  const
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		return Scalar((p_max_ - p_min_).norm());
	}

	Vec center() const
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		Vec center = (p_max_ + p_min_) / Scalar(2);
		return center;
	}

	bool is_initialized() const
	{
		return initialized_;

	}

	// reinitialize the axis-aligned bounding box
	void reset()
	{
		initialized_ = false;
	}

	// add a point to the axis-aligned bounding box
	void add_point(const Vec& p)
	{
		if(!initialized_)
		{
			p_min_ = p;
			p_max_ = p;
			initialized_ = true;
		}
		else
		{
			for(uint32 i = 0; i < dim_; ++i)
			{
				if(p[i] < p_min_[i])
					p_min_[i] = p[i];
				if(p[i] > p_max_[i])
					p_max_[i] = p[i];
			}
		}
	}

	// return true if bb intersects the axis-aligned bounding box
	bool intersects(const AABB<Vec>& bb) const
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
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
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		Vec bbmin = bb.min();
		Vec bbmax = bb.max();
		for(uint32 i = 0; i < dim_; ++i)
		{
			if(bbmin[i] < p_min_[i])
				p_min_[i] = bbmin[i];
			if(bbmax[i] > p_max_[i])
				p_max_[i] = bbmax[i];
		}
	}

	// return true if the point belongs strictly to a bounding box
	bool contains(const Vec& p) const
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		for(uint32 i = 0; i < dim_; ++i)
		{
			if(p_min_[i] > p[i])
				return false;
			if(p[i] > p_max_[i])
				return false;
		}
			return true;
	}


	// return true if the bounding box belongs strictly to a bounding box
	bool contains(const AABB<Vec>& bb) const
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		return this->contains(bb.min()) && this->contains(bb.max());
	}

	// scale the bounding box
	void scale(Scalar size)
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		p_min_ *= size;
		p_max_ *= size;
	}

	// 0-centered scale of the bounding box
	void centered_scale(Scalar size)
	{
		cgogn_message_assert(initialized_, "Axis-Aligned Bounding box not initialized");
		Vec center = (p_min_ + p_max_) / Scalar(2);
		p_min_ = ((p_min_ - center) * size) + center;
		p_max_ = ((p_max_ - center) * size) + center;
	}

//	// test if bb is intersected by a ray
//	bool ray_intersect(const Vec& P, const Vec& V) const
//	{
//		if (!cgogn::almost_equal_relative(V[2], Scalar(0)))
//		{
//			Vec Q = P + ((p_min_[2] - P[2]) / V[2]) * V;
//			if ((Q[0] < p_max_[0]) && (Q[0] > p_min_[0]) && (Q[1] < p_max_[1]) && (Q[1] > p_min_[1]))
//				return true;
//			Q = P + ((p_max_[2] - P[2]) / V[2]) * V;
//			if ((Q[0] < p_max_[0]) && (Q[0] > p_min_[0]) && (Q[1] < p_max_[1]) && (Q[1] > p_min_[1]))
//				return true;
//		}

//		if (!cgogn::almost_equal_relative(V[1], Scalar(0)))
//		{
//			Vec Q = P + ((p_min_[1] - P[1]) / V[1]) * V;
//			if ((Q[0] < p_max_[0]) && (Q[0] > p_min_[0]) && (Q[2] < p_max_[2]) && (Q[2] > p_min_[2]))
//				return true;
//			Q = P + ((p_max_[1] - P[1]) / V[1]) * V;
//			if ((Q[0] < p_max_[0]) && (Q[0] > p_min_[0]) && (Q[2] < p_max_[2]) && (Q[2] > p_min_[2]))
//				return true;
//		}

//		if (!cgogn::almost_equal_relative(V[0], Scalar(0)))
//		{
//			Vec Q = P + ((p_min_[0] - P[0]) / V[0]) * V;
//			if ((Q[1] < p_max_[1]) && (Q[1] > p_min_[1]) && (Q[2] < p_max_[2]) && (Q[2] > p_min_[2]))
//				return true;
//			Q = P + ((p_max_[0] - P[0]) / V[0]) * V;
//			if ((Q[1] < p_max_[1]) && (Q[1] > p_min_[1]) && (Q[2] < p_max_[2]) && (Q[2] > p_min_[2]))
//				return true;
//		}

//		return false;
//	}

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

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_TYPES_AABB_CPP_))
extern template class CGOGN_GEOMETRY_API AABB<Eigen::Vector3d>;
extern template class CGOGN_GEOMETRY_API AABB<Eigen::Vector3f>;
extern template class CGOGN_GEOMETRY_API AABB<Vec_T<std::array<float32, 3>>>;
extern template class CGOGN_GEOMETRY_API AABB<Vec_T<std::array<float64,3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_GEOMETRY_TYPES_AABB_CPP_))

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_AABB_H_

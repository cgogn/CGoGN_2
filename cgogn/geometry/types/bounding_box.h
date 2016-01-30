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

#ifndef GEOMETRY_TYPES_BOUNDING_BOX_H_
#define GEOMETRY_TYPES_BOUNDING_BOX_H_

#include <type_traits>
#include <array>

#include <core/utils/precision.h>

#include <geometry/dll.h>
#include <geometry/types/geometry_traits.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC_T>
class BoundingBox
{
	static_assert(vector_traits<VEC_T>::SIZE == 3ul, "The size of the vector must be equal to 3.");

public:

	using Vec = VEC_T;
	using Scalar = typename vector_traits<Vec>::Scalar;
	using Self = BoundingBox<Vec>;
	static const unsigned int dim_ = vector_traits<Vec>::SIZE;

private:

	bool initialized_;
	Vec p_min_;
	Vec p_max_;

public:
	/**********************************************/
	/*                CONSTRUCTORS                */
	/**********************************************/

	BoundingBox() :
		initialized_(false)
	{}

	// initialize the bounding box with one first point
	BoundingBox(const Vec& p) :
		initialized_(true),
		p_min_(p),
		p_max_(p)
	{}

	/**********************************************/
	/*                 ACCESSORS                  */
	/**********************************************/

	Vec& min()
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		return p_min_;
	}

	const Vec& min() const
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		return p_min_;
	}

	Vec& max()
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		return p_max_;
	}

	const Vec& max() const
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		return p_max_;
	}

	Scalar size(unsigned int coord) const
	{
		cgogn_assert(initialized_ && coord < dim_);
		return p_max_[coord] - p_min_[coord];
	}

	Scalar max_size() const
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		Scalar max = p_max_[0] - p_min_[0];
		for(unsigned int i = 1; i < dim_; ++i)
		{
			Scalar size = p_max_[i] - p_min_[i];
			if(size > max)
				max = size;
		}
		return max;
	}

	Scalar min_size() const
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		Scalar min = p_max_[0] - p_min_[0];
		for(unsigned int i = 1; i < dim_; ++i)
		{
			Scalar size = p_max_[i] - p_min_[i];
			if(size < min)
				min = size;
		}
		return min;
	}

	Vec diag() const
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		return p_max_ - p_min_;
	}

	Scalar diag_size()  const
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		return Scalar((p_max_ - p_min_).norm());
	}

	Vec center() const
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		Vec center = (p_max_ + p_min_) / Scalar(2);
		return center;
	}

	bool is_initialized() const;

	// reinitialize the bounding box
	void reset()
	{
		initialized_ = false;
	}

	// add a point to the bounding box
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
			for(unsigned int i = 0; i < dim_; ++i)
			{
				if(p[i] < p_min_[i])
					p_min_[i] = p[i];
				if(p[i] > p_max_[i])
					p_max_[i] = p[i];
			}
		}
	}

	// return true if bb intersects the bounding box
	bool intersects(const BoundingBox<Vec>& bb) const
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		Vec bbmin = bb.min();
		Vec bbmax = bb.max();
		for(unsigned int i = 0; i < dim_; ++i)
		{
			if(p_max_[i] < bbmin[i])
				return false;
			if(p_min_[i] > bbmax[i])
				return false;
		}
		return true;
	}

	// fusion with the given bounding box
	void fusion(const BoundingBox<Vec>& bb)
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		Vec bbmin = bb.min();
		Vec bbmax = bb.max();
		for(unsigned int i = 0; i < dim_; ++i)
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
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		for(unsigned int i = 0; i < dim_; ++i)
		{
			if(p_min_[i] > p[i])
				return false;
			if(p[i] > p_max_[i])
				return false;
		}
			return true;
	}


	// return true if the segment belongs strictly to a bounding box
	bool contains(const Vec& a, const Vec& b) const
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");

	#define LEFT 'l'
	#define RIGHT 'r'
	#define MIDDLE 'm'

		// Algorithm from Graphic Gems
		// modified to test segment
		Vec dir(b - a); // ray

		bool inside = true;
//		std::vector<char> quadrant(p_min_.dimension());
		char quadrant[dim_];

		Vec candidatePlane;

		// Find candidate planes; this loop can be avoided if
		// rays cast all from the eye(assume perpsective view)
		for (unsigned int i = 0; i < dim_; i++)
		{
			if (a[i] < p_min_[i])
			{
				quadrant[i] = LEFT;
				candidatePlane[i] = p_min_[i];
				inside = false;
			}
			else if (a[i] > p_max_[i])
			{
				quadrant[i] = RIGHT;
				candidatePlane[i] = p_max_[i];
				inside = false;
			}
			else
				quadrant[i] = MIDDLE;
		}

		// Ray origin inside bounding box
		if (inside)
			return true;

		Vec maxT;
		Vec coord;			/* hit point */
		/* Calculate T distances to candidate planes */
		for(unsigned int i = 0u; i < dim_; i++)
		{
			if (quadrant[i] != MIDDLE && dir[i] != 0)
				maxT[i] = (candidatePlane[i] - a[i]) / dir[i];
			else
				maxT[i] = -1;
		}

	#undef LEFT
	#undef RIGHT
	#undef MIDDLE

		// Get largest of the maxT's for final choice of intersection
		unsigned int whichPlane = 0u;
		for(unsigned int i = 1u; i < dim_; i++)
			if (maxT[whichPlane] < maxT[i])
				whichPlane = i;

		/* Check final candidate actually inside box */
		if (maxT[whichPlane] < 0.)
			return false;
		for (unsigned int i = 0u; i < dim_; i++)
		{
			if (whichPlane != i)
			{
				coord[i] = a[i] + maxT[whichPlane] * dir[i];
				if (coord[i] < p_min_[i] || coord[i] > p_max_[i])
					return false;
				else
					coord[i] = candidatePlane[i];
			}
		}

		return (coord - b).dot(a - b); // intersection in segment ?
	}

	// return true if the bounding box belongs strictly to a bounding box
	bool contains(const BoundingBox<Vec>& bb) const
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		return this->contains(bb.min()) && this->contains(bb.max());
	}

	// scale the bounding box
	void scale(Scalar size)
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		p_min_ *= size;
		p_max_ *= size;
	}

	// 0-centered scale of the bounding box
	void centered_scale(Scalar size)
	{
		cgogn_message_assert(initialized_, "Bounding box not initialized");
		Vec center = (p_min_ + p_max_) / Scalar(2);
		p_min_ = ((p_min_ - center) * size) + center;
		p_max_ = ((p_max_ - center) * size) + center;
	}

	// test if bb is intersected by a ray
	bool ray_intersect(const Vec& P, const Vec& V) const
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
		return std::string("cgogn::geometry::BoundingBox<") + name_of_type(Vec()) + std::string(">");
	}
};

template <typename VEC_T>
std::ostream& operator<<(std::ostream& out, const BoundingBox<VEC_T>& bb)
{
	out << bb.min() << " " << bb.max();
	return out;
}

template <typename VEC_T>
std::istream& operator>>(std::istream& in, BoundingBox<VEC_T>& bb)
{
	in >> bb.min() >> bb.max();
	return in;
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(GEOMETRY_BOUNDING_BOX_CPP_))
extern template class CGOGN_GEOMETRY_API BoundingBox<Eigen::Vector3d>;
extern template class CGOGN_GEOMETRY_API BoundingBox<Eigen::Vector3f>;
extern template class CGOGN_GEOMETRY_API BoundingBox<Vec_T<std::array<double, 3>>>;
extern template class CGOGN_GEOMETRY_API BoundingBox<Vec_T<std::array<float,3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(GEOMETRY_BOUNDING_BOX_CPP_))

} // namespace geometry

} // namespace cgogn

#endif // GEOMETRY_TYPES_BOUNDING_BOX_H_

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

#ifndef CGOGN_GEOMETRY_ALGOS_EAR_TRIANGULATION_H_
#define CGOGN_GEOMETRY_ALGOS_EAR_TRIANGULATION_H_

#include <set>

#include <cgogn/geometry/types/geometry_traits.h>
#include <cgogn/geometry/algos/normal.h>
#include <cgogn/geometry/functions/inclusion.h>

namespace cgogn
{

namespace geometry
{

template <typename VEC3, typename MAP>
class EarTriangulation
{
	using Vertex = typename MAP::Vertex;
	using Face   = typename MAP::Face;
	using Scalar = typename vector_traits<VEC3>::Scalar;

	class VertexPoly
	{
	public:

		using VPMS = std::multiset<VertexPoly*, bool(*)(VertexPoly*,VertexPoly*)>;

		int32 id;
		Vertex vert_;
		Scalar value_;
		Scalar length_;
		VertexPoly* prev_;
		VertexPoly* next_;
		typename VPMS::iterator ear_;

		VertexPoly(Vertex ve, Scalar va, Scalar l, VertexPoly* p = nullptr) : vert_(ve), value_(va), length_(l), prev_(p), next_(nullptr)
		{
			if (prev_ != nullptr)
				prev_->next_ = this;
		}

		static void close(VertexPoly* first, VertexPoly* last)
		{
			last->next_ = first;
			first->prev_ = last;
		}

		static VertexPoly* erase(VertexPoly* vp)
		{
			VertexPoly* tmp = vp->prev_;
			tmp->next_ = vp->next_;
			vp->next_->prev_ = tmp;
			delete vp;
			return tmp;
		}
	};

	using VPMS = typename VertexPoly::VPMS;

	// normal to polygon (for orientation of angles)
	VEC3 normalPoly_;

	// ref on map
	MAP& map_;

	// ref on position attribute
	const typename MAP::template VertexAttribute<VEC3>& positions_;

	// map of ears
	VPMS ears_;

	// is current polygin convex
	bool convex_;

	// number off vertices
	uint32 nb_verts_;

	// initial face
	Face face_;

	inline static bool cmp_VP(VertexPoly* lhs, VertexPoly* rhs)
	{
		if (std::abs(lhs->value_ - rhs->value_) < 0.2f)
			return lhs->length_ < rhs->length_;
		return lhs->value_ < rhs->value_;
	}

	void recompute_2_ears(VertexPoly* vp)
	{
		VertexPoly* vprev = vp->prev_;
		VertexPoly* vp2 = vp->next_;
		VertexPoly* vnext = vp2->next_;
		const VEC3& Ta = positions_[vp->vert_];
		const VEC3& Tb = positions_[vp2->vert_];
		const VEC3& Tc = positions_[vprev->vert_];
		const VEC3& Td = positions_[vnext->vert_];

		// compute angle
		VEC3 v1 = Tb - Ta;
		VEC3 v2 = Tc - Ta;
		VEC3 v3 = Td - Tb;

		v1.normalize();
		v2.normalize();
		v3.normalize();

		Scalar dotpr1 = std::acos(v1.dot(v2)) / Scalar(M_PI_2);
		Scalar dotpr2 = std::acos(-(v1.dot(v3))) / Scalar(M_PI_2);

		if (!convex_)	// if convex no need to test if vertex is an ear (yes)
		{
			VEC3 nv1 = v1.cross(v2);
			VEC3 nv2 = v1.cross(v3);

			if (nv1.dot(normalPoly_) < 0.0)
				dotpr1 = 10.0f - dotpr1;// not an ear (concave)
			if (nv2.dot(normalPoly_) < 0.0)
				dotpr2 = 10.0f - dotpr2;// not an ear (concave)

			bool finished = (dotpr1 >= 5.0f) && (dotpr2 >= 5.0f);
			for (auto it = ears_.rbegin(); (!finished) && (it != ears_.rend()) && ((*it)->value_ > 5.0f); ++it)
			{
				Vertex id = (*it)->vert_;
				const VEC3& P = positions_[id];

				if ((dotpr1 < 5.0f) && (id.dart != vprev->vert_.dart))
					if (in_triangle(P, normalPoly_, Tb, Tc, Ta))
						dotpr1 = 5.0f; // not an ear !

				if ((dotpr2 < 5.0f) && (id.dart != vnext->vert_.dart) )
					if (in_triangle(P, normalPoly_,Td,Ta,Tb))
						dotpr2 = 5.0f; // not an ear !

				finished = (dotpr1 >= 5.0f) && (dotpr2 >= 5.0f);
			}
		}

		vp->value_ = dotpr1;
		vp->length_ = Scalar((Tb-Tc).squaredNorm());
		vp->ear_ = ears_.insert(vp);
		vp2->value_ = dotpr2;
		vp->length_ = Scalar((Td-Ta).squaredNorm());
		vp2->ear_ = ears_.insert(vp2);

		// polygon if convex only if all vertices have convex angle (last have ...)
		convex_ = (*(ears_.rbegin()))->value_ < 5.0f;
	}

	Scalar ear_angle(const VEC3& P1, const VEC3& P2, const VEC3& P3)
	{
		VEC3 v1 = P1 - P2;
		VEC3 v2 = P3 - P2;
		v1.normalize();
		v2.normalize();

		Scalar dotpr = std::acos(v1.dot(v2)) / Scalar(M_PI_2);

		VEC3 vn = v1.cross(v2);
		if (vn.dot(normalPoly_) > 0.0f)
			dotpr = 10.0f - dotpr; 	// not an ear (concave, store at the end for optimized use for intersections)

		return dotpr;
	}

	bool ear_intersection(VertexPoly* vp)
	{
		VertexPoly* endV = vp->prev_;
		VertexPoly* curr = vp->next_;
		const VEC3& Ta = positions_[vp->vert_];
		const VEC3& Tb = positions_[curr->vert_];
		const VEC3& Tc = positions_[endV->vert_];
		curr = curr->next_;

		while (curr != endV)
		{
			if (in_triangle(positions_[curr->vert_], normalPoly_, Tb, Tc, Ta))
			{
				vp->value_ = 5.0f; // not an ear !
				return false;
			}
			curr = curr->next_;
		}
		return true;
	}

public:

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(EarTriangulation);

	/**
	 * @brief EarTriangulation constructor
	 * @param map ref on map
	 * @param f the face to tringulate
	 * @param position attribute of position to use
	 */
	EarTriangulation(MAP& map, const typename MAP::Face f, const typename MAP::template VertexAttribute<VEC3>& position) :
		map_(map),
		positions_(position),
		ears_(cmp_VP)
	{
		if (map_.codegree(f) == 3)
		{
			face_ = f;
			nb_verts_ = 3;
			return;
		}

		// compute normals for orientation
		normalPoly_ = normal<VEC3>(map_, Cell<Orbit::PHI1>(f.dart), position);

		// first pass create polygon in chained list with angle computation
		VertexPoly* vpp = nullptr;
		VertexPoly* prem = nullptr;
		nb_verts_ = 0;
		convex_ = true;

		Dart a = f.dart;
		Dart b = map_.phi1(a);
		Dart c = map_.phi1(b);
		do
		{
			const VEC3& P1 = position[Vertex(a)];
			const VEC3& P2 = position[Vertex(b)];
			const VEC3& P3 = position[Vertex(c)];

			Scalar val = ear_angle(P1, P2, P3);
			VertexPoly* vp = new VertexPoly(Vertex(b), val, Scalar((P3-P1).squaredNorm()), vpp);

			if (vp->value_ > 5.0f)  // concav angle
				convex_ = false;

			if (vpp == nullptr)
				prem = vp;
			vpp = vp;
			a = b;
			b = c;
			c = map_.phi1(c);
			nb_verts_++;
		} while (a != f.dart);

		VertexPoly::close(prem, vpp);

		if (convex_)
		{
			// second pass with no test of intersections with polygons
			vpp = prem;
			for (uint32 i = 0; i < nb_verts_; ++i)
			{
				vpp->ear_ = ears_.insert(vpp);
				vpp = vpp->next_;
			}
		}
		else
		{
			// second pass test intersections with polygons
			vpp = prem;
			for (uint32 i = 0; i < nb_verts_; ++i)
			{
				if (vpp->value_ < 5.0f)
					ear_intersection(vpp);
				vpp->ear_ = ears_.insert(vpp);
				vpp = vpp->next_;
			}
		}
	}

	/**
	 * @brief compute table of vertices indices (embeddings) of triangulation
	 * @param table_indices
	 */
	void append_indices(std::vector<uint32>& table_indices)
	{
		table_indices.reserve((nb_verts_ - 2) * 3);

		if (nb_verts_ == 3)
		{
			map_.foreach_incident_vertex(face_, [&] (Vertex v)
			{
				table_indices.push_back(map_.embedding(v));
			});
			return;
		}

		while (nb_verts_ > 3)
		{
			// take best (and valid!) ear
			typename VPMS::iterator be_it = ears_.begin(); // best ear
			VertexPoly* be = *be_it;

			table_indices.push_back(map_.embedding(be->vert_));
			table_indices.push_back(map_.embedding(be->next_->vert_));
			table_indices.push_back(map_.embedding(be->prev_->vert_));
			--nb_verts_;

			if (nb_verts_ > 3)	// do not recompute if only one triangle left
			{
				// remove ears and two sided ears
				ears_.erase(be_it);				// from map of ears
				ears_.erase(be->next_->ear_);
				ears_.erase(be->prev_->ear_);
				be = VertexPoly::erase(be); 	// and remove ear vertex from polygon
				recompute_2_ears(be);
			}
			else // finish (no need to update ears)
			{
				// remove ear from polygon
				be = VertexPoly::erase(be);
				// last triangle
				table_indices.push_back(map_.embedding(be->vert_));
				table_indices.push_back(map_.embedding(be->next_->vert_));
				table_indices.push_back(map_.embedding(be->prev_->vert_));
				// release memory of last triangle in polygon
				delete be->next_;
				delete be->prev_;
				delete be;
			}
		}
	}

	/**
	 * @brief apply the ear triangulation the face
	 */
	void apply()
	{
		while (nb_verts_ > 3)
		{
			// take best (and valid!) ear
			typename VPMS::iterator be_it = ears_.begin(); // best ear
			VertexPoly* be = *be_it;

			map_.cut_face(be->prev_->vert_.dart, be->next_->vert_.dart);
			--nb_verts_;

			if (nb_verts_ > 3)	// do not recompute if only one triangle left
			{
				// remove ears and two sided ears
				ears_.erase(be_it);				// from map of ears
				ears_.erase(be->next_->ear_);
				ears_.erase(be->prev_->ear_);
				// replace dart to be in remaining poly
				be ->prev_->vert_ = Vertex(map_.phi2(map_.phi_1(be ->prev_->vert_.dart)));
				be = VertexPoly::erase(be); 	// and remove ear vertex from polygon
				recompute_2_ears(be);
			}
			else // finish (no need to update ears)
			{
				// release memory of last triangle in polygon
				delete be->next_;
				delete be->prev_;
				delete be;
			}
		}
	}
};

/**
 * @brief compute ear triangulation
 * @param map
 * @param f face
 * @param position
 * @param table_indices table of indices (vertex embedding) to append
 */
template <typename VEC3, typename MAP>
static void append_ear_triangulation(
	MAP& map,
	const typename MAP::Face f,
	const typename MAP::template VertexAttribute<VEC3>& position,
	std::vector<uint32>& table_indices
)
{
	EarTriangulation<VEC3, MAP> tri(map, f, position);
	tri.append_indices(table_indices);
}

/**
 * @brief apply ear triangulation to a face (face is cut)
 * @param map
 * @param f
 * @param position
 */
template <typename VEC3, typename MAP>
static void apply_ear_triangulation(
	MAP& map,
	const typename MAP::Face f,
	const typename MAP::template VertexAttribute<VEC3>& position
)
{
	EarTriangulation<VEC3, MAP> tri(map, f, position);
	tri.apply();
}

/**
 * @brief apply ear triangulation to a map
 * @param map
 * @param f
 * @param position
 */
template <typename VEC3, typename MAP>
static void apply_ear_triangulation(
	MAP& map,
	const typename MAP::template VertexAttribute<VEC3>& position
)
{
	map.template foreach_cell([&] (typename MAP::Face f)
	{
		if (!map.has_codegree(f, 3))
		{
			EarTriangulation<VEC3, MAP> tri(map, f, position);
			tri.apply();
		}
	});
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_EAR_TRIANGULATION_H_

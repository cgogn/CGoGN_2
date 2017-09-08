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

#include <cgogn/topology/types/adaptive_tri_quad_cmap2.h>

namespace cgogn
{

AdaptiveTriQuadCMap2::AdaptiveTriQuadCMap2(CMap2& map) : map_(map)
{
	dart_level_ = map_.add_attribute<uint8, CDart::ORBIT>("__dart_level");
	face_subd_id_ = map_.add_attribute<uint32, Face::ORBIT>("__face_subdivision_id");
	tri_face_ = map_.add_attribute<bool, Face::ORBIT>("__tri_face");
}

AdaptiveTriQuadCMap2::~AdaptiveTriQuadCMap2()
{
	map_.remove_attribute(tri_face_);
	map_.remove_attribute(face_subd_id_);
	map_.remove_attribute(dart_level_);
}

void AdaptiveTriQuadCMap2::init()
{
	map_.parallel_foreach_cell(
		[&] (Face f)
		{
			if (map_.codegree(f) == 3)
				tri_face_[f] = true;
			else
				tri_face_[f] = false;
			face_subd_id_[f] = 0;
			map_.foreach_dart_of_orbit(f, [&] (Dart d) { dart_level_[d] = 0; });
		}
	);
}

bool AdaptiveTriQuadCMap2::is_simplifiable(Face f)
{
	if (face_level(f) == 0)
		return false;

	switch (face_type(f))
	{
		case TRI_CORNER: {
			if (map_.codegree(f) != 3)
				return false;
			Dart it = oldest_dart(f);
			it = map_.phi<12>(it); // central face
			if (map_.codegree(Face(it)) != 3)
				return false;
			Dart it2 = map_.phi<12>(it); // corner face 1
			if (map_.codegree(Face(it2)) != 3)
				return false;
			it2 = map_.phi2(map_.phi_1(it)); // corner face 2
			if (map_.codegree(Face(it2)) != 3)
				return false;
			return true;
			break;
		}
		case TRI_CENTRAL: {
			if (map_.codegree(f) != 3)
				return false;
			Dart it = f.dart;
			if (map_.codegree(Face(map_.phi2(it))) != 3) // corner face 1
				return false;
			it = map_.phi1(it);
			if (map_.codegree(Face(map_.phi2(it))) != 3) // corner face 2
				return false;
			it = map_.phi1(it);
			if (map_.codegree(Face(map_.phi2(it))) != 3) // corner face 3
				return false;
			return true;
			break;
		}
		case QUAD: {
			Dart cv = map_.phi<12>(oldest_dart(f));
			bool res = true;
			map_.foreach_incident_face(Vertex(cv), [&] (Face iface) -> bool
			{
				if (map_.codegree(iface) != 4)
					res = false;
				return res; // break foreach loop when res is false
			});
			return res;
			break;
		}
	}
}

uint8 AdaptiveTriQuadCMap2::face_level(Face f)
{
	uint32 id = face_subd_id_[f];
	if (id == 0) return 0;
	if (id < 5) return 1;
	if (id < 21) return 2;
	if (id < 85) return 3;
	if (id < 341) return 4;
	if (id < 1365) return 5;
	if (id < 5461) return 6;
}

AdaptiveTriQuadCMap2::FaceType AdaptiveTriQuadCMap2::face_type(Face f)
{
	if (!tri_face_[f])
		return QUAD;
	else if (face_subd_id_[f] % 4 == 0)
		return TRI_CENTRAL;
	else
		return TRI_CORNER;
}

Dart AdaptiveTriQuadCMap2::oldest_dart(Face f)
{
	Dart res = f.dart;
	uint8 min = dart_level_[f.dart];
	map_.foreach_dart_of_orbit(f, [&] (Dart d)
	{
		if (dart_level_[d] < min)
			res = d;
	});
	return res;
}

} // namespace cgogn

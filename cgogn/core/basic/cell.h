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

#ifndef CORE_BASIC_CELL_H_
#define CORE_BASIC_CELL_H_

#include <core/basic/dart.h>


/**
 * \file core/basic/cell.h
 * \brief Orbit and cell definitions for CGOGN API
 */

namespace cgogn
{

const unsigned int NB_ORBITS	= 8;
const unsigned int VERTEX1		= 0;
const unsigned int VERTEX2		= 1;
const unsigned int EDGE2		= 2;
const unsigned int FACE2		= 3;
const unsigned int VERTEX3		= 4;
const unsigned int EDGE3		= 5;
const unsigned int FACE3		= 6;
const unsigned int VOLUME3		= 7;

inline std::string orbitName(unsigned int orbit)
{
	switch(orbit)
	{
		case VERTEX1: return "VERTEX1"; break;
		case VERTEX2: return "VERTEX2"; break;
		case EDGE2:   return "EDGE2"; break;
		case FACE2:   return "FACE2"; break;
		case VERTEX3: return "VERTEX3"; break;
		case EDGE3:   return "EDGE3"; break;
		case FACE3:   return "FACE3"; break;
		case VOLUME3: return "VOLUME3";
		default: break;
	}
	return "UNKNOWN";
}


/**
 * \brief Cellular typing
 *
 * \details warning to automatic conversion
 * cell -> Dart (or const Dart&) ok
 * Dart -> Cell (or const Cell&) ok
 * \tparam ORBIT The type of the orbit used to create the Cell
 */
template <unsigned int ORBIT>
class Cell
{
public:

	Dart dart;

	/**
	 * \brief Constructs a new  empty Cell with NIL dart.
	 */
	inline Cell(): dart() {}

	/**
	 * \brief Constructs a new Cell with a dart.
	 * \param d dart to convert to a cell of a given orbit
	 */
	inline Cell(Dart d): dart(d) {}

	/// copy constructor
	inline Cell(const Cell<ORBIT>& c): dart(c.dart) {}

	/// Dart cast operator
	inline operator Dart() const { return dart; }

	friend std::ostream& operator<<( std::ostream &out, const Cell<ORBIT>& fa ) { return out << fa.dart; }

	inline bool valid() const { return !dart.isNil(); }
};

} // namespace cgogn

#endif // CORE_BASIC_CELL_H_

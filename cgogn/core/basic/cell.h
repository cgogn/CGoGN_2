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
#include <utils/definitions.h>

/**
 * \file core/basic/cell.h
 * \brief Orbit and cell definitions for CGOGN API
 */

namespace cgogn
{

//const unsigned int Orbit::DART   = 0;
//const unsigned int Orbit::PHI21   = 1;
//const unsigned int Orbit::PHI2     = 2;
//const unsigned int Orbit::PHI1     = 3;
//const unsigned int Orbit::PHI21_PHI31   = 4;
//const unsigned int Orbit::PHI2_PHI3     = 5;
//const unsigned int Orbit::PHI1_PHI3     = 6;
//const unsigned int Orbit::PHI1_PHI2   = 7;
enum Orbit: unsigned int
{
	DART = 0,
	PHI21,
	PHI2,
	PHI1,
	PHI21_PHI31,
	PHI2_PHI3,
	PHI1_PHI3,
	PHI1_PHI2,
};

static const unsigned int NB_ORBITS = PHI1_PHI2+1;



inline std::string orbit_name(Orbit orbit)
{
	switch(orbit)
	{
		case Orbit::DART: return "Orbit::DART"; break;
		case Orbit::PHI21: return "Orbit::PHI21"; break;
		case Orbit::PHI2:   return "Orbit::PHI2"; break;
		case Orbit::PHI1:   return "Orbit::PHI1"; break;
		case Orbit::PHI21_PHI31: return "Orbit::PHI21_PHI31"; break;
		case Orbit::PHI2_PHI3:   return "Orbit::PHI2_PHI3"; break;
		case Orbit::PHI1_PHI3:   return "Orbit::PHI1_PHI3"; break;
		case Orbit::PHI1_PHI2: return "Orbit::PHI1_PHI2"; break;
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
template <Orbit ORBIT>
class Cell
{
public:

	Dart dart;

	/**
	 * \brief Constructs a new empty Cell with NIL dart.
	 */
	inline Cell() : dart()
	{}

	/**
	 * \brief Constructs a new Cell with a dart.
	 * \param d dart to convert to a cell of a given orbit
	 */
	inline Cell(Dart d) : dart(d)
	{}

	/// copy constructor
	inline Cell(const Cell<ORBIT>& c) : dart(c.dart)
	{}

	/// Dart cast operator
	inline operator Dart() const { return dart; }

	friend std::ostream& operator<<(std::ostream &out, const Cell<ORBIT>& fa) { return out << fa.dart; }

	inline bool is_valid() const { return !dart.is_nil(); }
};

} // namespace cgogn

#endif // CORE_BASIC_CELL_H_

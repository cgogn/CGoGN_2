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
 * \brief Orbit and cell definitions used in cgogn.
 */
namespace cgogn
{

const unsigned int NB_ORBITS = 8;

const unsigned int VERTEX1   = 0;
const unsigned int VERTEX2   = 1;
const unsigned int EDGE2     = 2;
const unsigned int FACE2     = 3;
const unsigned int VERTEX3   = 4;
const unsigned int EDGE3     = 5;
const unsigned int FACE3     = 6;
const unsigned int VOLUME3   = 7;

inline std::string orbit_name(unsigned int orbit)
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
		case VOLUME3: return "VOLUME3"; break;
		default: cgogn_assert_not_reached("orbit of this name do not exist"); break;
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

	/**
	 * \brief the dart representing this cell
	 */
	Dart dart;

	/**
	 * \brief Creates a new empty Cell as a nil dart.
	 */
	inline Cell() : dart()
	{}

	/**
	 * \brief Creates a new Cell with a dart. 
	 * \param[in] d dart to convert to a cell of a given orbit
	 */
	inline Cell(Dart d) : dart(d)
	{}

	/**
	 * \brief Copy constructor.
	 * Creates a new Cell from an another one.
	 * \param[in] c a cell
	 */
	inline Cell(const Cell<ORBIT>& c) : dart(c.dart)
	{}

	//TODO
	// Cell(Cell<ORBIT>&& ) = delete;

	/**
	 * \brief Cast operator.
	 * \return the dart 
	 */
	inline operator Dart() const { return dart; }

	/**
	 * \brief Tests the validity of the cell.
	 * \retval true if the cell is valid
	 * \retval false otherwise
	 */
	inline bool is_valid() const { return !dart.is_nil(); }

	/**
	 * \brief Assigns to the left hand side cell the value
	 * of the right hand side cell.
	 * \param[in] rhs the cell to assign
	 * \return The cell with the assigned value
	 */
	Cell<ORBIT> operator=(Cell<ORBIT> rhs) { dart = rhs.dart; return *this; }


	//TODO
	// Cell<ORBIT> operator=(Cell<ORBIT>&& rhs) { dart = rhs.dart return *this; }

	/**
	 * \brief Prints a cell to a stream.
	 * \param[out] out the stream to print on
	 * \param[in] rhs the cell to print
	 */
	friend std::ostream& operator<<(std::ostream &out, const Cell<ORBIT>& rhs) { return out << rhs.dart; }

	/**
	 * \brief Reads a cell from a stream.
	 * \param[in] in the stream to read from
	 * \param[out] rhs the cell read
	 */
	friend std::istream& operator>>(std::istream &in, Cell<ORBIT>& rhs) { in >> rhs.dart; return in; }
};

} // namespace cgogn

#endif // CORE_BASIC_CELL_H_

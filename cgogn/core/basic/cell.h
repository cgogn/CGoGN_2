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

#include <core/utils/assert.h>
#include <core/utils/definitions.h>

/**
 * \file core/basic/cell.h
 * \brief Orbit and cell definitions used in cgogn.
 */
namespace cgogn
{

enum Orbit: unsigned int
{
	DART = 0,
	PHI1,
	PHI2,
	PHI1_PHI2,
	PHI1_PHI3,
	PHI2_PHI3,
	PHI21,
	PHI21_PHI31,
};

static const std::size_t NB_ORBITS = Orbit::PHI21_PHI31 + 1;

static const unsigned int EMBNULL = 0xffffffff;

inline std::string orbit_name(Orbit orbit)
{
	switch(orbit)
	{
		case Orbit::DART: return "cgogn::Orbit::DART"; break;
		case Orbit::PHI1: return "cgogn::Orbit::PHI1"; break;
		case Orbit::PHI2: return "cgogn::Orbit::PHI2"; break;
		case Orbit::PHI1_PHI2: return "cgogn::Orbit::PHI1_PHI2"; break;
		case Orbit::PHI1_PHI3: return "cgogn::Orbit::PHI1_PHI3"; break;
		case Orbit::PHI2_PHI3: return "cgogn::Orbit::PHI2_PHI3"; break;
		case Orbit::PHI21: return "cgogn::Orbit::PHI21"; break;
		case Orbit::PHI21_PHI31: return "cgogn::Orbit::PHI21_PHI31"; break;
		default: cgogn_assert_not_reached("This orbit does not exist"); break;
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

	/**
	* \brief Name of this CGoGN type
	* \return a string representing the name of the class
	*/
	static std::string cgogn_name_of_type() { return std::string("cgogn::Cell<") + orbit_name(ORBIT) +std::string(">"); }
};

} // namespace cgogn

#endif // CORE_BASIC_CELL_H_

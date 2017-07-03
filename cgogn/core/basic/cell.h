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

#ifndef CGOGN_CORE_BASIC_CELL_H_
#define CGOGN_CORE_BASIC_CELL_H_

#include <cgogn/core/basic/dart.h>

#include <cgogn/core/utils/assert.h>
#include <cgogn/core/utils/numerics.h>

#include <climits>

/**
 * \file core/basic/cell.h
 * \brief Orbit and cell definitions used in cgogn.
 */

namespace cgogn
{

enum Orbit: uint32
{
	DART = 0,
	PHI1,
	PHI2,
	PHI21,
	PHI1_PHI2,
	PHI1_PHI3,
	PHI2_PHI3,
	PHI21_PHI31,
	PHI1_PHI2_PHI3
};

static const std::size_t NB_ORBITS = Orbit::PHI1_PHI2_PHI3 + 1;

inline std::string orbit_name(Orbit orbit)
{
	switch (orbit)
	{
		case Orbit::DART: return "cgogn::Orbit::DART"; break;
		case Orbit::PHI1: return "cgogn::Orbit::PHI1"; break;
		case Orbit::PHI2: return "cgogn::Orbit::PHI2"; break;
		case Orbit::PHI21: return "cgogn::Orbit::PHI21"; break;
		case Orbit::PHI1_PHI2: return "cgogn::Orbit::PHI1_PHI2"; break;
		case Orbit::PHI1_PHI3: return "cgogn::Orbit::PHI1_PHI3"; break;
		case Orbit::PHI2_PHI3: return "cgogn::Orbit::PHI2_PHI3"; break;
		case Orbit::PHI21_PHI31: return "cgogn::Orbit::PHI21_PHI31"; break;
		case Orbit::PHI1_PHI2_PHI3: return "cgogn::Orbit::PHI1_PHI2_PHI3"; break;
//		default: cgogn_assert_not_reached("This orbit does not exist"); return "UNKNOWN"; break;
	}
	cgogn_assert_not_reached("This orbit does not exist");
#ifdef NDEBUG 
	return "UNKNOWN"; // little trick to avoid warning on VS
#endif
}

static const uint32 ALL_CELLS_MASK = 0xffffffff;

template <typename CellType>
inline uint32 orbit_mask()
{
	return 1 << CellType::ORBIT;
}

inline uint32 orbit_mask(Orbit orbit)
{
	return 1 << orbit;
}

/**
 * \brief Cellular typing
 * \tparam ORBIT The type of the orbit used to create the Cell
 */
template <Orbit ORBIT_VAL>
class Cell
{
public:

	static const Orbit ORBIT = ORBIT_VAL;
	using Self = Cell<ORBIT>;

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
	inline explicit Cell(Dart d) : dart(d)
	{}

	/**
	 * \brief Copy constructor.
	 * Creates a new Cell from an another one.
	 * \param[in] c a cell
	 */
	inline Cell(const Self& c) : dart(c.dart)
	{}

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
	inline Self& operator=(Self rhs) { dart = rhs.dart; return *this; }

	/**
	 * \brief Prints a cell to a stream.
	 * \param[out] out the stream to print on
	 * \param[in] rhs the cell to print
	 */
	inline friend std::ostream& operator<<(std::ostream &out, const Self& rhs) { return out << rhs.dart; }

	/**
	 * \brief Reads a cell from a stream.
	 * \param[in] in the stream to read from
	 * \param[out] rhs the cell read
	 */
	inline friend std::istream& operator>>(std::istream &in, Self& rhs) { in >> rhs.dart; return in; }

	/**
	* \brief Name of this CGoGN type
	* \return a string representing the name of the class
	*/
	static std::string cgogn_name_of_type() { return std::string("cgogn::Cell<") + orbit_name(ORBIT) +std::string(">"); }

	inline void cgogn_binary_serialize(std::ostream& o, bool little_endian)
	{
		serialization::serialize_binary(o, dart.index, little_endian);
	}
};

} // namespace cgogn

#endif // CGOGN_CORE_BASIC_CELL_H_

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

#ifndef CORE_BASIC_DART_H_
#define CORE_BASIC_DART_H_

#include <climits>
#include <string>
#include <iostream>

/**
 * \file cgogn/core/basic/dart.h
 * \brief Dart definition.
 */
namespace cgogn
{

/**
 * \brief Dart.
 */
struct Dart
{
	// MSVC doesn't support  std::numeric_limits<unsigned int>::max() when declaring static const variables
	static const unsigned int INVALID_INDEX = UINT_MAX;

	/**
	 * \brief the value of a dart.
	 */
	unsigned int index;

	/**
	 * \brief Creates a new nil Dart
	 */
	inline Dart() : index(INVALID_INDEX)
	{}

	/**
	 * \brief Creates a new Dart with a value
	 * \details The explicit keyword specifies that this
	 * constructor is only considered for direct initialization.
	 * \code
	 * Dart d = 10 is forbidden
	 * \endcode
	 *
	 * \param[in] v the value of the new dart
	 */
	inline explicit Dart(unsigned int v) : index(v)
	{}

	/**
	 * \brief Copy constructor.
	 * Creates a new Dart from an another one.
	 * \param[in] d a dart
	 */
	inline Dart(const Dart& d) : index(d.index)
	{}

	/**
	 * \brief Tests the nullity of the dart.
	 * \retval true if the dart is nil
	 * \retval false otherwise
	 */
	inline bool is_nil() const { return index == INVALID_INDEX; }

	/**
	 * \brief Assigns to the left hand side dart the value
	 * of the right hand side dart.
	 * \param[in] rhs the dart to assign
	 * \return The dart with the assigned value
	 */
	inline Dart& operator=(Dart rhs) { index = rhs.index; return *this; }

	/**
	 * \brief Tests whether the left hand side dart is equal
	 * from the right hand side dart.
	 * \param[in] rhs the dart to compare with
	 * \retval true if \p lhs is equal than \p rhs
	 * \retval false otherwise
	 */
	inline bool operator==(Dart rhs) const { return index == rhs.index; }

	/**
	 * \brief Tests whether the left hand side dart is different
	 * from the right hand side dart.
	 * \param[in] rhs the dart to compare with
	 * \retval true if \p lhs is different than \p rhs
	 * \retval false otherwise
	 */
	inline bool operator!=(Dart rhs) const { return index != rhs.index; }

	// operator < is needed if we want to use std::set<Dart>
	inline bool operator<(Dart rhs) const { return index < rhs.index; }
	/**
	 * \brief Prints a dart to a stream.
	 * \param[out] out the stream to print on
	 * \param[in] rhs the dart to print
	 */
	inline friend std::ostream& operator<<(std::ostream &out, const Dart& rhs) { return out << rhs.index; }

	/**
	 * \brief Reads a dart from a stream.
	 * \param[in] in the stream to read from
	 * \param[out] rhs the dart read
	 */
	inline friend std::istream& operator>>(std::istream &in, Dart& rhs) { in >> rhs.index; return in; }
};

} // namespace cgogn

#endif // CORE_BASIC_DART_H_

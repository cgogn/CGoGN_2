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

#include <limits>

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
		/**
		 * \brief the value of a dart.
		 */
		unsigned int index;

		/**
		 * \brief Creates a new nil Dart 
		 */
		Dart(): index(std::numeric_limits<unsigned int>::max()) {}

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
		explicit Dart(unsigned int v): index(v) {}

		/**
		 * \brief Copy constructor.
		 * Creates a new Dart from an another one.
		 * \param[in] d a dart
		 */
		Dart(const Dart& d): index(d.index) {}

		//TODO why ???? puisque Dart d; -> d is nil
		//utilise avec l'operateur de comparaison a la place de isNil ...
		static Dart nil() { Dart d; d.index = std::numeric_limits<unsigned int>::max(); return d; 
		}

		//TODO why ??? Dar d = Dart::create(10) instead of Dard d(10)
		//utilise dans genericmap newDart() 
		static Dart create(unsigned int i) { Dart d; d.index = i; return d; }

		/**
		 * \brief Name of this CGoGN type
		 * \return a string representing the name of the class
		 * \todo un peu moche comme facon de faire...
		 */
		static std::string CGoGNnameOfType() { return "Dart"; }

		/**
		 * \brief Tests the nullity of the dart.
		 * \retval true if the dart is nil
		 * \retval false otherwise
		 */
		bool isNil() const { return index == std::numeric_limits<unsigned int>::max() ; }

		/**
		 * \brief Assigns to the left hand side dart the value 
		 * of the right hand side dart.
		 * \param[in] rhs the dart to assign
		 * \return The dart with the assigned value
		 */
		Dart operator=(Dart rhs) { index = rhs.index; return *this; }

		/**
		 * \brief Tests whether the left hand side dart is equal 
		 * from the right hand side dart.
		 * \param[in] rhs the dart to compare with
		 * \retval true if \p lhs is equal than \p rhs
		 * \retval false otherwise
		 */
		bool operator==(Dart rhs) const { return index == rhs.index; }

		/**
		 * \brief Tests whether the left hand side dart is different 
		 * from the right hand side dart.
		 * \param[in] rhs the dart to compare with
		 * \retval true if \p lhs is different than \p rhs
		 * \retval false otherwise
		 */
		bool operator!=(Dart rhs) const { return index != rhs.index; }

		/**
		 * \brief Tests whether the left hand side dart is less 
		 * greather than the right hand side dart.
		 * \param[in] rhs the dart to compare with
		 * \retval true if \p lhs is less greather than \p rhs
		 * \retval false otherwise
		 */
		bool operator<(Dart rhs) const { return index < rhs.index; }

		/**
		 * \brief Prints a dart to a stream.
		 * \param[out] out the stream to print on
		 * \param[in] rhs the dart to print
		 */
		friend std::ostream& operator<<( std::ostream &out, const Dart& rhs )  { return out << rhs.index; }
		
		/**
		 * \brief Reads a dart from a stream.
		 * \param[in] in the stream to read from
		 * \param[out] rhs the dart read
		 */
		friend std::istream& operator>>( std::istream &in, Dart& rhs ) { in >> rhs.index; return in; }

	};


	/**
	 * \brief Definition of a nil dart
	 * \todo usefull .. Le constructeur vide creer un dart nil
	 */
	const Dart NIL = Dart::nil();

	/**
	 * \brief Definition of null embedding
	 */
	const unsigned int EMBNULL = std::numeric_limits<unsigned int>::max();

} // namespace cgogn

#endif // CORE_BASIC_DART_H_

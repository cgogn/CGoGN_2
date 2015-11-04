/*
 * CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps
 * Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at your
 * option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.
 *
 * Web site: http://cgogn.unistra.fr/
 * Contact information: cgogn@unistra.fr
 *
 */

#ifndef CORE_BASIC_ASSERT_
#define CORE_BASIC_ASSERT_

#include <string>
#include <iostream>
#include <sstream>

/**
 * \file cgogn/core/basic/assert.h
 * \brief Assertion checking mechanism.
 *
 * Allows the user to add a specific message to output
 */
namespace cgogn 
{

	/**
	 * Prints an assertion failure.
	 * This function is called when a boolean condition is not met.
	 * It prints an error message and terminates the program.
	 * \param[in] expression string representation of the condition.
	 * \param[in] message string information message to print out.
	 * \param[in] file_name file where the assertion failed.
	 * \param[in] function_name function where the assertion failed.
	 * \param[in] line_number line where the assertion failed.
	 *
	 * \todo Add attribute [[noreturn]] when MSVC min version = 14
	 */
	void assertion_failed(const std::string& expression, const std::string& message,
		const std::string& file_name, const std::string& function_name, int line_number );

}

/**
 * \brief Verifies that a condition is met.
 * \details
 * \param[in] x the boolean expression of the condition
 * \see assertion_failed()
 */
#define cgogn_assert(x) \
	(!(x)) ? cgogn::assertion_failed(#x, "", __FILE__, __func__, __LINE__) : (void)0

/**
 * \brief Verifies that a condition is met and take a specific message.
 * \details
 * \param[in] x the boolean expression of the condition
 * \parap[in] msg the specific message about the condition
 * \see assertion_failed()
 */
#define cgogn_message_assert(x, msg) \
	(!(x)) ? cgogn::assertion_failed(#x, msg, __FILE__, __func__, __LINE__) : (void)0

/**
 * \brief Verifies that the required contract condition is met.
 * \details
 * 
 * \param[in] x the boolean expression of the condition
 * \see assertion_failed()
 */
#define cgogn_require(x) \
	(!(x)) ? cgogn::assertion_failed(#x, "", __FILE__, __func__, __LINE__) : (void)0

/**
 * \brief Verifies that the ensured contract condition is met.
 * \details
 * 
 * \param[in] x the boolean expression of the condition
 * \see assertion_failed()
 */
#define cgogn_ensure(x) \
	(!(x)) ? cgogn::assertion_failed(#x, "", __FILE__, __func__, __LINE__) : (void)0

/**
 * \brief Verifies that the invariant contract condition is met.
 * \details
 * 
 * \param[in] x the boolean expression of the condition
 * \see assertion_failed()
 */
#define cgogn_invariant(x) \	
	(!(x)) ? cgogn::assertion_failed(#x, "", __FILE__, __func__, __LINE__) : (void)0


/**
 * \def debug_assert(x)
 * \copydoc cgogn_assert()
 * \note This assertion check is only active in debug mode.
 */
 /**
 * \def debug_message_assert(x, msg)
 * \copydoc cgogn_assert()
 * \note This assertion check is only active in debug mode.
 */
#ifdef CGOGN_DEBUG
 	#define debug_assert(x) cgogn_assert(x)
	#define debug_message_assert(x, msg) cgogn_message_assert(x, msg)
 	#define debug_require(x) cgogn_require(x)
 	#define debug_ensure(x) cgogn_ensure(x)
 	#define debug_invariant(x) cgogn_invariant(x)
#else
  	#define debug_assert(x)
	#define debug_message_assert(x, msg)
  	#define debug_require(x)
 	#define debug_ensure(x)
 	#define debug_invariant(x)
#endif

/**
 * \def parano_assert(x)
 * \copydoc cgogn_assert()
 * \note This assertion check is only active in parano mode.
 */
 /**
 * \def parano_message_assert(x, msg)
 * \copydoc cgogn_assert()
 * \note This assertion check is only active in parano mode.
 */
#ifdef CGOGN_PARANO
 	#define parano_assert(x) cgogn_assert(x)
 	#define parano_message_assert(x, msg) cgogn_message_assert(x, msg)
#else
 	#define parano_assert(x)
 	#define parano_message_assert(x, msg)
#endif

#endif
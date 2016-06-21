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

#define CGOGN_CORE_DLL_EXPORT
#define CGOGN_CORE_UTILS_LOGGER_OUTPUT_CPP_

#include <iostream>
#include <limits>

#include <cgogn/core/utils/logger_output.h>

#include <termcolor.hpp>

namespace cgogn
{

namespace logger
{

void NullOutput::process_entry(const LogEntry&)
{}

ConsoleOutput::ConsoleOutput() : LoggerOutput()
{}

void ConsoleOutput::process_entry(const LogEntry& e)
{
	if (e)
	{
		std::ostream& o = (e.get_level() <= LogLevel::LogLevel_DEPRECATED) ? std::cout : std::cerr;
		internal::add_color(o,e.get_level()) << termcolor::bold;
		o << "[" << internal::loglevel_to_string(e.get_level()) << "]" << termcolor::reset;
		if (!e.get_sender().empty())
		{
			o << '(' << termcolor::magenta << e.get_sender() << termcolor::reset << ')';
		}
		o << ": " << e.get_message_str();
		if (e.get_level() >= LogLevel::LogLevel_DEBUG && !e.get_fileinfo().empty())
		{
			o << " (file " << termcolor::magenta;
			o << e.get_fileinfo() << termcolor::reset << ')';
		}
		o << std::endl;
	}
}

FileOutput::FileOutput(const std::string& filename) : LoggerOutput()
  ,filename_(filename)
{
	out_.open(filename, std::ios_base::out | std::ios_base::trunc);
}

void FileOutput::process_entry(const LogEntry& e)
{
	if (out_.good())
	{
		out_ << "[" << internal::loglevel_to_string(e.get_level()) << "]";
		if (!e.get_sender().empty())
		{
			out_ << '(' << e.get_sender() << ')';
		}
		out_ << ": " << e.get_message_str();
		if (!e.get_fileinfo().empty())
			out_ << " (file " << e.get_fileinfo() << ')';
		out_ << std::endl;
	}
}

} // namespace logger
} // namespace cgogn

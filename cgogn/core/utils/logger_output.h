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

#ifndef CORE_UTILS_LOGGER_OUTPUT_H_
#define CORE_UTILS_LOGGER_OUTPUT_H_

#include <mutex>
#include <core/utils/log_entry.h>

namespace cgogn
{

namespace logger
{

class CGOGN_CORE_API LoggerOutput
{
public:
		using Self = LoggerOutput;

		inline LoggerOutput() {}
		CGOGN_NOT_COPYABLE_NOR_MOVABLE(LoggerOutput);
		virtual ~LoggerOutput() {}

		virtual void process_entry(const LogEntry& entry) = 0;
};

class CGOGN_CORE_API NullOutput final : public  LoggerOutput
{
public:
		using Self = NullOutput;

		inline NullOutput() {}
		CGOGN_NOT_COPYABLE_NOR_MOVABLE(NullOutput);

		virtual void process_entry(const LogEntry&) override;
};


class CGOGN_CORE_API ConsoleOutput final : public  LoggerOutput
{
public:
		using Self = ConsoleOutput;
		using LogLevel = internal::LogLevel;

		ConsoleOutput();
		CGOGN_NOT_COPYABLE_NOR_MOVABLE(ConsoleOutput);

		virtual void process_entry(const LogEntry&) override;
};

} // namespace logger
} // namespace cgogn

#endif // CORE_UTILS_LOGGER_OUTPUT_H_

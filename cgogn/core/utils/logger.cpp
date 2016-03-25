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
#define CORE_UTILS_LOGGER_CPP_

#include <iostream>
#include <core/utils/logger.h>

namespace cgogn
{

namespace logger
{

Logger& Logger::get_logger()
{
	static Logger logger_instance;
	return logger_instance;
}

void Logger::process(const LogEntry& entry) const
{
	std::lock_guard<std::mutex> guard(process_mutex_);
	for (auto& o : outputs_)
		o->process_entry(entry);
}

LogStream Logger::info(const std::string& sender, Logger::FileInfo fileinfo) const
{
	return log(LogLevel::LogLevel_INFO, sender, fileinfo);
}

LogStream Logger::debug(const std::string& sender, Logger::FileInfo fileinfo) const
{
	return log(LogLevel::LogLevel_DEBUG, sender, fileinfo);
}

LogStream Logger::deprecated(const std::string& sender, Logger::FileInfo fileinfo) const
{
	return log(LogLevel::LogLevel_DEPRECATED, sender, fileinfo);
}

LogStream Logger::warning(const std::string& sender, Logger::FileInfo fileinfo) const
{
	return log(LogLevel::LogLevel_WARNING, sender, fileinfo);
}

LogStream Logger::error(const std::string& sender, Logger::FileInfo fileinfo) const
{
	return log(LogLevel::LogLevel_ERROR, sender, fileinfo);
}

Logger::Logger()
{
	outputs_.push_back(make_unique<ConsoleOutput>());
}

LogStream Logger::log(LogLevel lvl, const std::string& sender, Logger::FileInfo fileinfo) const
{
	return LogStream(lvl, sender, fileinfo);
}



} // namespace logger
} // namespace cgogn


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

#ifndef CORE_UTILS_LOGGER_H_
#define CORE_UTILS_LOGGER_H_

#include <ostream>
#include <memory>
#include <vector>

#include <core/utils/log_entry.h>
#include <core/utils/logger_output.h>
#include <core/utils/log_stream.h>
#include <core/utils/unique_ptr.h>

#define CGOGN_FILE_INFO ::cgogn::logger::internal::FileInfo(__FILE__, __LINE__)
#define cgogn_log_info(emitter) ::cgogn::logger::Logger::get_logger().info(emitter,CGOGN_FILE_INFO)
#define cgogn_log_debug(emitter) ::cgogn::logger::Logger::get_logger().debug(emitter,CGOGN_FILE_INFO)
#define cgogn_log_deprecated(emitter) ::cgogn::logger::Logger::get_logger().deprecated(emitter,CGOGN_FILE_INFO)
#define cgogn_log_warning(emitter) ::cgogn::logger::Logger::get_logger().warning(emitter,CGOGN_FILE_INFO)
#define cgogn_log_error(emitter) ::cgogn::logger::Logger::get_logger().error(emitter,CGOGN_FILE_INFO)

namespace cgogn
{

namespace logger
{

class CGOGN_CORE_API Logger final
{
public:
	using Self = Logger;
	using FileInfo = internal::FileInfo;
	using LogLevel = internal::LogLevel;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Logger);

	static Logger& get_logger();
	void process(const LogEntry& entry) const;

	LogStream info(const std::string& sender = "", FileInfo fileinfo = FileInfo()) const;
	LogStream debug(const std::string& sender = "", FileInfo fileinfo = FileInfo()) const;
	LogStream deprecated(const std::string& sender = "", FileInfo fileinfo = FileInfo()) const;
	LogStream warning(const std::string& sender = "", FileInfo fileinfo = FileInfo()) const;
	LogStream error(const std::string& sender = "", FileInfo fileinfo = FileInfo()) const;
private:
	Logger();
	LogStream log(LogLevel lvl, const std::string& sender, FileInfo fileinfo) const;

	std::vector<std::unique_ptr<LoggerOutput>>	outputs_;
};


} // namespace logger
} // namespace cgogn
#endif // CORE_UTILS_LOGGER_H_

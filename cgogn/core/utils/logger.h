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

#ifndef CGOGN_CORE_UTILS_LOGGER_H_
#define CGOGN_CORE_UTILS_LOGGER_H_

#include <ostream>
#include <memory>
#include <vector>

#include <cgogn/core/utils/log_entry.h>
#include <cgogn/core/utils/logger_output.h>
#include <cgogn/core/utils/log_stream.h>
#include <cgogn/core/utils/unique_ptr.h>

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

	static const Logger& get_logger();
	void process(const LogEntry& entry) const;

	LogStream info(const std::string& sender, FileInfo fileinfo) const;
	LogStream debug(const std::string& sender, FileInfo fileinfo) const;
	LogStream deprecated(const std::string& sender, FileInfo fileinfo) const;
	LogStream warning(const std::string& sender, FileInfo fileinfo) const;
	LogStream error(const std::string& sender, FileInfo fileinfo) const;

	void add_console_output();
	void remove_console_output();
	void add_file_output(const std::string& filename);
	void remove_file_output(const std::string& filename);
private:
	Logger();
	LogStream log(LogLevel lvl, const std::string& sender, FileInfo fileinfo) const;

	std::unique_ptr<ConsoleOutput>				console_out_;
	std::vector<std::unique_ptr<FileOutput>>	file_out_;
	mutable std::mutex							process_mutex_;
};


} // namespace logger
} // namespace cgogn
#endif // CGOGN_CORE_UTILS_LOGGER_H_

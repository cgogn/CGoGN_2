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

#define CGOGN_CORE_UTILS_LOGGER_CPP_

#include <iostream>
#include <cgogn/core/utils/logger.h>

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
	if (console_out_)
		console_out_->process_entry(entry);
	for (auto& o : file_out_)
		o->process_entry(entry);
	for (auto& o : other_outputs_)
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

void Logger::add_console_output()
{
	bool already_added = true;
	std::lock_guard<std::mutex> guard(process_mutex_);
	if (!console_out_)
	{
		already_added = false;
		console_out_ = make_unique<ConsoleOutput>();
	}
	if (already_added)
		std::cerr << "Logger::add_console_output: The console output is already activated." << std::endl;
}

void Logger::remove_console_output()
{
	std::lock_guard<std::mutex> guard(process_mutex_);
	console_out_.reset();
}

void Logger::add_file_output(const std::string& filename)
{
	bool already_added = false;
	std::lock_guard<std::mutex> guard(process_mutex_);
	for (auto& fileout : file_out_)
		if (fileout->get_filename() == filename)
			already_added = true;
	if (!already_added)
		file_out_.push_back(cgogn::make_unique<FileOutput>(filename));
	else
		std::cerr << "Logger::add_file_output: The file \"" << filename <<  "\" is already used by the logger." << std::endl;
}

void Logger::remove_file_output(const std::string& filename)
{
	bool found = false;
	std::lock_guard<std::mutex> guard(process_mutex_);
	for (auto it = file_out_.begin(), end = file_out_.end(); it != end ; ++it)
	{
		if ((*it)->get_filename() == filename)
		{
			found = true;
			file_out_.erase(it);
			break;
		}
	}
	if (!found)
		std::cerr << "Logger::remove_file_output: The file \"" << filename <<  "\" was not used by the logger." << std::endl;
}

void Logger::add_output(LoggerOutput* output)
{
	bool already_added = false;
	std::lock_guard<std::mutex> guard(process_mutex_);
	for (auto& out : other_outputs_)
		if (out == output)
			already_added = true;
	if (!already_added)
		other_outputs_.push_back(output);
	else
		std::cerr << "Logger::add_output : the specified output is already added. Ignoring." << std::endl;
}

Logger::Logger()
{
	add_console_output();
	add_file_output("cgogn.log");
}

LogStream Logger::log(LogLevel lvl, const std::string& sender, Logger::FileInfo fileinfo) const
{
	return LogStream(lvl, sender, fileinfo);
}



} // namespace logger
} // namespace cgogn


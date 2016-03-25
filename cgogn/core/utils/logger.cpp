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

namespace internal
{

FileInfo::FileInfo(const char* f, uint32 l):
 filename_(f)
,line_(l) {}

FileInfo::FileInfo():
filename_("unspecified file")
,line_(std::numeric_limits<uint32>::max())
{}

FileInfo::FileInfo(const FileInfo& other):
filename_(other.filename_)
,line_(other.line_)
{}

FileInfo::FileInfo(FileInfo&& other):
filename_(std::move(other.filename_))
,line_(other.line_)
{}

FileInfo& FileInfo::operator=(const FileInfo& other)
{
	if (this != &other)
	{
		filename_ = other.filename_;
		line_ = other.line_;
	}
	return *this;
}

FileInfo& FileInfo::operator=(FileInfo&& other)
{
	if (this != &other)
	{
		filename_ = std::move(other.filename_);
		line_ = other.line_;
	}
	return *this;
}

std::ostream& operator<<(std::ostream& o, const FileInfo& fileinfo)
{
	o << fileinfo.filename_;
	if (fileinfo.line_ != std::numeric_limits<uint32>::max())
		o << ":" << fileinfo.line_;
	return o;
}

} // namespace internal

LogStream::LogStream(LogLevel level, const std::string& sender, const LogStream::FileInfo& fileinfo) :
	log_entry_(level, sender, fileinfo)
{}

LogStream::LogStream(LogStream&& ls)
	: log_entry_( std::move(ls.log_entry_))
{}

LogStream::~LogStream()
{
	Logger::get_logger().process(log_entry_);
}

Logger& Logger::get_logger()
{
	static Logger logger_instance;
	return logger_instance;
}

void Logger::process(LogEntry& entry)
{
	for (auto& o : outputs)
		o->process_entry(entry);
}

LogStream Logger::info(const std::string& sender, Logger::FileInfo fileinfo)
{
	return log(LogLevel::LogLevel_INFO, sender, fileinfo);
}

Logger::Logger()
{
	outputs.push_back(make_unique<NullOutput>());
}

LogStream Logger::log(LogLevel lvl, const std::string& sender, Logger::FileInfo fileinfo)
{
	return LogStream(lvl, sender, fileinfo);
}

void NullOutput::process_entry(LogEntry&)
{}

LogEntry::LogEntry(const LogEntry& other) :
	sender_(other.sender_)
  ,fileinfo_(other.fileinfo_)
  ,message_(other.message_.str())
  ,level_(other.level_)
{}

LogEntry::LogEntry(LogEntry&& other) :
	sender_(std::move(other.sender_))
  ,fileinfo_(std::move(other.fileinfo_))
  ,message_(std::move(other.message_))
  ,level_(other.level_)
{}

LogEntry& LogEntry::operator=(const LogEntry& other)
{
	if (this != &other)
	{
		sender_ = other.sender_;
		fileinfo_ = other.fileinfo_;
		message_ = std::stringstream(other.message_.str());
		level_ = other.level_;
	}
	return *this;
}

LogEntry::LogEntry(LogLevel level, const std::string& sender, const LogEntry::FileInfo& fileinfo) :
	level_(level)
  ,sender_(sender)
  ,fileinfo_(fileinfo)
{}

LogEntry& LogEntry::operator=(LogEntry&& other)
{
	if (this != &other)
	{
		sender_ = std::move(other.sender_);
		fileinfo_ = std::move(other.fileinfo_);
		message_ = std::move(other.message_);
		level_ = other.level_;
	}
	return *this;
}

} // namespace logger
} // namespace cgogn


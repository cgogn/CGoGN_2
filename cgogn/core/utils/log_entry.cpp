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
#define CGOGN_CORE_UTILS_LOG_ENTRY_CPP_

#include <iostream>
#include <limits>

#include <cgogn/core/utils/log_entry.h>
#include <cgogn/core/utils/unique_ptr.h>

#include <termcolor.hpp>

namespace cgogn
{

namespace logger
{

namespace internal
{

CGOGN_CORE_API std::string loglevel_to_string(LogLevel lvl)
{
	switch (lvl)
	{
		case LogLevel::LogLevel_INFO: return "INFO";
		case LogLevel::LogLevel_DEBUG: return "DEBUG";
		case LogLevel::LogLevel_DEPRECATED: return "DEPRECATED";
		case LogLevel::LogLevel_WARNING: return "WARNING";
		case LogLevel::LogLevel_ERROR: return "ERROR";
		default: return "UNKNOWN_LOG_LEVEL";
	}
}

std::ostream& add_color(std::ostream& o, LogLevel lvl)
{
	o << termcolor::on_grey;
	switch (lvl)
	{
		case LogLevel::LogLevel_INFO: o << termcolor::green; break;
		case LogLevel::LogLevel_DEBUG: o << termcolor::blue; break;
		case LogLevel::LogLevel_DEPRECATED: o << termcolor::cyan; break;
		case LogLevel::LogLevel_WARNING: o << termcolor::yellow; break;
		case LogLevel::LogLevel_ERROR: o << termcolor::red; break;
		default: o << termcolor::reset;
	}
	return o;
}

FileInfo::FileInfo(const char* f, uint32 l) :
	filename_(f),
	line_(l)
{}

FileInfo::FileInfo() :
	filename_("unspecified file"),
	line_(std::numeric_limits<uint32>::max())
{}

FileInfo::FileInfo(const FileInfo& other) :
	filename_(other.filename_),
	line_(other.line_)
{}

FileInfo::FileInfo(FileInfo&& other) :
	filename_(std::move(other.filename_)),
	line_(other.line_)
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

bool FileInfo::empty() const
{
	return filename_.empty();
}

std::ostream& operator<<(std::ostream& o, const FileInfo& fileinfo)
{
	o << fileinfo.filename_;
	if (fileinfo.line_ != std::numeric_limits<uint32>::max())
		o << ":" << fileinfo.line_;
	return o;
}

} // namespace internal


LogEntry::LogEntry()
{
	message_ = make_unique<std::stringstream>();
}

LogEntry::LogEntry(LogEntry&& other) :
	sender_(std::move(other.sender_)),
	fileinfo_(std::move(other.fileinfo_)),
	message_(std::move(other.message_)),
	level_(other.level_)
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

LogEntry::LogEntry(LogLevel level, const std::string& sender, const LogEntry::FileInfo& fileinfo) :
	level_(level),
	sender_(sender),
	fileinfo_(fileinfo)
{
	message_ = make_unique<std::stringstream>();
}

} // namespace logger

} // namespace cgogn

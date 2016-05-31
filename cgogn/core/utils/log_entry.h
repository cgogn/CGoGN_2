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

#ifndef CGOGN_CORE_UTILS_LOG_ENTRY_H_
#define CGOGN_CORE_UTILS_LOG_ENTRY_H_

#include <string>
#include <sstream>
#include <memory>

#include <cgogn/core/dll.h>
#include <cgogn/core/utils/numerics.h>

namespace cgogn
{

namespace logger
{

namespace internal
{

class CGOGN_CORE_API FileInfo
{
public:
	FileInfo(const char *f, uint32 l);
	FileInfo();
	FileInfo(const FileInfo& other);
	FileInfo(FileInfo&& other);
	FileInfo& operator=(const FileInfo& other);
	FileInfo& operator=(FileInfo&& other);
	bool empty() const;

	friend std::ostream& operator<<(std::ostream& o, const FileInfo& fileinfo);
private:
	std::string filename_;
	uint32 line_;
};

enum LogLevel
{
	LogLevel_INFO = 0,
	LogLevel_DEBUG = 1,
	LogLevel_DEPRECATED = 2,
	LogLevel_WARNING = 3,
	LogLevel_ERROR = 4
};

CGOGN_CORE_API std::string loglevel_to_string(LogLevel lvl);
CGOGN_CORE_API std::ostream& add_color(std::ostream& o, LogLevel lvl);

} // namespace internal


class CGOGN_CORE_API LogEntry final
{
public:
	using Self = LogEntry;
	using FileInfo = internal::FileInfo;
	using LogLevel = internal::LogLevel;

	LogEntry();
	explicit LogEntry(LogLevel level, const std::string&sender, const FileInfo& fileinfo);
	LogEntry(LogEntry&& other);
	LogEntry& operator=(LogEntry&& other);
	LogEntry(const LogEntry&) = delete;
	LogEntry& operator=(const LogEntry&) = delete;

	inline const std::string& get_sender() const
	{
		return sender_;
	}

	inline const FileInfo& get_fileinfo() const
	{
		return fileinfo_;
	}

	inline const std::string get_message_str() const
	{
		return message_->str();
	}

	inline LogLevel get_level() const
	{
		return level_;
	}

	template <class T>
	LogEntry& operator<<(const T &x)
	{
		(*message_) << x;
		return *this;
	}

	// false iff the message is empty
	inline operator bool() const
	{
		auto* buff = message_->rdbuf();
		const auto curr_pos = buff->pubseekoff(0, std::ios_base::cur);
		const auto end = buff->pubseekoff(0, std::ios_base::end);
		buff->pubseekpos(curr_pos, std::ios_base::out);
		return end > 0;
	}

private:
	std::string							sender_;
	internal::FileInfo					fileinfo_;
	std::unique_ptr<std::stringstream>	message_;
	LogLevel							level_;
};

} // namespace logger
} // namespace cgogn

#endif // CGOGN_CORE_UTILS_LOG_ENTRY_H_

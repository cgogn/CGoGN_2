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

#include <string>
#include <ostream>
#include <limits>
#include <sstream>
#include <memory>
#include <vector>

#include <core/dll.h>
#include <core/utils/definitions.h>
#include <core/utils/unique_ptr.h>

#define CGOGN_FILE_INFO ::cgogn::logger::internal::FileInfo(__FILE__, __LINE__)

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

	friend std::ostream& operator<<(std::ostream& o, const FileInfo& fileinfo);
private:
	std::string filename_;
	uint32 line_;
};

} // namespace internal


enum LogLevel
{
	LogLevel_INFO = 0,
	LogLevel_DEBUG = 1,
	LogLevel_DEPRECATED = 2,
	LogLevel_WARNING = 3,
	LogLevel_ERROR = 4
};

class CGOGN_CORE_API LogEntry final
{
public:
	using Self = LogEntry;
	using FileInfo = internal::FileInfo;

	inline LogEntry() {}

	LogEntry(const LogEntry& other);
	LogEntry(LogEntry&& other);
	Self& operator=(const Self& other);
	inline Self& operator=(Self&& other);
	explicit LogEntry(LogLevel level, const std::string& sender = "", const FileInfo& fileinfo = FileInfo());

	inline const std::string& get_sender() const
	{
		return sender_;
	}

	inline const internal::FileInfo& get_fileinfo() const
	{
		return fileinfo_;
	}

	inline std::stringstream& get_message()
	{
		return message_;
	}

	inline LogLevel get_level() const
	{
		return level_;
	}

	template<class T>
	LogEntry& operator<<(const T &x)
	{
		get_message() << x;
		return *this;
	}

	// false iff the message is empty
	inline operator bool() const
	{
		auto* buff = const_cast<Self&>(*this).get_message().rdbuf();
		const auto curr_pos = buff->pubseekoff(0, std::ios_base::cur);
		const auto end = buff->pubseekoff(0, std::ios_base::end);
		buff->pubseekpos(curr_pos, const_cast<Self&>(*this).get_message().out);
		return end > 0;
	}

	private:
	std::string			sender_;
	internal::FileInfo	fileinfo_;
	std::stringstream	message_;
	LogLevel			level_;
};

class CGOGN_CORE_API LoggerOutput
{
public:
	using Self = LoggerOutput;
	inline LoggerOutput() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(LoggerOutput);
	virtual ~LoggerOutput() {}

	virtual void process_entry(LogEntry& entry) = 0;
};

class CGOGN_CORE_API NullOutput : public LoggerOutput
{
public:
	using Self = NullOutput;

	inline NullOutput() {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(NullOutput);

	virtual void process_entry(LogEntry&) override;
};

class CGOGN_CORE_API LogStream final
{
public:
	using FileInfo = internal::FileInfo;
	explicit LogStream(LogLevel level, const std::string& sender = "", const FileInfo& fileinfo = FileInfo());

	LogStream(LogStream&& ls);

	template<class T>
	LogStream& operator<<(const T &x)
	{
		log_entry_ << x;
		return *this;
	}

	~LogStream();

private:
	LogEntry log_entry_;
};

class CGOGN_CORE_API Logger final
{
public:
	using Self = Logger;
	using FileInfo = internal::FileInfo;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(Logger);

	static Logger& get_logger();
	void process(LogEntry& entry);


	LogStream info(const std::string& sender = "", FileInfo fileinfo = FileInfo());
private:
	Logger();
	LogStream log(LogLevel lvl, const std::string& sender, FileInfo fileinfo);

	std::vector<std::unique_ptr<LoggerOutput>>	outputs;
};


} // namespace logger
} // namespace cgogn
#endif // CORE_UTILS_LOGGER_H_

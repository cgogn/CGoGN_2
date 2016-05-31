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

#ifndef CGOGN_CORE_UTILS_LOG_STREAM_H_
#define CGOGN_CORE_UTILS_LOG_STREAM_H_

#include <string>
#include <sstream>

#include <cgogn/core/utils/log_entry.h>

namespace cgogn
{

namespace logger
{

class CGOGN_CORE_API LogStream final
{
public:
	using FileInfo = internal::FileInfo;
	using LogLevel = internal::LogLevel;

	explicit LogStream(LogLevel level, const std::string& sender, const FileInfo& fileinfo);
	LogStream(LogStream&& ls);

	template <class T>
	LogStream& operator<<(const T &x)
	{
		log_entry_ << x;
		return *this;
	}

	~LogStream();

private:
	LogEntry log_entry_;
};

} // namespace logger
} // namespace cgogn

#endif // CGOGN_CORE_UTILS_LOG_STREAM_H_

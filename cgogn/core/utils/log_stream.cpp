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

#define CGOGN_CORE_UTILS_LOG_STREAM_CPP_

#include <iostream>
#include <limits>

#include <cgogn/core/utils/log_stream.h>
#include <cgogn/core/utils/logger.h>
namespace cgogn
{

namespace logger
{

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


} // namespace logger
} // namespace cgogn

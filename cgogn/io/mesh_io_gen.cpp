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

#define CGOGN_IO_DLL_EXPORT

#include <istream>
#include <iostream>

#include <cgogn/core/utils/logger.h>
#include <cgogn/io/mesh_io_gen.h>

namespace cgogn
{

namespace io
{

MeshImportGen::~MeshImportGen() {}

bool MeshImportGen::import_file(const std::string& filename)
{
	this->clear();
	Scoped_C_Locale loc;

	if (!filename.empty())
	{
		// test if file exist
		std::ifstream fp(filename.c_str(), std::ios::in);
		if (!fp.good())
		{
			cgogn_log_warning("MeshImportGen::import_file") << "Unable to open file \"" << filename << "\"";
			return false;
		}
	}

	return this->import_file_impl(filename);
}

} // namespace io

} // namespace cgogn

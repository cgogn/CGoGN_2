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

#ifndef UTILS_IMPORT_SURFACE_H_
#define UTILS_IMPORT_SURFACE_H_

namespace cgogn
{

enum SurfaceFileType
{
	SurfaceFileType_AUTO = 0,
	SurfaceFileType_OFF,
	SurfaceFileType_OBJ
};

template <typename MAP>
class SurfaceImport
{
protected:

	unsigned int nb_vertices_;
	unsigned int nb_edges_;
	unsigned int nb_faces_;

	std::vector<unsigned short> faces_nb_edges_;
	std::vector<unsigned int> faces_vertex_indices_;

	MAP& map_;

public:

	SurfaceImport(MAP& m) : map_(m)
	{}

	void import_file(const std::string& filename);
};

} // namespace cgogn

#endif // UTILS_IMPORT_SURFACE_H_

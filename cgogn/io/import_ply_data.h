/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* version 0.1                                                                  *
* Copyright (C) 2009-2012, IGG Team, LSIIT, University of Strasbourg           *
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

#ifndef _IMPORT_PLY_DATA_H
#define _IMPORT_PLY_DATA_H

#include <cstdio>
#include <cmath>
#include <string>

#include <ply.h>
#include <cgogn/core/utils/numerics.h>
#include <cgogn/io/dll.h>

namespace cgogn
{

namespace io
{

class CGOGN_IO_API PlyImportData
{
public:

	using Self = PlyImportData;

	template <typename VEC>
	void vertex_position(int32 i, VEC& P) { P[0] = vlist[i]->x; P[1] = vlist[i]->y; P[2] = vlist[i]->z; }

	template <typename VEC>
	void vertex_normal(int32 i, VEC& N) { N[0] = vlist[i]->nx; N[1] = vlist[i]->ny; N[2] = vlist[i]->nz; }

	template <typename VEC>
	void vertexColorUint8(int32 i, VEC& C) { C[0] = vlist[i]->red; C[1] = vlist[i]->green; C[2] = vlist[i]->blue; }

	template <typename VEC>
	void vertex_color_float32(int32 i, VEC& C) { C[0] = vlist[i]->r; C[1] = vlist[i]->g; C[2] = vlist[i]->b; }


	inline int32 nb_vertices() { return nverts; }

	inline int32 nb_faces() { return nfaces; }

	/**
	* each vertex has a normal vector
	*/
	inline bool has_normals() { return has_normals_!=0;}

	/**
	* each vertex has a color vector
	*/
	bool has_colors() { return has_colors_uint8() || has_colors_float32(); }
	bool has_colors_uint8() { return per_vertex_color_uint8 != 0; }
	bool has_colors_float32() { return per_vertex_color_float32 != 0; }

	/**
	* get the number of edges of a face
	*/
	int32 face_valence(int32 i) { return flist[i]->nverts; }

	/**
	* get a table (pointer) of int32 of vertex indices of
	*/
	int32* face_indices(int32 i) { return flist[i]->verts; }

	PlyImportData();
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(PlyImportData);
	~PlyImportData();

	bool read_file(const std::string& filename);

protected:

	/* vertex and face definitions for a polygonal object */

	typedef struct VertexPly
	{
		float32 x,y,z;
		float32 r,g,b;
		unsigned char red,green,blue;
		float32 nx,ny,nz;
		void *other_props;       /* other properties */
	} VertexPly;

	typedef struct FacePly
	{
		unsigned char nverts;    /* number of vertex indices in list */
		int32 *verts;              /* vertex index list */
		void *other_props;       /* other properties */
	} FacePly;

	static char *elem_names[];

	static PlyProperty vert_props[];

	static PlyProperty face_props[];

	/*** the PLY object ***/

	int32 nverts,nfaces;
	VertexPly** vlist;
	FacePly** flist;

	PlyOtherProp *vert_other,*face_other;

	int32 per_vertex_color_float32, per_vertex_color_uint8 ;
	int32 has_normals_;
};

} // namespace io

} // namespace cgogn

#endif

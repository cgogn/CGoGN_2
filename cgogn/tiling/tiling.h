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


#ifndef TILING_TILING_H_
#define TILING_TILING_H_

namespace cgogn
{

namespace tiling
{

/*! \brief The class of regular tiling
 */
template <typename MAP>
class Tiling
{

protected:
    /**
    * Map in which we are working
    */
    MAP& map_;

    /**
     * Dimensions of the tiling
     */
    unsigned int nx_, ny_, nz_;

    /**
     * Reference dart of Polyhedron
     */
    Dart dart_;

    /**
    * Table of vertex darts (one dart per vertex)
    * Order depend on tiling kind
    */
    std::vector<Dart> vertex_table_;

	/**
	* Table of vertex darts (one dart per vertex)
	* Order depend on tiling kind
	*/
	std::vector<Dart> face_table_;

public:
    Tiling(MAP& map, unsigned int x, unsigned int y, unsigned int z):
        map_(map),
        nx_(x), 
        ny_(y), 
        nz_(z)
	{}

    Tiling(MAP& map) :
        map_(map),
        nx_(-1), 
        ny_(-1), 
        nz_(-1)
	{}

    /**
	* get the table of vertex darts (one per vertex)
    */
    std::vector<Dart>& get_vertices() { return vertex_table_; }

	/**
	* get the table of face darts (one per face)
	*/
	std::vector<Dart>& get_faces() { return face_table_; }

};

} // namespace tiling

} // namespace cgogn

#endif // TILING_TILING_H_
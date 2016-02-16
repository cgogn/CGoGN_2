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

#ifndef IO_MAP_EXPORT_H_
#define IO_MAP_EXPORT_H_

#include <string>
#include <fstream>
#include <iomanip>
#include <iostream>



#include <core/cmap/cmap2.h>
//#include <core/cmap/cmap3.h>



namespace cgogn
{

namespace io
{

//template <typename VEC3, class MAP_TRAITS>
//void export_surface(cgogn::CMap2<MAP_TRAITS>& cmap2, const std::string& filename);

/**
 * @brief export surface in off format
 * @param map the map to export
 * @param position the position attribute of vertices
 * @param filename the name of file to save
 * @return ok ?
 */
template <typename VEC3, typename MAP>
bool export_off(MAP& map, const typename MAP::template VertexAttributeHandler<VEC3>& position, const std::string& filename)
{

	std::ofstream fp(filename.c_str(), std::ios::out);
	if (!fp.good())
	{
		std::cout << "Unable to open file " << filename << std::endl;
		return false;
	}

	fp << "OFF"<< std::endl;
	fp << map.template nb_cells<MAP::VERTEX>() << " "<< map.template nb_cells<MAP::FACE>() << " 0"<< std::endl; // nb_edge unused ?

	// set precision for real output
	fp<< std::setprecision(12);

	// two pass of traversal to avoid huge buffer (with same performance);

	// first pass to save positions & store contiguous indices
	typename MAP::template VertexAttributeHandler<unsigned int>  ids = map.template add_attribute<unsigned int, MAP::VERTEX>("indices");
	ids.get_data()->set_all_values(0xffffffff);
	unsigned int count = 0;
	map.template foreach_cell<MAP::FACE>([&] (typename MAP::Face f)
	{
		map.template foreach_incident_vertex(f, [&] (typename MAP::Vertex v)
		{
			if (ids[v]==0xffffffff)
			{
				ids[v] = count++;
				const VEC3& P = position[v];
				fp << P[0] << " " << P[1] << " " << P[2] << std::endl;
			}
		});
	});

	// second pass to save primitives
	std::vector<unsigned int> prim;
	prim.reserve(20);
	map.template foreach_cell<MAP::FACE>([&] (typename MAP::Face f)
	{
		unsigned int valence = 0;
		prim.clear();

		map.template foreach_incident_vertex(f, [&] (typename MAP::Vertex v)
		{
			prim.push_back(ids[v]);
			++valence;
		});
		fp << valence;
		for(unsigned int i: prim)
			fp << " " << i;
		fp << std::endl;

	});

	map.remove_attribute(ids);
	fp.close();
	return true;
}


/**
 * @brief export surface in off format
 * @param map the map to export
 * @param position the position attribute of vertices
 * @param filename the name of file to save
 * @return ok ?
 */
template <typename VEC3, typename MAP>
bool export_off_bin(MAP& map, const typename MAP::template VertexAttributeHandler<VEC3>& position, const std::string& filename)
{

	// local function for little/big endian conversion
	auto changeEndianness = [](unsigned int x) -> unsigned int
	{
		return (x>>24) | ((x<<8) & 0x00FF0000) | ((x>>8) & 0x0000FF00) |  (x<<24);
	};


	std::ofstream fp(filename.c_str(), std::ios::out|std::ofstream::binary);
	if (!fp.good())
	{
		std::cout << "Unable to open file " << filename << std::endl;
		return false;
	}

	fp << "OFF BINARY"<< std::endl;

	unsigned int nb_cells[3];
	nb_cells[0] = changeEndianness(map.template nb_cells<MAP::VERTEX>());
	nb_cells[1] = changeEndianness(map.template nb_cells<MAP::FACE>());
	nb_cells[2] = 0;

	fp.write(reinterpret_cast<char*>(nb_cells),3*sizeof(unsigned int));

	// two pass of traversal to avoid huge buffer (with same performance);

	// first pass to save positions & store contiguous indices
	typename MAP::template VertexAttributeHandler<unsigned int>  ids = map.template add_attribute<unsigned int, MAP::VERTEX>("indices");
	ids.get_data()->set_all_values(0xffffffff);
	unsigned int count = 0;

	static const unsigned int BUFFER_SZ = 1024*1024;

	std::vector<float> buffer_pos;
	buffer_pos.reserve(BUFFER_SZ+3);

	map.template foreach_cell<MAP::FACE>([&] (typename MAP::Face f)
	{
		map.template foreach_incident_vertex(f, [&] (typename MAP::Vertex v)
		{
			if (ids[v]==0xffffffff)
			{
				ids[v] = count++;
				VEC3 P = position[v];
				// VEC3 can be double !
				float Pf[3]={float(P[0]),float(P[1]),float(P[2])};
				unsigned int* ui_vec = reinterpret_cast<unsigned int*>(Pf);
				ui_vec[0] = changeEndianness(ui_vec[0]);
				ui_vec[1] = changeEndianness(ui_vec[1]);
				ui_vec[2] = changeEndianness(ui_vec[2]);

				buffer_pos.push_back(Pf[0]);
				buffer_pos.push_back(Pf[1]);
				buffer_pos.push_back(Pf[2]);

				if (buffer_pos.size() >= BUFFER_SZ)
				{
					fp.write(reinterpret_cast<char*>(&(buffer_pos[0])),buffer_pos.size()*sizeof(float));
					buffer_pos.clear();
				}
			}
		});
	});
	if (!buffer_pos.empty())
	{
		fp.write(reinterpret_cast<char*>(&(buffer_pos[0])),buffer_pos.size()*sizeof(float));
		buffer_pos.clear();
		buffer_pos.shrink_to_fit();
	}

	// second pass to save primitives
	std::vector<unsigned int> buffer_prims;
	buffer_prims.reserve(BUFFER_SZ+128);// + 128 to avoid re-allocations

	std::vector<unsigned int> prim;
	prim.reserve(20);
	map.template foreach_cell<MAP::FACE>([&] (typename MAP::Face f)
	{
		unsigned int valence = 0;
		prim.clear();

		map.template foreach_incident_vertex(f, [&] (typename MAP::Vertex v)
		{
			prim.push_back(ids[v]);
			++valence;
		});

		buffer_prims.push_back(changeEndianness(valence));
		for(unsigned int i: prim)
			buffer_prims.push_back(changeEndianness(i));

		if (buffer_prims.size() >= BUFFER_SZ)
		{
			fp.write(reinterpret_cast<char*>(&(buffer_prims[0])),buffer_prims.size()*sizeof(unsigned int));
			buffer_prims.clear();
		}
	});
	if (!buffer_prims.empty())
	{
		fp.write(reinterpret_cast<char*>(&(buffer_prims[0])),buffer_prims.size()*sizeof(unsigned int));
		buffer_prims.clear();
		buffer_prims.shrink_to_fit();
	}

	map.remove_attribute(ids);
	fp.close();
	return true;
}



/**
 * @brief export surface in obj format
 * @param map the map to export
 * @param position the position attribute of vertices
 * @param filename the name of file to save
 * @return ok ?
 */
template <typename VEC3, typename MAP>
bool export_obj(MAP& map, const typename MAP::template VertexAttributeHandler<VEC3>& position, const std::string& filename)
{
	std::ofstream fp(filename.c_str(), std::ios::out);
	if (!fp.good())
	{
		std::cout << "Unable to open file " << filename << std::endl;
		return false;
	}

	// set precision for float output
	fp<< std::setprecision(12);

	// two passes of traversal to avoid huge buffer (with same performance);
	// first pass to save positions & store contiguous indices (from 1 because of obj format)
	typename MAP::template VertexAttributeHandler<unsigned int>  ids = map.template add_attribute<unsigned int, MAP::VERTEX>("indices");
	ids.get_data()->set_all_values(0xffffffff);
	unsigned int count = 1;
	map.template foreach_cell<MAP::FACE>([&] (typename MAP::Face f)
	{
		map.template foreach_incident_vertex(f, [&] (typename MAP::Vertex v)
		{
			if (ids[v]==0xffffffff)
			{
				ids[v] = count++;
				const VEC3& P = position[v];
				fp <<"v " << P[0] << " " << P[1] << " " << P[2] << std::endl;
			}
		});
	});

	// second pass to save primitives
	std::vector<unsigned int> prim;
	prim.reserve(20);
	map.template foreach_cell<MAP::FACE>([&] (typename MAP::Face f)
	{
		fp << "f";
		map.template foreach_incident_vertex(f, [&] (typename MAP::Vertex v)
		{
			fp << " " << ids[v];
		});
		fp << std::endl;
	});

	map.remove_attribute(ids);
	fp.close();
	return true;
}



} // namespace io

} // namespace cgogn

#endif // IO_MAP_IMPORT_H_

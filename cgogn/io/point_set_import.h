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

#ifndef CGOGN_IO_POINT_SET_IMPORT_H_
#define CGOGN_IO_POINT_SET_IMPORT_H_

#include <istream>
#include <sstream>
#include <set>

#include <cgogn/core/utils/endian.h>
#include <cgogn/core/utils/name_types.h>
#include <cgogn/core/utils/string.h>

#include <cgogn/core/cmap/cmap0.h>

#include <cgogn/io/dll.h>
#include <cgogn/io/c_locale.h>
#include <cgogn/io/mesh_io_gen.h>
#include <cgogn/io/data_io.h>

namespace cgogn
{

namespace io
{

template <typename MAP>
class PointSetImport
{
public:
	static_assert(MAP::DIMENSION == 0, "Must use map of dimension 0 in point set import");

	using Self = PointSetImport<MAP>;

	using ChunkArrayContainer = MapBaseData::ChunkArrayContainer<uint32>;
	using ChunkArrayGen = MapBaseData::ChunkArrayGen;
	template <typename T>
	using ChunkArray = MapBaseData::ChunkArray<T>;

	using Vertex = typename MAP::Vertex;
	using MapBuilder = typename MAP::Builder;

	template <typename T, Orbit ORBIT>
	using Attribute = Attribute<T, ORBIT>;

	using DataInputGen = cgogn::io::DataInputGen;

	inline PointSetImport(MAP& map) :
		map_(map),
		mbuild_(map)
	{
		map_.clear_and_remove_attributes();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(PointSetImport);
	virtual ~PointSetImport() {}

	inline ChunkArrayContainer& vertex_container()
	{
		return mbuild_.template attribute_container<Vertex::ORBIT>();
	}

	inline uint32 insert_line_vertex_container()
	{
		return vertex_container().template insert_lines<1>();
	}

	inline void copy_line_vertex_container(uint32 dest, uint32 src)
	{
		vertex_container().copy_line(dest, src, false, false);
	}

	inline ChunkArrayGen* add_vertex_attribute(const DataInputGen& in_data, const std::string& att_name)
	{
		ChunkArrayGen* att = in_data.add_attribute(vertex_container(), att_name);
		in_data.to_chunk_array(att);
		return att;
	}

	template<typename T>
	inline ChunkArray<T>* add_vertex_attribute(const std::string& att_name)
	{
		return vertex_container().template add_chunk_array<T>(att_name);
	}

	inline uint32 nb_vertices() const
	{
		return nb_vertices_;
	}

	void create_map()
	{
		if (nb_vertices() == 0u)
			return;
		mbuild_.template create_embedding<Vertex::ORBIT>();

		for (uint32 i = 0, end = nb_vertices(); i < end; ++i)
		{
			Dart d = mbuild_.add_topology_element();
			mbuild_.template set_orbit_embedding<Vertex>(Vertex(d), i);
		}
	}

protected:
	uint32 nb_vertices_;
	MAP&              map_;
	MapBuilder        mbuild_;
};

template <typename MAP>
class PointSetFileImport : public PointSetImport<MAP>, public FileImport
{
		using Self = PointSetFileImport<MAP>;
		using Inherit_Import = PointSetImport<MAP>;
		using Inherit_File = FileImport;

		CGOGN_NOT_COPYABLE_NOR_MOVABLE(PointSetFileImport);

public:

		inline PointSetFileImport(MAP& map) : Inherit_Import(map), Inherit_File() {}
		virtual ~PointSetFileImport() {}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_EXTERNAL_TEMPLATES_CPP_))
extern template class CGOGN_IO_API PointSetImport<CMap0>;
extern template class CGOGN_IO_API PointSetFileImport<CMap0>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_EXTERNAL_TEMPLATES_CPP_))

} // end namespace io

} // end namespace cgogn

#endif // CGOGN_IO_POINT_SET_IMPORT_H_

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

#ifndef CGOGN_IO_POLYLINE_IMPORT_H_
#define CGOGN_IO_POLYLINE_IMPORT_H_

#include <istream>
#include <sstream>
#include <set>

#include <cgogn/core/utils/endian.h>
#include <cgogn/core/utils/name_types.h>
#include <cgogn/core/utils/string.h>

#include <cgogn/core/cmap/cmap1.h>

#include <cgogn/io/dll.h>
#include <cgogn/io/c_locale.h>
#include <cgogn/io/mesh_io_gen.h>
#include <cgogn/io/data_io.h>

namespace cgogn
{

namespace io
{

template <typename MAP>
class PolylineImport
{
public:
	static_assert(MAP::DIMENSION == 1, "Must use map of dimension 1 in polyline import");

	using Self = PolylineImport<MAP>;

	using ChunkArrayContainer = MapBaseData::ChunkArrayContainer<uint32>;
	using ChunkArrayGen = MapBaseData::ChunkArrayGen;
	template <typename T>
	using ChunkArray = MapBaseData::ChunkArray<T>;

	using Vertex = typename MAP::Vertex;
	using MapBuilder = typename MAP::Builder;

	template <typename T, Orbit ORBIT>
	using Attribute = Attribute<T, ORBIT>;

	using DataInputGen = cgogn::io::DataInputGen;

	inline PolylineImport(MAP& map) :
		map_(map),
		mbuild_(map)
	{
		map_.clear_and_remove_attributes();
	}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(PolylineImport);
	virtual ~PolylineImport() {}

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

	inline uint32 nb_edges() const
	{
		return uint32(edges_vertex_indices_.size() / 2 );
	}

	void create_map()
	{
		if (nb_edges() == 0u)
			return;

		mbuild_.template create_embedding<Vertex::ORBIT>();

		auto darts_out_vertex = map_.template add_attribute<Dart, Vertex>("darts_out_vertex");

		Dart first;
		for (uint32 i = 0, end = edges_vertex_indices_.size(); i < end; i=i+2)
		{
			Dart d = mbuild_.add_topology_element();
			first = d;
			uint32 idx = edges_vertex_indices_[i];
			uint32 idx2 = edges_vertex_indices_[i+1];
			mbuild_.template set_orbit_embedding<Vertex>(Vertex(d), idx);
			std::cout << idx << " " << idx2 << std::endl;
			darts_out_vertex[idx2] = d;
		}

		Dart last = mbuild_.add_topology_element();
		uint32 idx = edges_vertex_indices_[edges_vertex_indices_.size()-1];
		mbuild_.template set_orbit_embedding<Vertex>(Vertex(last), idx);
		darts_out_vertex[idx] = first;

		uint32 nb_boundary_vertex = 0;

		map_.foreach_dart([&] (Dart d)
		{
			if (map_.phi1(d) == d)
			{
				uint32 vertex_index = map_.embedding(Vertex(d));
				Dart prev_vertex_darts = darts_out_vertex[vertex_index];

				if(prev_vertex_darts.is_nil())
					++nb_boundary_vertex;
				else
					mbuild_.phi1_sew(d, prev_vertex_darts);
			}
		});

		mbuild_.boundary_mark(Vertex(last));

		if(nb_boundary_vertex > 0)
		{
			cgogn_log_info("create_map") << nb_boundary_vertex << " hole(s) have been closed";
		}

		map_.remove_attribute(darts_out_vertex);
	}

protected:

	std::vector<uint32> edges_vertex_indices_;

	uint32 nb_vertices_;
	MAP&              map_;
	MapBuilder        mbuild_;
};

template <typename MAP>
class PolylineFileImport : public PolylineImport<MAP>, public FileImport
{
	using Self = PolylineFileImport<MAP>;
	using Inherit_Import = PolylineImport<MAP>;
	using Inherit_File = FileImport;

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(PolylineFileImport);

public:

	inline PolylineFileImport(MAP& map) : Inherit_Import(map), Inherit_File() {}
	virtual ~PolylineFileImport() {}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_EXTERNAL_TEMPLATES_CPP_))
extern template class CGOGN_IO_API PolylineImport<CMap1>;
extern template class CGOGN_IO_API PolylineFileImport<CMap1>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_EXTERNAL_TEMPLATES_CPP_))

} // end namespace io

} // end namespace cgogn

#endif // CGOGN_IO_POLYLINE_IMPORT_H_

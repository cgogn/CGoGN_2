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

#ifndef CGOGN_IO_MESH_IO_GEN_H_
#define CGOGN_IO_MESH_IO_GEN_H_

#include <fstream>

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/utils/masks.h>
#include <cgogn/io/c_locale.h>
#include <cgogn/io/io_utils.h>

namespace cgogn
{

namespace io
{

class CGOGN_IO_API FileImport
{
public:

	using Self = FileImport;

	FileImport();

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(FileImport);

	virtual ~FileImport();

	/**
	 * @brief import_file, try to open and read a file (during the operation the C locale is used)
	 * @param filename
	 * @return true iff the file is successfully read
	 */
	bool import_file(const std::string& filename);

protected:

	virtual bool import_file_impl(const std::string& filename) = 0;
};

template<typename MAP>
class MeshExport
{
public:

	using Map = MAP;
	using Self = MeshExport<Map>;
	using Vertex = typename Map::Vertex;
	using ChunkArrayGen = typename Map::ChunkArrayGen;
	using ChunkArrayContainer = typename Map::template ChunkArrayContainer<uint32>;
	template<typename T>
	using VertexAttribute = typename Map::template VertexAttribute<T>;
	using CellCache = typename Map::CellCache;

	inline MeshExport() :
		cell_cache_(nullptr)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MeshExport);

	virtual ~MeshExport() {}

	void export_file(Map& map, const ExportOptions& options)
	{
		Scoped_C_Locale loc;
		this->reset();

		this->cell_cache_ = cgogn::make_unique<CellCache>(map);
		cgogn_assert(cell_cache_);

		auto output = io::create_file(options.filename_, options.binary_, options.overwrite_);
		if (!output || !output->good())
			return;

		if(options.position_attributes_.empty())
		{
			cgogn_log_warning("MeshExport::export_file") << "The position attribute is empty.";
			return;
		}

		const Orbit orb = Vertex::ORBIT;
		if(options.position_attributes_.find(orb) == options.position_attributes_.end())
		{
			cgogn_log_warning("MeshExport::export_file") << "The VERTEX ORBIT position attribute do not exist.";
			return;
		}
		else
			position_attributes_.insert(std::make_pair(
					orb,
					map.template attribute_container<Vertex::ORBIT>().get_chunk_array(options.position_attributes_.at(orb)))
				);

		vindices_ = map.template add_attribute<uint32, Vertex>("indices_vert_export");

		this->prepare_for_export(map, options);

		this->export_file_impl(map,*output, options);
		this->clean_added_attributes(map);
	}

protected:

	inline std::vector<const ChunkArrayGen*> const & vertex_attributes() const
	{
		return vertex_attributes_;
	}

	virtual void clean_added_attributes(Map& map)
	{
		map.remove_attribute(vindices_);
	}

	inline const ChunkArrayGen* position_attribute(const Orbit orb) const
	{
		auto it = position_attributes_.find(orb);
		return it == position_attributes_.end() ? nullptr : it->second;
	}

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& options) = 0;
	virtual void prepare_for_export(Map& map, const ExportOptions& options) = 0;
	virtual void reset()
	{
		position_attributes_.clear();
		vertex_attributes_.clear();
		cell_cache_.reset();
	}

	VertexAttribute<uint32> vindices_;
	std::map<Orbit, const ChunkArrayGen*> position_attributes_;
	std::vector<const ChunkArrayGen*> vertex_attributes_;
	std::unique_ptr<CellCache>        cell_cache_;
};

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_MESH_IO_GEN_H_

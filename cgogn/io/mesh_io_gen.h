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
	/**
	 * @brief import_file, try to open and read a file (during the operation the C locale is used)
	 * @param filename
	 * @return true iff the file is successfully read
	 */
	bool import_file(const std::string& filename);
	virtual ~FileImport();
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
	using CellCache = cgogn::CellCache<Map>;

	inline MeshExport() :
		position_attribute_(nullptr),
		cell_cache_(nullptr)
	{}

	CGOGN_NOT_COPYABLE_NOR_MOVABLE(MeshExport);

	void export_file(Map& map, const ExportOptions& options)
	{
		Scoped_C_Locale loc;
		this->reset();

		this->cell_cache_ = cgogn::make_unique<CellCache>(map);
		cgogn_assert(cell_cache_);

		auto output = io::create_file(options.filename_, options.binary_, options.overwrite_);
		if (!output || !output->good())
			return;

		map.add_attribute(indices_, "indices_vert_export");

		this->prepare_for_export(map, options);

		if (position_attribute_ == nullptr)
		{
			cgogn_log_warning("MeshExport::export_file") << "The position attribute is invalid.";
			map.remove_attribute(indices_);
			return;
		}

		this->export_file_impl(map,*output, options);
		this->clean_added_attributes(map);
	}

	virtual ~MeshExport() {}
protected:
	inline std::vector<ChunkArrayGen*> const & vertex_attributes() const
	{
		return vertex_attributes_;
	}

	virtual void clean_added_attributes(Map& map)
	{
		map.remove_attribute(indices_);
	}

	ChunkArrayGen const * position_attribute() const
	{
		return position_attribute_;
	}

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& options) = 0;
	virtual void prepare_for_export(Map& map, const ExportOptions& options) = 0;
	virtual void reset()
	{
		position_attribute_ = nullptr;
		vertex_attributes_.clear();
		cell_cache_.reset();
	}

	VertexAttribute<uint32>		indices_;
	std::vector<ChunkArrayGen*>	vertex_attributes_;
	ChunkArrayGen*				position_attribute_;
	std::unique_ptr<CellCache>	cell_cache_;
};


} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_MESH_IO_GEN_H_

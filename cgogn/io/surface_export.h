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

#ifndef CGOGN_IO_SURFACE_EXPORT_H_
#define CGOGN_IO_SURFACE_EXPORT_H_

#include <ostream>
#include <iomanip>

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/utils/string.h>
#include <cgogn/core/container/chunk_array_container.h>
#include <cgogn/core/cmap/cmap3.h>

#include <cgogn/io/io_utils.h>
#include <cgogn/io/mesh_io_gen.h>

namespace cgogn
{

namespace io
{

template <typename MAP>
class SurfaceExport : public MeshExport<MAP>
{
public:

	using Inherit = MeshExport<MAP>;
	using Self = SurfaceExport<MAP>;
	using Map = MAP;

	using Vertex = typename Map::Vertex;
	using Edge = typename Map::Edge;
	using Face = typename Map::Face;

	using ChunkArrayGen = typename Map::ChunkArrayGen;
	using ChunkArrayContainer = typename Map::template ChunkArrayContainer<uint32>;
	template<typename T>
	using EdgeAttribute = typename Map::template EdgeAttribute<T>;
	template<typename T>
	using FaceAttribute = typename Map::template FaceAttribute<T>;

	inline SurfaceExport() {}
	virtual ~SurfaceExport() {}

protected:

	inline std::vector<const ChunkArrayGen*> const & face_attributes() const
	{
		return face_attributes_;
	}

	void clean_added_attributes(Map& map) override
	{
		Inherit::clean_added_attributes(map);
		if(eindices_.is_valid())
			map.remove_attribute(eindices_);
		if(findices_.is_valid())
			map.remove_attribute(findices_);
	}

	inline uint32 nb_vertices() const
	{
		return uint32(this->cell_cache_->template size<Vertex>());
	}

	inline uint32 nb_faces() const
	{
		return uint32(this->cell_cache_->template size<Face>());
	}

private:

	virtual void prepare_for_export(Map& map, const ExportOptions& options) override
	{
		const ChunkArrayContainer& ver_cac = map.template attribute_container<Vertex::ORBIT>();
		const ChunkArrayContainer& edge_cac = map.template attribute_container<Edge::ORBIT>();
		const ChunkArrayContainer& face_cac = map.template attribute_container<Face::ORBIT>();

		for(const auto& pair : options.position_attributes_)
		{
			const ChunkArrayGen* pos_cag = nullptr;

			if(pair.first == Edge::ORBIT)
				pos_cag = edge_cac.get_chunk_array(pair.second);
			else if(pair.first == Face::ORBIT)
				pos_cag = face_cac.get_chunk_array(pair.second);

			if(pos_cag)
				this->position_attributes_.insert(std::make_pair(pair.first, pos_cag));
		}

		for (const auto& pair : options.attributes_to_export_)
		{
			if (pair.first == Vertex::ORBIT)
			{
				const ChunkArrayGen* ver_cag = ver_cac.get_chunk_array(pair.second);
				if (ver_cag)
					this->vertex_attributes_.push_back(ver_cag);
			}
			else if(pair.first == Edge::ORBIT)
			{
				const ChunkArrayGen* edge_cag = edge_cac.get_chunk_array(pair.second);
				if (edge_cag)
					edge_attributes_.push_back(edge_cag);
			}
			else if(pair.first == Face::ORBIT)
			{
				const ChunkArrayGen* face_cag = face_cac.get_chunk_array(pair.second);
				if (face_cag)
					face_attributes_.push_back(face_cag);
			}
		}

		this->cell_cache_->template build<Vertex>();
		this->cell_cache_->template build<Face>();
		uint32 count{0u};
		map.foreach_cell(
			[&] (Vertex v) { this->vindices_[v] = count++; },
			*(this->cell_cache_)
		);

		const Orbit eorb = Edge::ORBIT;
		if(this->position_attributes_.find(eorb) != this->position_attributes_.end())
		{
			this->cell_cache_->template build<Edge>();
			eindices_ = map.template add_attribute<uint32, Edge>("indices_vert_export");
			map.foreach_cell(
				[&] (Edge e) { eindices_[e] = count++; },
				*(this->cell_cache_)
			);
		}
		const Orbit forb = Face::ORBIT;
		if(this->position_attributes_.find(forb) != this->position_attributes_.end())
		{
			findices_ = map.template add_attribute<uint32, Face>("indices_vert_export");
			map.foreach_cell(
				[&] (Face f) { findices_[f] = count++; },
				*(this->cell_cache_)
			);
		}
	}

	virtual void reset() override
	{
		Inherit::reset();
		edge_attributes_.clear();
		face_attributes_.clear();
	}

	std::vector<const ChunkArrayGen*>	edge_attributes_;
	std::vector<const ChunkArrayGen*>	face_attributes_;	

protected:
	EdgeAttribute<uint32> eindices_;
	FaceAttribute<uint32> findices_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_SURFACE_EXPORT_CPP_))
extern template class CGOGN_IO_API SurfaceExport<CMap2>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_SURFACE_EXPORT_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_SURFACE_EXPORT_H_

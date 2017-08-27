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

#ifndef CGOGN_IO_GRAPH_EXPORT_H_
#define CGOGN_IO_GRAPH_EXPORT_H_

#include <cgogn/io/dll.h>

#include <cgogn/core/graph/undirected_graph.h>

#include <cgogn/io/mesh_io_gen.h>

namespace cgogn
{

namespace io
{

template <typename MAP>
class GraphExport : public MeshExport<MAP>
{
public:
	using Inherit = MeshExport<MAP>;
	using Self = GraphExport<MAP>;
	using Map = MAP;

	using Vertex = typename Map::Vertex;
	using Edge = typename Map::Edge;

	using ChunkArrayGen = typename Map::ChunkArrayGen;
	using ChunkArrayContainer = typename Map::template ChunkArrayContainer<uint32>;

	inline GraphExport()
	{}

	virtual ~GraphExport()
	{}
	
protected:

	inline std::vector<const ChunkArrayGen*> const & edge_attributes() const
	{
		return edge_attributes_;
	}

	void clean_added_attributes(Map& map) override
	{
		Inherit::clean_added_attributes(map);
	}

	inline uint32 nb_vertices() const
	{
		return uint32(this->cell_cache_->template size<Vertex>());
	}

	inline uint32 nb_edges() const
	{
		return uint32(this->cell_cache_->template size<Edge>());
	}

private:

	virtual void prepare_for_export(Map& map, const ExportOptions& options) override
	{
		const ChunkArrayContainer& ver_cac = map.template attribute_container<Vertex::ORBIT>();
		const ChunkArrayContainer& edge_cac = map.template attribute_container<Edge::ORBIT>();

		for (const auto& pair : options.attributes_to_export_)
		{
			if (pair.first == Vertex::ORBIT)
			{
				const ChunkArrayGen* ver_cag = ver_cac.get_chunk_array(pair.second);
				if (ver_cag)
					this->vertex_attributes_.push_back(ver_cag);
			} else {
				const ChunkArrayGen* edge_cag = edge_cac.get_chunk_array(pair.second);
				if (edge_cag)
					edge_attributes_.push_back(edge_cag);
			}
		}

		this->cell_cache_->template build<Vertex>();
		this->cell_cache_->template build<Edge>();
		uint32 count{0u};
		map.foreach_cell([&] (Vertex v)
		{
			this->vindices_[v] = count++;
		}, *(this->cell_cache_));
	}

	virtual void reset() override
	{
		Inherit::reset();
		edge_attributes_.clear();
	}

	std::vector<const ChunkArrayGen*> edge_attributes_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_GRAPH_EXPORT_CPP_))
extern template class CGOGN_IO_API GraphExport<UndirectedGraph>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_GRAPH_EXPORT_CPP_))

} // naemspace io

} // namespace cgogn

#endif // CGOGN_IO_GRAPH_EXPORT_H_

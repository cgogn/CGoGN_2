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

#ifndef CGOGN_IO_OVM_IO_H_
#define CGOGN_IO_OVM_IO_H_

#include <map>
#include <sstream>
#include <iomanip>

#include <cgogn/core/utils/logger.h>
#include <cgogn/io/dll.h>
#include <cgogn/io/data_io.h>
#include <cgogn/io/volume_import.h>
#include <cgogn/io/volume_export.h>

namespace cgogn
{

namespace io
{

template <typename VEC3>
class OVMVolumeImport : public VolumeFileImport<VEC3>
{
	using Inherit_Nastran = OVMVolumeImport<VEC3>;
	using Inherit_Import = VolumeFileImport<VEC3>;
	using Self = OVMVolumeImport<VEC3>;
	template <typename T>
	using ChunkArray = typename Inherit_Import::template ChunkArray<T>;

protected:

	virtual bool import_file_impl(const std::string& filename) override
	{
		// TODO
//		std::ifstream file(filename, std::ios::in);
//		ChunkArray<VEC3>* position = this->position_attribute();

//		std::string line;
//		line.reserve(512);

//		getline_safe (file, line);

		return true;
	}
};

template <typename MAP>
class OVMVolumeExport : public VolumeExport<MAP>
{
public:

	using Inherit = VolumeExport<MAP>;
	using Self = OVMVolumeExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;
	using Edge = typename Inherit::Edge;
	using Face = typename Inherit::Face;
	using Volume = typename Inherit::Volume;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;

protected:

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& /*option*/) override
	{

		ChunkArrayGen const* pos = this->position_attribute();

		// The volume not present in the CellCache won't be exported.
		// We need to mark every edges and faces belonging to a volume that is going to be exported.
		typename Map::template CellMarker<Edge::ORBIT> emarker(map);
		typename Map::template CellMarker<Face::ORBIT> fmarker(map);
		std::vector<Edge> edges;
		std::vector<Face> faces;
		edges.reserve(1024);
		faces.reserve(1024);

		map.foreach_cell([&](Volume w)
		{
			map.foreach_incident_edge(w,[&](Edge e)
			{
				if (!emarker.is_marked(e))
				{
					emarker.mark(e);
					edges.push_back(e);
				}
			});
			map.foreach_incident_face(w,[&](Face f)
			{
				if (!fmarker.is_marked(f))
				{
					fmarker.mark(f);
					faces.push_back(f);
				}
			});
		}, *(this->cell_cache_));

		output << "OVM ASCII  # Exported using CGoGN_2 github.com/cgogn/CGoGN_2" << std::endl;
		output << std::endl;

		output << "Vertices" << std::endl;
		output << this->nb_vertices() << std::endl;
		map.foreach_cell([&](Vertex v)
		{
			pos->export_element(map.embedding(v), output, false, false);
			output << std::endl;
		}, *(this->cell_cache_));

		output << "Edges" << std::endl;
		output << edges.size() << std::endl;
		for(Edge e : edges)
			output << map.embedding(Vertex(e.dart)) << " " << map.embedding(Vertex(map.phi2(e.dart))) << std::endl;

		output << "Faces" << std::endl;
		output << faces.size() << std::endl;
		for (Face f : faces)
		{
			output << map.codegree(f);
			Dart it = f.dart;
			do {
				output << " " << map.embedding(Vertex(it));
				it = map.phi1(it);
			} while (it != f.dart);
			output << std::endl;
		};

		output << "Polyhedra" << std::endl;
		output << this->nb_volumes();
		map.foreach_cell([&](Volume w)
		{
			output << std::endl;
			output << this->vertices_of_volumes(w).size();
			for (uint32 v : this->vertices_of_volumes(w))
				output << " " << v;
		}, *(this->cell_cache_));
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_OVM_IO_CPP_))
extern template class CGOGN_IO_API OVMVolumeImport<Eigen::Vector3d>;
extern template class CGOGN_IO_API OVMVolumeImport<Eigen::Vector3f>;
extern template class CGOGN_IO_API OVMVolumeImport<geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API OVMVolumeImport<geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API OVMVolumeExport<CMap3>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_OVM_IO_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_OVM_IO_H_

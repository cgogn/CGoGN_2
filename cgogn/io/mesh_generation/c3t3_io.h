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

#ifndef IO_C3T3_IO_H_
#define IO_C3T3_IO_H_

#include <map>
#include <io/volume_import.h>

#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/refine_mesh_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Labeled_image_mesh_domain_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_criteria_3.h>


namespace cgogn
{

namespace io
{

struct VolumeMeshFromImageCGALTraits
{
	using Image			= CGAL::Image_3;
	using Kernel		= CGAL::Exact_predicates_inexact_constructions_kernel;
	using Domain		= CGAL::Labeled_image_mesh_domain_3<Image, Kernel>;
	using Triangulation	= typename CGAL::Mesh_triangulation_3<Domain>::type;
	using Criteria		= CGAL::Mesh_criteria_3<Triangulation>;
	using C3T3			= typename CGAL::Mesh_complex_3_in_triangulation_3<Triangulation>;
};


template<typename C3T3, typename MAP_TRAITS, typename VEC3>
class C3T3VolumeImport : public VolumeImport<MAP_TRAITS>
{
public:
	using Inherit = VolumeImport<MAP_TRAITS>;
	using Self = C3T3VolumeImport<C3T3, MAP_TRAITS,VEC3>;

	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	using Triangulation				= typename C3T3::Triangulation;
	using Vertex_handle				= typename Triangulation::Vertex_handle;

	C3T3VolumeImport() = delete;
	C3T3VolumeImport(const Self&) = delete;
	C3T3VolumeImport(Self&&) = delete;
	Self& operator=(const Self&) = delete;
	Self& operator=(Self&&) = delete;

	inline C3T3VolumeImport(const C3T3& cpx) : Inherit(),
		cpx_(cpx)
	{}

protected:
	virtual bool import_file_impl(const std::string& /*filename*/) override
	{
		const Triangulation& triangulation = cpx_.triangulation();
		std::map<Vertex_handle, unsigned int> vertices_indices;
		ChunkArray<VEC3>* position = this->vertex_attributes_.template add_attribute<VEC3>("position");

		const unsigned int num_vertices = triangulation.number_of_vertices();
		const unsigned int num_cells = cpx_.number_of_cells_in_complex();

		this->volumes_nb_vertices_.reserve(num_cells);
		this->volumes_vertex_indices_.reserve(4u*num_cells);
		this->nb_vertices_ = num_vertices;
		this->nb_volumes_ = num_cells;

		for (auto vit = triangulation.finite_vertices_begin(), vend = triangulation.finite_vertices_end(); vit != vend; ++vit)
		{
			const auto& P = vit->point();
			const unsigned id = this->vertex_attributes_.template insert_lines<1>();
			vertices_indices[vit] = id;
			position->operator [](id) = VEC3(Scalar(P.x()), Scalar(P.y()), Scalar(P.z()));
		}

		for (auto cit = cpx_.cells_in_complex_begin(), cend = cpx_.cells_in_complex_end(); cit != cend; ++cit)
			this->add_tetra(*position, vertices_indices[cit->vertex(0)], vertices_indices[cit->vertex(1)], vertices_indices[cit->vertex(2)], vertices_indices[cit->vertex(3)], true);

		ChunkArray<int>* subdomain_indices = this->volume_attributes_.template add_attribute<int>("subdomain index");
		for (auto cit = cpx_.cells_in_complex_begin(), cend = cpx_.cells_in_complex_end(); cit != cend; ++cit)
		{
			const unsigned id = this->volume_attributes_.template insert_lines<1>();
			subdomain_indices->operator [](id) = cpx_.subdomain_index(cit);
		}

		return true;
	}
private:
	C3T3 cpx_;
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_C3T3_IO_CPP_))
extern template class CGOGN_IO_API C3T3VolumeImport<VolumeMeshFromImageCGALTraits::C3T3, DefaultMapTraits, Eigen::Vector3d>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_C3T3_IO_CPP_))

} // namespace io
} // namespace cgogn



#endif // IO_C3T3_IO_H_

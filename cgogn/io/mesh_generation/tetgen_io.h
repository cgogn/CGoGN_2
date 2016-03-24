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

#ifndef IO_TETGEN_IO_H
#define IO_TETGEN_IO_H

#include <memory>
#include <io/dll.h>
#include <io/volume_import.h>

#include <tetgen.h>
//template <typename PFP>
//bool tetrahedralize(const typename PFP::MAP2& map2, const VertexAttribute<typename PFP::VEC3> position2,
//					typename PFP::MAP3& map3, VertexAttribute<typename PFP::VEC3> position3,
//					bool add_steiner_points_on_exterior_boundary, bool add_steiner_points_on_interior_boundary, double max_volume, double max_shape)
//{
//	//
//	// 1. map to tetgen
//	//

//	tetgenio surface;

//	// memory initialization
//	surface.initialize();

//	// 0-based indexing
//	surface.firstnumber = 0;

//	// input vertices
//	surface.numberofpoints = map2.nbOrbits<VERTEX>();
//	surface.pointlist = new REAL[surface.numberofpoints * 3];

//	//for each vertex
//	uint32 i = 0;
//	TraversorV tv(map2);
//	for(Dart it = tv.begin() ; it != tv.end() ; it = tv.next())
//	{
//		surface.pointlist[i] = position2[it][0] ; i++ ; //x
//		surface.pointlist[i] = position2[it][1] ; i++ ; //y
//		surface.pointlist[i] = position2[it][2] ; i++ ; //z
//	}

//	tetgenio::facet* f ;
//	tetgenio::polygon* p ;
//	surface.numberoffacets = map2.nbOrbits<FACE>();
//	surface.facetlist = new tetgenio::facet[surface.numberoffacets] ;


//	//for each facet
//	i = 0;
//	TraversorF tf(map2);
//	for(Dart it = tf.begin() ; it != tf.end() ; it = tf.next())
//	{
//		f = &(surface.facetlist[i]) ;
//		f->numberofpolygons = 1 ;
//		f->polygonlist = new tetgenio::polygon[f->numberofpolygons] ;
//		p = f->polygonlist ;
//		p->numberofvertices = map2.faceDegree(it);
//		p->vertexlist = new int[p->numberofvertices] ;

//		uint32 j = 0;
//		Dart dit = it;
//		do
//		{
//			p->vertexlist[j] = map2.getEmbedding<VERTEX>(dit);
//			dit = map.phi1(dit);
//			j++;
//		}while(dit != it);

//		f->numberofholes = 0 ;
//		f->holelist = nil ;
//		i++ ;
//	}

//	//
//	// 2. tetgen argument list
//	//
//	std::ostringstream s ;

//	// Q: Quiet: No terminal output except errors
//	// p: PLC : input data is surfacic
//	// n: output tet neighbors

//	// q: desired quality
//	if(max_volume > 0 && max_shape > 0.0)
//	{
//		s << "Qpna" << max_volume << "q"<< max_shape;
//	}
//	else if(max_volume > 0.0)
//	{
//		s << "Qpna" << max_volume ;
//	}
//	else if(max_shape > 0.0)
//	{
//		s << "Qpnq" << max_shape ;
//	}
//	else
//	{
//		s << "Qpn";
//	}

//	// YY: prohibit steiner points on boundaries
//	// (first Y for exterior boundary, second Y for the
//	// other ones).

//	if( add_steiner_points_on_exterior_boundary && !add_steiner_points_on_interior_boundary)
//	{
//	   //Invalid combination of flags (do not preserve exterior boundary and preserve interior ones) - preserving exterior boundary as well
//		add_steiner_points_on_exterior_boundary = false ;
//	}

//	if(!add_steiner_points_on_exterior_boundary)
//	{
//		s << "Y" ;
//	}

//	if(!add_steiner_points_on_interior_boundary)
//	{
//		s << "Y" ;
//	}
//	std::string params = s.str() ;

//	//
//	// 3. tetrahedralization
//	//
//	tetgenio volume;
//	::tetrahedralize(params.c_str(), &surface, &volume) ;


//	//
//	// 4. tetgen to map
//	//

//	//create vertices
//	float64* p = volume_->pointlist ;
//	std::vector<uint32> verticesID;
//	verticesID.reserve(volume_->numberofpoints);
//	AttributeContainer& container = map3.template getAttributeContainer<VERTEX>() ;

//	for(uint32 i = 0; i < volume_->numberofpoints; i++)
//	{
//		typename PFP::VEC3 pos(p[0], p[1], p[2]);
//		uint32 id = container.insertLine();

//		position3[id] = pos;
//		verticesID.push_back(id);

//		p += 3 ;
//	}

//	//create tetrahedrons
//	int* t = volume_->tetrahedronlist ;
//	for(uint32 i = 0; i < volume_->numberoftetrahedra; i++)
//	{
//		Dart d = Algo::Surface::Modelisation::createTetrahedron<PFP>(map3, false);

//		for(uint32 j = 0; j < 3; j++)
//		{
//			FunctorSetEmb<typename PFP::MAP, VERTEX> fsetemb(map, verticesID[t[j] - volume_->firstnumber]);
//			map.template foreach_dart_of_orbit<PFP::MAP::VERTEX_OF_PARENT>(d, fsetemb);

////            //store darts per vertices to optimize reconstruction
////            Dart dd = d;
////            do
////            {
////                m.mark(dd) ;
////                vecDartsPerVertex[pt[2-j]].push_back(dd);
////                dd = map.phi1(map.phi2(dd));
////            } while(dd != d);

//			d = map.phi1(d);

//			set_cell_vertex(d, j, verticesID[t[j] - volume_->firstnumber]) ;
//		}

//		t += 4 ;
//	}

//	//create adjacency
//	int* pn = volume_->neighborlist ;
//	for(uint32 i = 0; i < volume_->numberoftetrahedra; i++)
//	{
//		for(int j=0; j<4; j++)
//		{
//			int adjacent = pn[j] ;

//			if(adjacent >= 0)
//			{
//				set_cell_adjacent( cells[i], j, cells[adjacent - volume_->firstnumber]
//				) ;
//			}
//		}
//		pn += 4 ;
//	}
//}
namespace cgogn
{

namespace io
{


//	//
//	// 2. tetgen argument list
//	//
//	std::ostringstream s ;

//	// Q: Quiet: No terminal output except errors
//	// p: PLC : input data is surfacic
//	// n: output tet neighbors

//	// q: desired quality
//	if(max_volume > 0 && max_shape > 0.0)
//	{
//		s << "Qpna" << max_volume << "q"<< max_shape;
//	}
//	else if(max_volume > 0.0)
//	{
//		s << "Qpna" << max_volume ;
//	}
//	else if(max_shape > 0.0)
//	{
//		s << "Qpnq" << max_shape ;
//	}
//	else
//	{
//		s << "Qpn";
//	}

//	// YY: prohibit steiner points on boundaries
//	// (first Y for exterior boundary, second Y for the
//	// other ones).

//	if( add_steiner_points_on_exterior_boundary && !add_steiner_points_on_interior_boundary)
//	{
//		//Invalid combination of flags (do not preserve exterior boundary and preserve interior ones) - preserving exterior boundary as well
//		add_steiner_points_on_exterior_boundary = false ;
//	}

//	if(!add_steiner_points_on_exterior_boundary)
//	{
//		s << "Y" ;
//	}

//	if(!add_steiner_points_on_interior_boundary)
//	{
//		s << "Y" ;
//	}

//	//
//	// 3. tetrahedralization
//	//
//	tetgenio volume;
//	::tetrahedralize(s.str().c_str(), &output, &volume) ;


template<typename MAP_TRAITS, typename VEC3>
class TetgenVolumeImport : public VolumeImport<MAP_TRAITS>
{
public:
	using Inherit = VolumeImport<MAP_TRAITS>;
	using Self = TetgenVolumeImport<MAP_TRAITS,VEC3>;

	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;
	template<typename T>
	using ChunkArray = typename Inherit::template ChunkArray<T>;

	TetgenVolumeImport() = delete;
	TetgenVolumeImport(const Self&) = delete;
	TetgenVolumeImport(Self&&) = delete;
	Self& operator=(const Self&) = delete;
	Self& operator=(Self&&) = delete;

	explicit inline TetgenVolumeImport(tetgenio * tetgen_output)
	{
		volume_ = tetgen_output;
	}

protected:
	virtual bool import_file_impl(const std::string& /*filename*/) override
	{

		this->nb_vertices_ = volume_->numberofpoints;
		this->nb_volumes_ = volume_->numberoftetrahedra;
		this->volumes_nb_vertices_.reserve(this->nb_volumes_);
		this->volumes_vertex_indices_.reserve(4u*this->nb_volumes_);

		if (this->nb_vertices_ == 0u || this->nb_volumes_ == 0u)
		{
			this->clear();
			return false;
		}

		ChunkArray<VEC3>* position = this->vertex_attributes_.template add_attribute<VEC3>("position");

		//create vertices
		std::vector<uint32> vertices_indices;
		float64* p = volume_->pointlist ;
		vertices_indices.reserve(this->nb_vertices_);

		for(uint32 i = 0u; i < this->nb_vertices_; ++i)
		{
			const unsigned id = this->vertex_attributes_.template insert_lines<1>();
			position->operator [](id) = VEC3(Scalar(p[0]), Scalar(p[1]), Scalar(p[2]));
			vertices_indices.push_back(id);
			p += 3 ;
		}

		//create tetrahedrons
		int* t = volume_->tetrahedronlist ;
		for(uint32 i = 0u; i < this->nb_volumes_; ++i)
		{
			this->volumes_nb_vertices_.push_back(4u);
			for(uint32 j = 0u; j < 3u; j++)
				this->volumes_vertex_indices_.push_back(vertices_indices[t[j] - volume_->firstnumber]);

			t += 4 ;
		}

		return true;
	}
private:
	tetgenio* volume_;
};

template <typename VEC3, typename MAP_TRAITS>
std::unique_ptr<tetgenio> export_tetgen(CMap2<MAP_TRAITS>& map, const typename CMap2<MAP_TRAITS>::template VertexAttributeHandler<VEC3>& pos)
{
	using Map = CMap2<MAP_TRAITS>;
	using Vertex = typename Map::Vertex;
	using Face = typename Map::Face;

	using TetgenReal = REAL;
	std::unique_ptr<tetgenio> output = make_unique<tetgenio>();

	// 0-based indexing
	output->firstnumber = 0;

	// input vertices
	output->numberofpoints = map.template nb_cells<Vertex::ORBIT>();
	output->pointlist = new TetgenReal[output->numberofpoints * 3];

	//for each vertex
	uint32 i = 0u;
	map.foreach_cell([&output,&i,&pos](Vertex v)
	{
		const VEC3& vec = pos[v];
		output->pointlist[i++] = vec[0];
		output->pointlist[i++] = vec[1];
		output->pointlist[i++] = vec[2];
	});

	output->numberoffacets = map.template nb_cells<Face::ORBIT>();
	output->facetlist = new tetgenio::facet[output->numberoffacets] ;

	//for each facet
	i = 0u;
	map.foreach_cell([&output,&i,&map](Face face)
	{
		tetgenio::facet* f = &(output->facetlist[i]);
		tetgenio::init(f);
		f->numberofpolygons = 1;
		f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
		tetgenio::polygon* p = f->polygonlist;
		tetgenio::init(p);
		p->numberofvertices = map.codegree(face);
		p->vertexlist = new int[p->numberofvertices];

		uint32 j = 0u;
		map.foreach_incident_vertex(face, [&p,&map,&j](Vertex v)
		{
			p->vertexlist[j++] = map.get_embedding(v);
		});

		f->numberofholes = 0;
		f->holelist = nullptr;
		++i;
	});

	return output;
}

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_TETGEN_IO_CPP))
extern template class CGOGN_IO_API TetgenVolumeImport<DefaultMapTraits, Eigen::Vector3d>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_TETGEN_IO_CPP))

} // namespace io
} // namespace cgogn

#endif // IO_TETGEN_IO_H

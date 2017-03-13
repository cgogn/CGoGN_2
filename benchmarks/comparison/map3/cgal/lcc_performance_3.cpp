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

#include "lcc_performance_3.h"

void Performance3_LCC::clear_mesh()
{
	lcc.reset(new LCC_3());
}

bool Performance3_LCC::read_mesh(const std::string& filename)
{
	// variables
	size_t i, num_vertices, num_tetras;
	std::vector<Point> points;
	std::string line;
	std::stringstream sStream;

	// try to open the file
	std::ifstream fin(filename);
	if(!fin)
		return false;

	// load number of vertices

	// skip Vertices label
	std::getline(fin, line);

	std::getline(fin, line); // num_vertices x
	sStream << line;
	sStream >> num_vertices;
	sStream.clear();

	// reverse the order of the vertices
	points.reserve(num_vertices);

	for(i = 0; i < num_vertices; ++i)
	{
		float x, y, z;
		std::getline(fin, line);
		sStream << line;
		sStream >> x >> y >> z;
		std::getline(sStream, line);
		sStream.clear();

		points.push_back(LCC_3::Point(x,y,z));
	}

	// skip Tetrahedra label
	std::getline(fin, line);

	// read number of tetraeders
	std::getline(fin, line); // num_tetras x
	sStream << line;
	sStream >> num_tetras;
	sStream.clear();


	for(i = 0; i < num_tetras; ++i)
	{
		int v0,v1,v2,v3;
		//int m;
		std::getline(fin, line);
		sStream << line;
		sStream >> v0 >> v1 >> v2 >> v3;
		std::getline(sStream, line);
		sStream.clear();

		Dart_handle dart=
				lcc->make_tetrahedron(points[v0-1], points[v1-1], points[v2-1], points[v3-1]);
		lcc->set_attribute<3>(dart, lcc->create_attribute<3>());
	}

	lcc->sew3_same_facets();

	update_accelerators();

	return lcc->is_valid() && lcc->number_of_darts() > 0;;
}

void Performance3_LCC::update_accelerators()
{
	// We use accelerators, similarly than in cgogn. the only difference is that in cgogn
	// they are use internally (i.e inside the library) while here they are use externally
	// (i.e. inside the benchmarks).
	// Note that in the current bench, they use 4 accelerators:
	// 1) indicent VERTEX, VOLUME; 2) incident VOLUME, VERTEX;
	// 3) adjacent VERTEX, EDGE; 4) adjacent VERTEX, VOLUME

	// Here we only use  adjacent VERTEX, EDGE and indicent VERTEX, VOLUME.

	// First we update all adjacent vertices
	adjacent_vertices.clear();
	for (LCC_3::Vertex_attribute_range::iterator vit = lcc->vertex_attributes().begin(),
		 vend = lcc->vertex_attributes().end(); vit != vend; ++vit)
	{
		std::vector<Vertex_handle> adj;
		for(LCC_3::One_dart_per_incident_cell_range<1,0>::iterator
			it = lcc->one_dart_per_incident_cell<1,0>(vit->dart()).begin(),
			itend = lcc->one_dart_per_incident_cell<1,0>(vit->dart()).end();
			it != itend; ++it)
		{
			if ( lcc->vertex_attribute(it)==vit )
			{
				adj.push_back(lcc->vertex_attribute(lcc->other_extremity(it)));
			}
			else
			{
				adj.push_back(lcc->vertex_attribute(it));
			}
		}

		adjacent_vertices[vit]=adj;
	}
}

Dart_handle Performance3_LCC::create_face(Dart_handle old1, Vertex_handle v1, Vertex_handle v2, Vertex_handle v3, Volume_handle vol)
{
	Dart_handle old2 = lcc->beta<2>(old1);

	Dart_handle d1=lcc->create_dart(v1);
	Dart_handle d2=lcc->create_dart(v2);
	Dart_handle d3=lcc->create_dart(v3);
	lcc->set_dart_attribute<3>(d1,vol);
	lcc->set_dart_attribute<3>(d2,vol);
	lcc->set_dart_attribute<3>(d3,vol);

	lcc->basic_link_beta_1(d1,d2);
	lcc->basic_link_beta_1(d2,d3);
	lcc->basic_link_beta_1(d3,d1);

	Dart_handle dd1=lcc->create_dart(v2);
	Dart_handle dd2=lcc->create_dart(v1);
	Dart_handle dd3=lcc->create_dart(v3);
	lcc->set_dart_attribute<3>(dd1,vol);
	lcc->set_dart_attribute<3>(dd2,vol);
	lcc->set_dart_attribute<3>(dd3,vol);

	lcc->basic_link_beta_1(dd1,dd2);
	lcc->basic_link_beta_1(dd2,dd3);
	lcc->basic_link_beta_1(dd3,dd1);

	lcc->basic_link_beta_for_involution<3>(d1,dd1);
	lcc->basic_link_beta_for_involution<3>(d2,dd3);
	lcc->basic_link_beta_for_involution<3>(d3,dd2);

	lcc->basic_link_beta_for_involution<2>(old1,d1);
	lcc->basic_link_beta_for_involution<2>(old2,dd1);

	return lcc->beta<1>(d1);
}

void Performance3_LCC::collapse(Dart_handle dart)
{
	std::vector<Dart_handle> tet_ring;

	// Get ring of tets
	// incident to the current edge

	Dart_handle iter = dart;
	do
	{
		tet_ring.push_back(iter);

		// Go over to adjacent tet in tet-ring
		iter = lcc->beta<2,3>(iter);
	}
	while(iter != dart);

	// Get top vertex
	Dart_handle topVertex = dart;
	Dart_handle botVertex = lcc->beta<1>(dart);

	// Get barycenter of current edge
	Point bc = lcc->barycenter<1>(dart);

	// Set geometry of top and bottom vertex to barycenter of former edge
	lcc->point(topVertex) = bc;
	lcc->point(botVertex) = bc;

	// 0=not known; 1=boundary; 2=no boundary
	if ( lcc->info<0>(topVertex).m==1 )
		lcc->info<0>(botVertex).m = 1;
	else if ( lcc->info<0>(topVertex).m==0 )
		lcc->info<0>(botVertex).m = 0;

	if ( lcc->info<0>(botVertex).m==1 )
		lcc->info<0>(topVertex).m = 1;
	else if ( lcc->info<0>(botVertex).m==0 )
		lcc->info<0>(topVertex).m = 0;

	// Delete tets in ring
	for(std::vector<Dart_handle>::const_iterator cr_it = tet_ring.begin(),
		cr_end = tet_ring.end();
		cr_it != cr_end;
		++cr_it)
	{
		Dart_handle top = lcc->beta<1,2,3>(*cr_it);
		Dart_handle bottom = lcc->beta<0,2,3>(*cr_it);
		CGAL::remove_cell<LCC_3, 3>(*lcc, *cr_it);
		lcc->sew<3>(top,bottom);
	}
}

Dart_handle Performance3_LCC::tet_split(Dart_handle d1)
{
	Point_3 c = LCC_3::Traits::Construct_translated_point()
			(CGAL::ORIGIN, barycenter_3(d1));

	Dart_handle d2=lcc->beta<1,2>(d1);
	Dart_handle d3=lcc->beta<2>(d1);
	Dart_handle d4=lcc->beta<1,2>(d3);

	Vertex_handle v0 = lcc->create_vertex_attribute(c);
	Vertex_handle v1 = lcc->vertex_attribute(d1);
	Vertex_handle v2 = lcc->vertex_attribute(d2);
	Vertex_handle v3 = lcc->vertex_attribute(d3);
	Vertex_handle v4 = lcc->vertex_attribute(d4);

	Dart_handle n1 = create_face(d1, v3, v1, v0, lcc->attribute<3>(d1));
	Dart_handle n2 = create_face(d2, v3, v2, v0, lcc->attribute<3>(d1));
	Dart_handle n3 = create_face(lcc->beta<0>(d1), v1, v2, v0, lcc->attribute<3>(d1));
	Dart_handle n4 = create_face(lcc->beta<0>(d2), v4, v2, v0, lcc->attribute<3>(d1));
	Dart_handle n5 = create_face(d4, v1, v4, v0, lcc->attribute<3>(d1));
	Dart_handle n6 = create_face(lcc->beta<1>(d2), v4, v3, v0, lcc->attribute<3>(d1));

	lcc->basic_link_beta_for_involution<2>(n1,           lcc->beta<1>(n3));
	lcc->basic_link_beta_for_involution<2>(lcc->beta<1>(n1),lcc->beta<1,3>(n2));

	lcc->basic_link_beta_for_involution<2>(lcc->beta<3>(n1),lcc->beta<1,3>(n5));
	lcc->basic_link_beta_for_involution<2>(lcc->beta<1,3>(n1),lcc->beta<3>(n6));

	lcc->basic_link_beta_for_involution<2>(n2, lcc->beta<1>(n4));
	lcc->basic_link_beta_for_involution<2>(lcc->beta<1>(n2), n6);
	lcc->basic_link_beta_for_involution<2>(lcc->beta<3>(n2), n3);

	lcc->basic_link_beta_for_involution<2>(lcc->beta<3>(n3), lcc->beta<1,3>(n4));
	lcc->basic_link_beta_for_involution<2>(lcc->beta<1,3>(n3), lcc->beta<1>(n5));

	lcc->basic_link_beta_for_involution<2>(lcc->beta<3>(n4), n5);
	lcc->basic_link_beta_for_involution<2>(n4, lcc->beta<1>(n6));

	lcc->basic_link_beta_for_involution<2>(lcc->beta<3>(n5), lcc->beta<1,3>(n6));

	lcc->set_attribute<3>(d2, lcc->create_attribute<3>());
	lcc->set_attribute<3>(d3, lcc->create_attribute<3>());
	lcc->set_attribute<3>(d4, lcc->create_attribute<3>());

	return d1;
}

bool Performance3_LCC::is_boundary(const Dart_handle& _dart)
{
	// 0=not known; 1=boundary; 2=no boundary
	if ( lcc->info<0>(_dart).m==1 ||
		 lcc->info<0>(lcc->beta<1>(_dart)).m==1 ) return true;

	// Both endpoints have to lie in the interior of the mesh
	if ( lcc->info<0>(_dart).m==0 )
	{
		for(LCC_3::Dart_of_cell_range<0>::iterator
			vv_it = lcc->darts_of_cell<0>(_dart).begin(),
			vv_itend = lcc->darts_of_cell<0>(_dart).end();
			vv_it != vv_itend; ++vv_it)
		{
			if(lcc->is_free<3>(vv_it))
			{
				lcc->info<0>(_dart).m = 1;
				return true;
			}
		}
		lcc->info<0>(_dart).m = 2;
	}

	if ( lcc->info<0>(lcc->beta<1>(_dart)).m==0 )
	{
		for(LCC_3::Dart_of_cell_range<0>::iterator
			vv_it = lcc->darts_of_cell<0>(lcc->beta<1>(_dart)).begin(),
			vv_itend = lcc->darts_of_cell<0>(lcc->beta<1>(_dart)).end();
			vv_it != vv_itend; ++vv_it)
		{
			if (lcc->is_free<3>(vv_it))
			{
				lcc->info<0>(vv_it).m = 1;
				return true;
			}
		}
		lcc->info<0>(lcc->beta<1>(_dart)).m = 2;
	}

	return false;
}

Dart_handle Performance3_LCC::getShortestEdge()
{
	double weight = std::numeric_limits<double>::max();
	Dart_handle dart = lcc->null_dart_handle;

	int m=lcc->get_new_mark();
	for (typename LCC_3::Dart_range::iterator e_it=lcc->darts().begin(), e_end=lcc->darts().end();
		 e_it != e_end; ++e_it)
	{
		if ( !lcc->is_marked(e_it, m) )
		{
			CGAL::mark_cell<LCC_3, 1>(*lcc, e_it, m);
			if( !is_boundary(e_it) )
			{
				Point p1 = lcc->point(e_it);
				Point p0 = lcc->point(lcc->other_extremity(e_it));
				Vector v = (p1 - p0);

				double w = sqrt(v.squared_length());
				if(w < weight)
				{
					weight = w;
					dart = e_it;
				}
			}
		}
	}
	lcc->free_mark(m);
	return dart;
}

Vector_3 Performance3_LCC::barycenter_3(Dart_handle d)
{
	int m = lcc->get_new_mark();

	Vector_3 v  = CGAL::NULL_VECTOR;
	int count=0;
	for (LCC_3::Dart_of_cell_basic_range<3>::iterator
		 vv_it = lcc->darts_of_cell_basic<3>(d, m).begin(),
		 vv_itend = lcc->darts_of_cell_basic<3>(d, m).end();
		 vv_it != vv_itend; ++vv_it)
	{
		if (!lcc->info<0>(vv_it).m)
		{
			lcc->info<0>(vv_it).m = 1;
			v = v + (lcc->point(vv_it)- CGAL::ORIGIN);
			++count;
		}
	}
	// Unmark
	lcc->negate_mark(m);
	for (LCC_3::Dart_of_cell_basic_range<3>::iterator
		 vv_it = lcc->darts_of_cell_basic<3>(d, m).begin(),
		 vv_itend = lcc->darts_of_cell_basic<3>(d, m).end();
		 vv_it != vv_itend; ++vv_it)
	{
		if (lcc->info<0>(vv_it).m)
		{
			lcc->info<0>(vv_it).m = 0;
		}
	}

	lcc->free_mark(m);
	return v/count;
}


BENCHMARK_F(Performance3_LCC, circulator)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		int counter = 0;

		int m = lcc->get_new_mark();

		//for each vertex enumerate its incident volumes
		for (LCC_3::Vertex_attribute_range::iterator vit = lcc->vertex_attributes().begin(),
			 vend = lcc->vertex_attributes().end(); vit != vend; ++vit)
		{
			for (LCC_3::Dart_of_cell_basic_range<0>::iterator
				 vv_it = lcc->darts_of_cell_basic<0>(vit->dart(), m).begin(),
				 vv_itend = lcc->darts_of_cell_basic<0>(vit->dart(), m).end();
				 vv_it != vv_itend; ++vv_it)
			{
				lcc->mark(vv_it, m);
				if (!lcc->info<3>(vv_it).m)
				{
					lcc->info<3>(vv_it).m = 1;
					++counter;
				}
			}
			lcc->negate_mark(m);
			for (LCC_3::Dart_of_cell_basic_range<0>::iterator
				 vv_it = lcc->darts_of_cell_basic<0>(vit->dart(), m).begin(),
				 vv_itend = lcc->darts_of_cell_basic<0>(vit->dart(), m).end();
				 vv_it != vv_itend; ++vv_it)
			{
				lcc->mark(vv_it, m);
				if (lcc->info<3>(vv_it).m)
				{
					lcc->info<3>(vv_it).m = 0;
				}
			}
			lcc->negate_mark(m);
		}

		assert( lcc->is_whole_map_unmarked(m) );

		//for each volume enumerate its incident vertices
		for (LCC_3::Attribute_range<3>::type::iterator vit = lcc->attributes<3>().begin(),
			 vend = lcc->attributes<3>().end(); vit != vend; ++vit)
		{
			for (LCC_3::Dart_of_cell_basic_range<3>::iterator
				 vv_it = lcc->darts_of_cell_basic<3>(vit->dart(), m).begin(),
				 vv_itend = lcc->darts_of_cell_basic<3>(vit->dart(), m).end();
				 vv_it != vv_itend; ++vv_it)
			{
				if (!lcc->info<0>(vv_it).m)
				{
					lcc->info<0>(vv_it).m = 1;
					--counter;
				}
			}
			lcc->negate_mark(m);
			for (LCC_3::Dart_of_cell_basic_range<3>::iterator
				 vv_it = lcc->darts_of_cell_basic<3>(vit->dart(), m).begin(),
				 vv_itend = lcc->darts_of_cell_basic<3>(vit->dart(), m).end();
				 vv_it != vv_itend; ++vv_it)
			{
				if (lcc->info<0>(vv_it).m)
				{
					lcc->info<0>(vv_it).m = 0;
				}
			}
			lcc->negate_mark(m);
		}

		assert( lcc->is_whole_map_unmarked(m) );
		lcc->free_mark(m);
	}
}

BENCHMARK_F(Performance3_LCC, circulator2)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		int counter = 0;

		//for each vertex enumerate its vertices adjacent through a common volume.
		int m = lcc->get_new_mark();
		int m2 = lcc->get_new_mark();

		for (LCC_3::Vertex_attribute_range::iterator vit = lcc->vertex_attributes().begin(),
			 vend = lcc->vertex_attributes().end(); vit != vend; ++vit)
		{
			vit->info().m = 1;
			for (LCC_3::Dart_of_cell_basic_range<0>::iterator
				 v_it = lcc->darts_of_cell_basic<0>(vit->dart(), m).begin(),
				 v_itend = lcc->darts_of_cell_basic<0>(vit->dart(), m).end();
				 v_it != v_itend; ++v_it)
			{
				if ( !lcc->is_marked(v_it, m2) )
				{
					for (LCC_3::Dart_of_cell_basic_range<3>::iterator
						 vv_it = lcc->darts_of_cell_basic<3>(v_it, m2).begin(),
						 vv_itend = lcc->darts_of_cell_basic<3>(v_it, m2).end();
						 vv_it != vv_itend; ++vv_it)
					{
						lcc->mark(vv_it, m2);

						if (!lcc->info<0>(vv_it).m)
						{
							lcc->info<0>(vv_it).m = 1;
							++counter;
						}
					}
				}
			}

			// Here we unmark things
			lcc->negate_mark(m);
			lcc->negate_mark(m2);
			vit->info().m = 0;
			for (LCC_3::Dart_of_cell_basic_range<0>::iterator
				 v_it = lcc->darts_of_cell_basic<0>(vit->dart(), m).begin(),
				 v_itend = lcc->darts_of_cell_basic<0>(vit->dart(), m).end();
				 v_it != v_itend; ++v_it)
			{
				if ( !lcc->is_marked(v_it, m2) )
				{
					for (LCC_3::Dart_of_cell_basic_range<3>::iterator
						 vv_it = lcc->darts_of_cell_basic<3>(v_it, m2).begin(),
						 vv_itend = lcc->darts_of_cell_basic<3>(v_it, m2).end();
						 vv_it != vv_itend; ++vv_it)
					{
						lcc->mark(vv_it, m2);

						if (lcc->info<0>(vv_it).m)
						{
							lcc->info<0>(vv_it).m = 0;
						}
					}
				}
			}
			lcc->negate_mark(m2);
			lcc->negate_mark(m);
		}

		assert( lcc->is_whole_map_unmarked(m) );
		assert( lcc->is_whole_map_unmarked(m2) );
		lcc->free_mark(m);
		lcc->free_mark(m2);
	}
}

BENCHMARK_F(Performance3_LCC, barycenter)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		Vector_3 s(CGAL::NULL_VECTOR);

		for (LCC_3::Attribute_range<3>::type::iterator vit = lcc->attributes<3>().begin(),
			 vend = lcc->attributes<3>().end(); vit != vend; ++vit)
		{
			s = s + barycenter_3(vit->dart());
		}
	}
}

BENCHMARK_F(Performance3_LCC, smoothing)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		for (LCC_3::Vertex_attribute_range::iterator vit = lcc->vertex_attributes().begin(),
			 vend = lcc->vertex_attributes().end(); vit != vend; ++vit )
		{
			Vector_3 v(0.0, 0.0, 0.0);
			int valence = 0;

			for (std::vector<Vertex_handle>::iterator it= adjacent_vertices[vit].begin(),
				 itend=adjacent_vertices[vit].end(); it!=itend; ++it )
			{
				v = v + (lcc->point_of_vertex_attribute(*it) - CGAL::ORIGIN);
				++valence;
			}

			v = v / double(valence);

			vit->info().v = v;
		}

		// Update all vertices's positions
		for (LCC_3::Vertex_attribute_range::iterator vit = lcc->vertex_attributes().begin(),
			 vend = lcc->vertex_attributes().end(); vit != vend; ++vit)
		{
			lcc->point_of_vertex_attribute(vit) = CGAL::ORIGIN + lcc->info_of_attribute<0>(vit).v;
		}
	}
}

BENCHMARK_F(Performance3_LCC, split_tet)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		state.ResumeTiming();

		LCC_3::Attribute_range<3>::type::iterator vit = lcc->attributes<3>().begin(),
				vend = lcc->attributes<3>().end();
		--vend;
		while ( vit!=vend )
		{
			Dart_handle d = vit->dart();
			++vit;
			tet_split(d);
		}
		tet_split(vend->dart());
	}
}

BENCHMARK_F(Performance3_LCC, collapse)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		state.ResumeTiming();

		for(unsigned int i = 0; i < this->collapse_nb_it_; ++i)
		{
			Dart_handle dart = getShortestEdge();

			if(dart == lcc->null_dart_handle)
			{
				std::cerr << "No valid edge anymore, aborting at step "<<i << std::endl;
				return;
			}

			// 0=not known; 1=boundary; 2=no boundary
			lcc->info<0>(dart).m = 0;
			lcc->info<0>(lcc->other_extremity(dart)).m = 0;
#if CGAL_VERSION_NR >= CGAL_VERSION_NUMBER(4,9,0)
			CGAL::contract_cell<LCC_3,1>(*lcc, dart, true);
#else
			CGAL::contract_cell<LCC_3,1>(*lcc, dart);
#endif
		}

		for (LCC_3::Vertex_attribute_range::iterator vit = lcc->vertex_attributes().begin(),
			 vend = lcc->vertex_attributes().end(); vit != vend; ++vit)
		{
			vit->info().m = 0;
		}
	}
}



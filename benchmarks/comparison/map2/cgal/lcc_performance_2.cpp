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

#include "lcc_performance_2.h"

void Performance2_LCC::clear_mesh()
{
	lcc.reset(new LCC());
}

bool Performance2_LCC::read_mesh(const std::string& filename)
{
	std::ifstream ifs(filename);
	if (!ifs.good())
		return false;
	CGAL::load_off(*lcc, ifs);

	for ( LCC::Dart_range::iterator dit=lcc->darts().begin(),
		  dend=lcc->darts().end(); dit!=dend; ++dit )
	{
		if ( lcc->attribute<2>(dit)==NULL )
			lcc->set_attribute<2>(dit, lcc->create_attribute<2>());
	}
	return lcc->is_valid() && lcc->number_of_darts() > 0;
}


void Performance2_LCC::flip_edge(LCC::Dart_handle d)
{
	LCC::Dart_handle d1 = lcc->beta<1>(d);
	LCC::Dart_handle d2 = lcc->beta<2, 1>(d);

	CGAL_assertion ( !lcc->is_free<1>(d1) && !lcc->is_free<1>(d2) );

	LCC::Dart_handle d3 = lcc->beta<1>(d1);
	LCC::Dart_handle d4 = lcc->beta<1>(d2);

	// We isolated the edge
	lcc->link_beta_1(lcc->beta<0>(d), d2);
	lcc->link_beta_1(lcc->beta<2,0>(d), d1);

	// Then we push the two extremities.
	lcc->basic_link_beta_0(d3, d);
	lcc->basic_link_beta_1(d1, lcc->beta<2>(d));
	lcc->basic_link_beta_1(d2, d);
	lcc->basic_link_beta_0(d4, lcc->beta<2>(d));

	// And we update the vertex attribute
	lcc->set_vertex_attribute_of_dart(d, lcc->vertex_attribute(d4));
	lcc->set_vertex_attribute_of_dart(lcc->beta<2>(d), lcc->vertex_attribute(d3));
}

void Performance2_LCC::contract_face(LCC::Dart_handle dh)
{
	CGAL_assertion( dh!=lcc->null_dart_handle );
	LCC::Dart_handle d1=lcc->beta<2>(dh);
	LCC::Dart_handle d2=lcc->beta<1,2>(dh);
	CGAL_assertion(d1!=lcc->null_dart_handle &&
			d2!=lcc->null_dart_handle);

	lcc->basic_link_beta<2>(d1, d2);
	lcc->set_dart_of_attribute<0>(lcc->vertex_attribute(d1), d1);
	lcc->set_dart_of_attribute<0>(lcc->vertex_attribute(d2), d2);

	lcc->erase_dart(lcc->beta<1>(dh));
	lcc->erase_dart(dh);
}

void Performance2_LCC::collapse_edge(LCC::Dart_handle dh)
{
	CGAL_assertion( dh!=lcc->null_dart_handle );
	CGAL_assertion(!lcc->is_free<2>(dh));

	LCC::Dart_handle h1=lcc->beta<0>(dh);
	LCC::Dart_handle o1=lcc->beta<2,1>(dh);

	lcc->set_attribute<0>(dh, lcc->attribute<0>(lcc->beta<2>(dh)));

	lcc->basic_link_beta_1(lcc->beta<2,0>(dh),lcc->beta<2,1>(dh));
	lcc->basic_link_beta_1(lcc->beta<0>(dh),lcc->beta<1>(dh));

	lcc->erase_dart(lcc->beta<2>(dh));
	lcc->erase_dart(dh);

	if (lcc->beta<1,1>(h1)==h1)
	{
		contract_face(h1);
	}

	if (lcc->beta<1,1>(o1)==o1)
	{
		contract_face(o1);
	}
}

BENCHMARK_F(Performance2_LCC, circulator)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		LCC::Vertex_attribute_range::iterator vit, vend = lcc->vertex_attributes().end();
		LCC::Attribute_range<2>::type::iterator fit, fend = lcc->attributes<2>().end();
		int counter = 0;

		for (vit = lcc->vertex_attributes().begin(); vit != vend; ++vit)
		{
			for( LCC::Dart_of_cell_range<0>::iterator
				 vhit  = lcc->darts_of_cell<0>(vit->dart()).begin(),
				 vhend = lcc->darts_of_cell<0>(vit->dart()).end();
				 vhit!=vhend; ++vhit )
			{
				++counter;
			}
		}

		for (fit = lcc->attributes<2>().begin(); fit != fend; ++fit)
		{
			for( LCC::Dart_of_cell_range<2>::iterator
				 fhit  = lcc->darts_of_cell<2>(fit->dart()).begin(),
				 fhend = lcc->darts_of_cell<2>(fit->dart()).end();
				 fhit!=fhend; ++fhit )
			{
				--counter;
			}
		}
	}
}

BENCHMARK_F(Performance2_LCC, barycenter)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		LCC::Vertex_attribute_range::iterator vit, vend = lcc->vertex_attributes().end();
		LCCVector_3 v(CGAL::NULL_VECTOR);
		for (vit = lcc->vertex_attributes().begin(); vit != vend; ++vit)
		{
			v = v + (vit->point() - CGAL::ORIGIN);
		}
		v = v / lcc->number_of_vertex_attributes();
		for (vit = lcc->vertex_attributes().begin(); vit != vend; ++vit)
		{
			vit->point() = vit->point() - v;
		}
	}
}

BENCHMARK_F(Performance2_LCC, normal)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		LCC::Vertex_attribute_range::iterator vit, vend = lcc->vertex_attributes().end();
		LCC::Attribute_range<2>::type::iterator fit, fend = lcc->attributes<2>().end();

		for (fit = lcc->attributes<2>().begin(); fit != fend; ++fit)
		{
			LCC::Dart_handle dh = fit->dart();
			LCCPoint_3& p0 = lcc->point(dh);
			dh = lcc->beta<1>(dh);
			LCCPoint_3& p1 = lcc->point(dh);
			dh = lcc->beta<1>(dh);
			LCCPoint_3& p2 = lcc->point(dh);
			LCCVector_3 n = cross_product(p0-p1, p2-p1);
			n = n / sqrt(n.squared_length());
			fit->info() = n;
		}

		for (vit = lcc->vertex_attributes().begin(); vit != vend; ++vit)
		{
			LCCVector_3 n(0,0,0);
			for( LCC::Dart_of_cell_range<0>::iterator
				 vhit  = lcc->darts_of_cell<0>(vit->dart()).begin(),
				 vhend = lcc->darts_of_cell<0>(vit->dart()).end();
				 vhit!=vhend; ++vhit )
			{
				n = n + lcc->info<2>(vhit);
			}
			n = n / sqrt(n.squared_length());
			vit->info() = n;
		}
	}
}


BENCHMARK_F(Performance2_LCC, smoothing)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		LCC::Vertex_attribute_range::iterator vit, vend = lcc->vertex_attributes().end();

		for (vit = lcc->vertex_attributes().begin(); vit != vend; ++vit)
		{
			bool vertex_is_border = false;
			LCCVector_3 v(0,0,0);
			float c(0);
			for( LCC::Dart_of_cell_range<0>::iterator
				 vhit  = lcc->darts_of_cell<0>(vit->dart()).begin(),
				 vhend = lcc->darts_of_cell<0>(vit->dart()).end();
				 vhit!=vhend; ++vhit )
			{
				if (lcc->is_free<2>(vhit))
				{
					vertex_is_border = true;
					break;
				}

				v = v + (lcc->point(lcc->other_extremity(vhit)) - CGAL::ORIGIN);
				++c;
			}
			if (!vertex_is_border)
				vit->point() = CGAL::ORIGIN + (v / c);
		}
		assert( lcc->is_valid());
	}
}

BENCHMARK_F(Performance2_LCC, subdivision)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		state.ResumeTiming();
		int nv = lcc->number_of_vertex_attributes();

		// iterators
		LCC::Vertex_attribute_range::iterator vit, vend = lcc->vertex_attributes().end();
		LCC::Attribute_range<2>::type::iterator fit, fend = lcc->attributes<2>().end();
		LCC::Dart_range::iterator dit, dend =lcc->darts().end();

		// compute new positions of old vertices
		int i;
		std::vector<LCCPoint_3> new_pos(nv);
		for (vit = lcc->vertex_attributes().begin(), i=0; vit != vend; ++vit, ++i)
		{
			bool      vertex_is_border = false;
			LCCVector_3  v(0,0,0);
			float     n = 0;

			for( LCC::Dart_of_cell_range<0>::iterator
				 vhit  = lcc->darts_of_cell<0>(vit->dart()).begin(),
				 vhend = lcc->darts_of_cell<0>(vit->dart()).end();
				 vhit!=vhend; ++vhit )
			{
				if (lcc->is_free<2>(vhit))
				{
					vertex_is_border = true;
					break;
				}

				v = v + (lcc->point(lcc->other_extremity(vhit)) - CGAL::ORIGIN);
				++n;
			}
			if (!vertex_is_border)
			{
				float alpha = (4.0 - 2.0*cos(2.0*M_PI/n)) / 9.0;
				v = (1.0f-alpha)*(vit->point() - CGAL::ORIGIN) + alpha/n*v;
				new_pos[i] = CGAL::ORIGIN + v;
			}
			else
			{
				new_pos[i] = vit->point();
			}
		}

		// adjust end iterators
		--vend; --fend; --dend;

		// split faces
		fit = lcc->attributes<2>().begin();
		do
		{
			lcc->insert_barycenter_in_cell<2>(fit->dart());
		}
		while (fit++ != fend);

		// adjust end iterators
		++vend; ++fend; ++dend;

		// set new positions of old vertices
		for (vit = lcc->vertex_attributes().begin(), i=0; vit != vend; ++vit, ++i)
			vit->point() = new_pos[i];

		// flip old edges
		for (dit = lcc->darts().begin(); dit != dend; ++dit)
		{
			if (!lcc->is_free<2>(dit) && dit<lcc->beta<2>(dit) )
				flip_edge(dit);
		}

		assert( lcc->is_valid());
	}
}

BENCHMARK_F(Performance2_LCC, collapse)(benchmark::State& state)
{
	while (state.KeepRunning())
	{
		state.PauseTiming();
		this->SetUp(state);
		state.ResumeTiming();

		LCC::Attribute_range<0>::type::iterator vit, vend = lcc->attributes<0>().end();
		LCC::Attribute_range<2>::type::iterator fit, fend = lcc->attributes<2>().end();

		// adjust end iterators
		--vend; --fend;

		fit = lcc->attributes<2>().begin();
		do
		{
			lcc->insert_point_in_cell<2>(fit->dart(), CGAL::ORIGIN);
		}
		while (fit++ != fend);

		// adjust end iterators
		++vend;

		// collapse new vertices
		vit=vend; vend=lcc->attributes<0>().end();
		for (; vit!=vend; )
		{
			LCC::Dart_handle cur = vit->dart();
			++vit;
			collapse_edge(cur);
		}
		assert( lcc->is_valid());
	}
}

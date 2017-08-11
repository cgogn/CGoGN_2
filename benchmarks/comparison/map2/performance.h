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

#ifndef BENCHMARK_COMPARISON_PERFORMANCE2_H
#define BENCHMARK_COMPARISON_PERFORMANCE2_H

#include <benchmark/benchmark.h>
#include <string>
#include <iostream>
#include <array>
#include <sstream>

//extern std::string filename;

class OStreamBlocker final
{
public:
	inline OStreamBlocker()
	{
		sbufs[0] =  std::cout.rdbuf();
		sbufs[1] =  std::cerr.rdbuf();
		std::cout.rdbuf(buffers[0].rdbuf());
		std::cerr.rdbuf(buffers[1].rdbuf());
	}

	inline ~OStreamBlocker()
	{
		std::cout.rdbuf(sbufs[0]);
		std::cerr.rdbuf(sbufs[1]);
	}

private:
	std::array<std::stringstream, 2> buffers;
	std::array<std::streambuf*, 2> sbufs;
};

class Performance2 : public ::benchmark::Fixture
{
public:
	using State = ::benchmark::State;
	using Real = double;
	virtual void clear_mesh() = 0;
	virtual bool read_mesh(const std::string& filename) = 0;
	void SetUp(State& state) override;
};

#endif // BENCHMARK_COMPARISON_PERFORMANCE2_H

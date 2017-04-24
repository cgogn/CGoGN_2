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

#include <cgogn/core/utils/definitions.h>
#include <cgogn/core/utils/logger.h>
#include <performance.h>

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_TEST_MESHES_PATH)

std::string filename;

int main(int argc, char** argv)
{
	::benchmark::Initialize(&argc, argv);

	if (argc < 2)
	{
		cgogn_log_info("bench_comparison2") << "USAGE: " << argv[0] << " [filename]";
		filename = std::string(DEFAULT_MESH_PATH) + std::string("off/horse.off");
		cgogn_log_info("bench_comparison2") << "Using default mesh : \"" << filename << "\".";
	}
	else
		filename = std::string(argv[1]);
	::benchmark::RunSpecifiedBenchmarks();
	return 0;
}


void Performance2::SetUp(State& state)
{
	this->clear_mesh();
	bool read_mesh_ok = false;
	{
		OStreamBlocker blocker;
		read_mesh_ok = this->read_mesh(filename);
	}
	if (!read_mesh_ok)
		state.SkipWithError("Unable to read the provided mesh.");
}

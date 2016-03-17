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

#ifndef IO_PROGRAM_OPTIONS_H
#define IO_PROGRAM_OPTIONS_H

#include <string>
#include <map>

#include <boost/program_options.hpp>


namespace cgogn
{

namespace io
{

class ProgramOptions {
private:
	bool optionsHandler();
public:
	explicit ProgramOptions(int argc, char **argv);
	ProgramOptions() = delete;
	ProgramOptions(const ProgramOptions & ) = delete;
	ProgramOptions(ProgramOptions && ) = delete;
	ProgramOptions& operator=(const ProgramOptions & ) = delete;
	ProgramOptions& operator=(ProgramOptions && ) = delete;
	~ProgramOptions();

	std::string input_file_;

	double facet_angle_;
	double facet_size_;
	double facet_distance_;
	double cell_radius_;
	double cell_size_;

	bool lloyd_;
	double lloyd_freeze_bound_;
	double lloyd_convergence_;
	std::size_t lloyd_max_iterations_;
	bool lloyd_freeze_;

	bool odt_;
	double odt_freeze_bound_ ;
	double odt_convergence_;
	std::size_t odt_max_iterations_;
	bool odt_freeze_;

	bool perturb_;
	double perturb_sliver_bound_;

	bool exude_;
	double exude_sliver_bound_;

private:
	boost::program_options::options_description desc_;
	boost::program_options::variables_map variable_map_;
};

} // namespace io
} // namespace cgogn

#endif // IO_PROGRAM_OPTIONS_H

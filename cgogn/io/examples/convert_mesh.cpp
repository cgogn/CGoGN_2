
#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/io/map_import.h>
#include <cgogn/io/map_export.h>

#include <cstring>
#include <sstream>
#include <string>
#include <iomanip>
#include <algorithm>
#include <cctype>

using namespace cgogn::numerics;

using Map2 = cgogn::CMap2;
using Map3 = cgogn::CMap3;

using Vec3 = Eigen::Vector3d;

const cgogn::Orbit vertex2 = Map2::Vertex::ORBIT;
const cgogn::Orbit vertex3 = Map3::Vertex::ORBIT;
const cgogn::Orbit face2   = Map2::Face::ORBIT;

bool string_to_bool (const std::string & v)
{
	return !v.empty() && (std::atoi(v.c_str()) != 0 || cgogn::i_equals(v, "true"));
}

int main(int argc, char** argv)
{
	std::string input_filename;
	std::string output_filename;
	bool is_surface = true;
	bool output_is_binary = false;
	bool compress_output = false;
	bool overwrite_output = false;

	if (argc < 4 || argc > 8)
	{
		cgogn_log_info("convert_mesh") << "USAGE: " << argv[0] << " [input_filename] [output_filename] [bool(is_surface)] [bool(binary_output)](optional, default 0) [bool(compress_output)](optional, default 0) [bool(overwrite_output)](optional, default 0)";
		return 0;
	}
	else
	{
		input_filename = std::string(argv[1]);
		output_filename = std::string(argv[2]);
		is_surface = string_to_bool(argv[3]);
		if (argc > 4)
			output_is_binary = string_to_bool(argv[4]);
		if (argc > 5)
			compress_output = string_to_bool(argv[5]);
		if (argc > 6)
			overwrite_output = string_to_bool(argv[6]);

		cgogn_log_info("convert_mesh") << "input mesh : " << input_filename;
		cgogn_log_info("convert_mesh") << "output mesh : " << output_filename;
		cgogn_log_info("convert_mesh") << "binary output : " << output_is_binary;
		cgogn_log_info("convert_mesh") << "compress output : " << compress_output;
	}
	auto export_options = cgogn::io::ExportOptions::create()
		.filename(output_filename)
		.binary(output_is_binary)
		.compress(compress_output)
		.overwrite(overwrite_output)
		.position_attribute(is_surface? vertex2:vertex3, "position");
	if (is_surface)
	{
		export_options.add_attribute(vertex2,"normal")
				.add_attribute(face2,"normal");
		Map2 map;
		cgogn::io::import_surface<Vec3>(map, input_filename);
		cgogn::io::export_surface(map, export_options);
	}
	else
	{
		Map3 map;
		cgogn::io::import_volume<Vec3>(map, input_filename);
		cgogn::io::export_volume(map, export_options);
	}

	return 0;
}

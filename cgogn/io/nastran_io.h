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

#ifndef CGOGN_IO_NASTRAN_IO_H_
#define CGOGN_IO_NASTRAN_IO_H_

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
class NastranIO
{
public:

	using Scalar = typename geometry::vector_traits<VEC3>::Scalar;

	inline Scalar parse_scalar(std::string& str)
	{
		Scalar x(0);
		const std::size_t pos1 = str.find_last_of('-');
		if ((pos1!=std::string::npos) && (pos1!=0))
		{
			std::string res = str.substr(0,pos1) + "e" + str.substr(pos1,8-pos1);
			x = Scalar(std::stod(res));
		}
		else
		{
			const std::size_t pos2 = str.find_last_of('+');
			if ((pos2!=std::string::npos) && (pos2!=0))
			{
				std::string res = str.substr(0,pos2) + "e" + str.substr(pos2,8-pos2);
				x = Scalar(std::stod(res));
			}
			else
			{
				x = Scalar(std::stod(str));
			}
		}
		return x;
	}
};

template <typename MAP, typename VEC3>
class NastranVolumeImport : public NastranIO<VEC3>, public VolumeFileImport<MAP, VEC3>
{
public:

	using Self = NastranVolumeImport<MAP, VEC3>;
	using Inherit_Nastran = NastranIO<VEC3>;
	using Inherit_Import = VolumeFileImport<MAP, VEC3>;
	template <typename T>
	using ChunkArray = typename Inherit_Import::template ChunkArray<T>;

	inline NastranVolumeImport(MAP& map) : Inherit_Import(map) {}
	CGOGN_NOT_COPYABLE_NOR_MOVABLE(NastranVolumeImport);
	virtual ~NastranVolumeImport() override {}

protected:

	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream file(filename, std::ios::in);
		ChunkArray<VEC3>* position = this->position_attribute();

		std::string line;
		line.reserve(512);
		std::string tag;
		tag.reserve(32);

		getline_safe (file, line);
		do
		{
			getline_safe (file, line);
			tag = line.substr(0,4);
		} while (tag !="GRID");

		// reading vertices
		std::map<uint32, uint32> old_new_ids_map;
		do
		{
			std::string s_v = line.substr(8,8);
			const uint32 old_index = std::stoi(s_v);
			const uint32 new_index = this->insert_line_vertex_container();
			old_new_ids_map[old_index] = new_index;
			auto& v = position->operator[](new_index);

			s_v = line.substr(24,8);
			v[0] = this->parse_scalar(s_v);
			s_v = line.substr(32,8);
			v[1] = this->parse_scalar(s_v);
			s_v = line.substr(40,8);
			v[2] = this->parse_scalar(s_v);

			getline_safe (file, line);
			tag = line.substr(0,4);
		} while (tag =="GRID");

		// reading volumes
		do
		{
			std::string s_v = line.substr(0, std::min(line.size(), std::size_t(12)));
			if (s_v[0] != '$')
			{
				if (s_v.compare(0, 5, "CHEXA") == 0)
				{
					std::array<uint32, 8> ids;

					s_v = line.substr(24,8);
					ids[0] = uint32(std::stoi(s_v));
					s_v = line.substr(32,8);
					ids[1] = uint32(std::stoi(s_v));
					s_v = line.substr(40,8);
					ids[2] = uint32(std::stoi(s_v));
					s_v = line.substr(48,8);
					ids[3] = uint32(std::stoi(s_v));
					s_v = line.substr(56,8);
					ids[4] = uint32(std::stoi(s_v));
					s_v = line.substr(64,8);
					ids[5] = uint32(std::stoi(s_v));

					getline_safe (file, line);
					s_v = line.substr(8,8);
					ids[6] = uint32(std::stoi(s_v));
					s_v = line.substr(16,8);
					ids[7] = uint32(std::stoi(s_v));

					for (uint32& id : ids)
						id = old_new_ids_map[id];

					this->add_hexa(ids[0], ids[1], ids[2], ids[3], ids[4], ids[5],ids[6], ids[7], true);
				}
				else
				{
					if (s_v.compare(0, 6,"CTETRA") == 0)
					{
						std::array<uint32, 4> ids;

						s_v = line.substr(24,8);
						ids[0] = uint32(std::stoi(s_v));
						s_v = line.substr(32,8);
						ids[1] = uint32(std::stoi(s_v));
						s_v = line.substr(40,8);
						ids[2] = uint32(std::stoi(s_v));
						s_v = line.substr(48,8);
						ids[3] = uint32(std::stoi(s_v));

						for (uint32& id : ids)
							id = old_new_ids_map[id];

						this->add_tetra(ids[0], ids[1], ids[2], ids[3], true);
					}
					else
					{
						if (s_v.compare(0, 7,"ENDDATA") == 0)
							break;
						cgogn_log_warning("NastranVolumeImport") << "Elements of type \"" << s_v << "\" are not supported. Ignoring.";
					}
				}
			}

			getline_safe (file, line);
			tag = line.substr(0,4);
		} while (!file.eof());

		return true;
	}
};

template <typename MAP>
class NastranVolumeExport : public VolumeExport<MAP>
{
public:

	using Inherit = VolumeExport<MAP>;
	using Self = NastranVolumeExport<MAP>;
	using Map = typename Inherit::Map;
	using Vertex = typename Inherit::Vertex;
	using Volume = typename Inherit::Volume;
	using ChunkArrayGen = typename Inherit::ChunkArrayGen;

protected:

	virtual void export_file_impl(const Map& map, std::ofstream& output, const ExportOptions& /*option*/) override
	{

		ChunkArrayGen const* pos = this->position_attribute(Vertex::ORBIT);
		std::string scalar_type = pos->nested_type_name();
		scalar_type[0] = std::toupper(scalar_type[0], std::locale());

		output << "$$ ---------------------------------------------------------------------------- $"<< std::endl;
		output << "$$      NASTRAN MEsh File Generated by CGoGN_2 (ICube/IGG)                      $"<< std::endl;
		output << "$$ ---------------------------------------------------------------------------- $"<< std::endl;
		output << "CEND" << std::endl;
		output << "BEGIN BULK" << std::endl;
		output << "$$ ---------------------------------------------------------------------------- $"<< std::endl;
		output << "$$      Vertices position                                                       $"<< std::endl;
		output << "$$ ---------------------------------------------------------------------------- $"<< std::endl;

		// 1. vertices
		uint32 count{1u};
		map.foreach_cell([&] (Vertex v)
		{
			output << "GRID    ";
			output << std::right;
			output.width(8);
			output << count++;
			output << "        ";
			output << std::left;
			std::stringstream position_stream;
			pos->export_element(map.embedding(v), position_stream, false, false);
			float32 tmp[3];
			position_stream >> tmp[0];
			position_stream >> tmp[1];
			position_stream >> tmp[2];
			output << std::setw(8) << trunc_float_to8(tmp[0]) << std::setw(8) << trunc_float_to8(tmp[1]) << std::setw(8) << trunc_float_to8(tmp[2]) << std::endl;
		}, *(this->cell_cache_));

		count = 1u;
		output << std::right;

		if (this->nb_hexas() > 0u)
		{
			output << "$$ ---------------------------------------------------------------------------- $"<< std::endl;
			output << "$$      Hexa indices                                                            $"<< std::endl;
			output << "$$ ---------------------------------------------------------------------------- $"<< std::endl;

			map.foreach_cell([&](Volume w)
			{
				const auto& vertices = this->vertices_of_volumes(w);
				if (vertices.size() == 8ul)
				{
					auto it = vertices.begin();
					output << "CHEXA   ";
					output << std::setw(8) << count++ << std::setw(8) << 0;
					output << std::setw(8) << (*it++ + 1);
					output << std::setw(8) << (*it++ + 1);
					output << std::setw(8) << (*it++ + 1);
					output << std::setw(8) << (*it++ + 1);
					output << std::setw(8) << (*it++ + 1);
					output << std::setw(8) << (*it++ + 1) << "+" << std::endl;
					output << "+       " << std::setw(8) << (*it++ + 1);
					output << std::setw(8) << (*it++ + 1) << std::endl;
				}
			}, *(this->cell_cache_));
		}

		if (this->nb_tetras() > 0u)
		{
			output << "$$ ---------------------------------------------------------------------------- $"<< std::endl;
			output << "$$      Tetra indices                                                           $"<< std::endl;
			output << "$$ ---------------------------------------------------------------------------- $"<< std::endl;


			map.foreach_cell([&](Volume w)
			{
				const auto& vertices = this->vertices_of_volumes(w);
				if (vertices.size() == 4ul)
				{
					auto it = vertices.begin();
					output << "CTETRA  ";
					output << std::setw(8) << count++ << std::setw(8) << 0;
					output << std::setw(8) << (*it++ + 1);
					output << std::setw(8) << (*it++ + 1);
					output << std::setw(8) << (*it++ + 1);
					output << std::setw(8) << (*it++ + 1) << std::endl;
				}
			}, *(this->cell_cache_));
		}
		output << "ENDDATA" << std::endl;
	}

private:

	static inline std::string trunc_float_to8(float32 f)
	{
		std::stringstream ss;
		ss << f;
		std::string res = ss.str();
		size_t expo = res.find('e');
		if (expo != std::string::npos)
		{
			if (res[expo+2] == '0')
				return res.substr(0, 6) + res[expo+1] + res[expo+3];

			return res.substr(0, 5) + res.substr(expo + 1);
		}
		return res.substr(0, 8);
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_NASTRAN_IO_CPP_))
extern template class CGOGN_IO_API NastranIO<Eigen::Vector3d>;
extern template class CGOGN_IO_API NastranIO<Eigen::Vector3f>;
extern template class CGOGN_IO_API NastranIO<geometry::Vec_T<std::array<float64, 3>>>;
extern template class CGOGN_IO_API NastranIO<geometry::Vec_T<std::array<float32, 3>>>;

extern template class CGOGN_IO_API NastranVolumeImport<CMap3, Eigen::Vector3d>;
extern template class CGOGN_IO_API NastranVolumeImport<CMap3, Eigen::Vector3f>;
extern template class CGOGN_IO_API NastranVolumeImport<CMap3, geometry::Vec_T<std::array<float64, 3>>>;
extern template class CGOGN_IO_API NastranVolumeImport<CMap3, geometry::Vec_T<std::array<float32, 3>>>;

extern template class CGOGN_IO_API NastranVolumeExport<CMap3>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(CGOGN_IO_NASTRAN_IO_CPP_))

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_NASTRAN_IO_H_

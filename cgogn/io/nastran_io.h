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

#include <cgogn/core/utils/logger.h>
#include <cgogn/io/dll.h>
#include <cgogn/io/data_io.h>
#include <cgogn/io/volume_import.h>

namespace cgogn
{

namespace io
{
template<typename VEC3>
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
		} else {
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

template<typename MAP_TRAITS, typename VEC3>
class NastranVolumeImport : public NastranIO<VEC3>, public VolumeImport<MAP_TRAITS>
{
	using Inherit_Nastran = NastranIO<VEC3>;
	using Inherit_Import = VolumeImport<MAP_TRAITS>;
	using Self = NastranVolumeImport<MAP_TRAITS, VEC3>;
	template <typename T>
	using ChunkArray = typename Inherit_Import::template ChunkArray<T>;

	// MeshImportGen interface
protected:
	virtual bool import_file_impl(const std::string& filename) override
	{
		std::ifstream file(filename, std::ios::in);
		ChunkArray<VEC3>* position = this->template get_position_attribute<VEC3>();

		std::string line;
		line.reserve(512);
		std::string tag;
		tag.reserve(32);

		std::getline (file, line);
		do
		{
			std::getline (file, line);
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
			auto& v = position->operator [](new_index);

			s_v = line.substr(24,8);
			v[0] = this->parse_scalar(s_v);
			s_v = line.substr(32,8);
			v[1] = this->parse_scalar(s_v);
			s_v = line.substr(40,8);
			v[2] = this->parse_scalar(s_v);

			std::getline (file, line);
			tag = line.substr(0,4);
			this->set_nb_vertices(this->get_nb_vertices() + 1u);
		} while (tag =="GRID");

		// reading volumes
		do
		{
			std::string s_v = line.substr(0,std::min(line.size(),std::size_t(12)));

			if (s_v.compare(0, 5,"CHEXA") == 0)
			{
				this->set_nb_volumes(this->get_nb_volumes() + 1u);
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

				std::getline (file, line);
				s_v = line.substr(8,8);
				ids[6] = uint32(std::stoi(s_v));
				s_v = line.substr(16,8);
				ids[7] = uint32(std::stoi(s_v));

				for (uint32& id : ids)
					id = old_new_ids_map[id];

				this->add_hexa(*position, ids[0], ids[1], ids[2], ids[3], ids[4], ids[5],ids[6], ids[7], true);
			} else {
				if (s_v.compare(0, 6,"CTETRA") == 0)
				{
					this->set_nb_volumes(this->get_nb_volumes() + 1u);
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

					this->add_tetra(*position, ids[0], ids[1], ids[2], ids[3], true);
				} else {
					if (s_v.compare(0, 7,"ENDDATA") == 0)
						break;
					cgogn_log_warning("NastranVolumeImport") << "Elements of type \"" << s_v << "\" are not supported. Ignoring.";
				}
			}

			std::getline (file, line);
			tag = line.substr(0,4);
		} while (!file.eof());


		return true;
	}
};

#if defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_NASTRAN_IO_CPP_))
extern template class CGOGN_IO_API NastranIO<Eigen::Vector3d>;
extern template class CGOGN_IO_API NastranIO<Eigen::Vector3f>;
extern template class CGOGN_IO_API NastranIO<geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API NastranIO<geometry::Vec_T<std::array<float32,3>>>;

extern template class CGOGN_IO_API NastranVolumeImport<DefaultMapTraits, Eigen::Vector3d>;
extern template class CGOGN_IO_API NastranVolumeImport<DefaultMapTraits, Eigen::Vector3f>;
extern template class CGOGN_IO_API NastranVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float64,3>>>;
extern template class CGOGN_IO_API NastranVolumeImport<DefaultMapTraits, geometry::Vec_T<std::array<float32,3>>>;
#endif // defined(CGOGN_USE_EXTERNAL_TEMPLATES) && (!defined(IO_NASTRAN_IO_CPP_))

} // namespace io
} // namespace cgogn
#endif // CGOGN_IO_NASTRAN_IO_H_

#ifndef MAP3_GEN_H
#define MAP3_GEN_H

#include <core/cmap/cmap3.h>
#include <core/utils/string.h>

#include <io/volume_import.h>
#include <io/mesh_generation/c3t3_io.h>

#include <CGAL/make_mesh_3.h>

namespace cgogn
{

namespace io
{

template <typename VEC3, class MAP_TRAITS>
inline void create_map3_from_image(CMap3<MAP_TRAITS>& map3, const std::string& image_path, const VolumeMeshFromImageCGALTraits::Criteria& criteria = VolumeMeshFromImageCGALTraits::Criteria())
{

	using Image			= VolumeMeshFromImageCGALTraits::Image;
	using Kernel		= VolumeMeshFromImageCGALTraits::Kernel;
	using Domain		= VolumeMeshFromImageCGALTraits::Domain;
	using Triangulation	= VolumeMeshFromImageCGALTraits::Triangulation;
	using Criteria		= VolumeMeshFromImageCGALTraits::Criteria;
	using C3T3			= VolumeMeshFromImageCGALTraits::C3T3;

	cgogn_assert(get_extension(image_path) == "inr");
	Image inrimage;
	inrimage.read(image_path.c_str());

	Domain domain(inrimage);

	C3T3 complex = CGAL::make_mesh_3<C3T3>(domain,
									 criteria,
									 CGAL::parameters::no_features(),
									 CGAL::parameters::no_odt(),
									 CGAL::parameters::no_lloyd(),
									 CGAL::parameters::no_perturb(),
									 CGAL::parameters::no_exude()
									);
	C3T3VolumeImport<C3T3, MAP_TRAITS, VEC3> volume_import(complex);
	volume_import.import_file("");
	volume_import.create_map(map3);
}

} // namespace io
} // namespace cgogn
#endif // MAP3_GEN_H

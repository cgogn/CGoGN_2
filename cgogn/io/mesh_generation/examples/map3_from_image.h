#ifndef IO_MAP3_FROM_IMAGE_H
#define IO_MAP3_FROM_IMAGE_H

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/utils/string.h>

#include <cgogn/io/volume_import.h>
#include <cgogn/io/mesh_generation/c3t3_io.h>

#include <CGAL/make_mesh_3.h>

namespace cgogn
{

namespace io
{

template <typename VEC3, class MAP_TRAITS>
inline void create_map3_from_image(CMap3<MAP_TRAITS>& map3,
								   const std::string& image_path,
								   const VolumeMeshFromImageCGALTraits::Criteria& criteria = VolumeMeshFromImageCGALTraits::Criteria(),
								   const CGAL::parameters::internal::Odt_options& odt= CGAL::parameters::odt(),
								   const CGAL::parameters::internal::Lloyd_options& lloyd= CGAL::parameters::lloyd(),
								   const CGAL::parameters::internal::Perturb_options& perturb= CGAL::parameters::perturb(),
								   const CGAL::parameters::internal::Exude_options& exude= CGAL::parameters::exude()
		)
{

	using Image			= VolumeMeshFromImageCGALTraits::Image;
	using Kernel		= VolumeMeshFromImageCGALTraits::Kernel;
	using Domain		= VolumeMeshFromImageCGALTraits::Domain;
	using Triangulation	= VolumeMeshFromImageCGALTraits::Triangulation;
	using Criteria		= VolumeMeshFromImageCGALTraits::Criteria;
	using C3T3			= VolumeMeshFromImageCGALTraits::C3T3;

	Image inrimage;
	inrimage.read(image_path.c_str());

	Domain domain(inrimage);

	C3T3 complex = CGAL::make_mesh_3<C3T3>(domain,
									 criteria,
									 CGAL::parameters::no_features(),
									 odt,
									 lloyd,
									 perturb,
									 exude
									);
	C3T3VolumeImport<C3T3, MAP_TRAITS, VEC3> volume_import(complex);
	volume_import.import_file("");
	volume_import.create_map(map3);
}

} // namespace io
} // namespace cgogn
#endif // IO_MAP3_FROM_IMAGE_H

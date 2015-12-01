#ifndef GEOMETRY_ALIGNED_BOX_H
#define GEOMETRY_ALIGNED_BOX_H

namespace cgogn {

template<typename SCALAR, int DIM>
using AlignedBox3D = Eigen::AlignedBox<SCALAR, DIM>;

// Utile ?
using AlignedBox3Df = AlignedBox3D<float, 3>;
using AlignedBox3Dd = AlignedBox3D<double, 3>;
using AlignedBox4Df = AlignedBox3D<float, 4>;
using AlignedBox4Dd = AlignedBox3D<double, 4>;


template<typename SCALAR, int DIM>
bool rayIntersect(const AlignedBox3D<SCALAR, DIM>& box,
				  const Eigen::Vector<SCALAR, DIM>& P,
				  const Eigen::Vector<SCALAR, DIM>& V,
				  const SCALAR epsilon) {

	typename Eigen::Vector<SCALAR, DIM>::Index maxIndex;
	SCALAR max = V.maxCoeff(&maxIndex);

//	if ()
	return false;
}

}

#endif // GEOMETRY_ALIGNED_BOX_H

#include <iostream>
#include <chrono>
#include <geometry/geometry.h>

#define ARRAY_SIZE 10000
#define ARRAY_LOOP 5000000

template <typename VECT>
void testLinear() {
	VECT tab[ARRAY_SIZE];
	for (int i=0; i<ARRAY_SIZE; ++i) tab[i].Random();

	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

	for (int i=0; i<ARRAY_LOOP; ++i)
		tab[(13*i+47)%ARRAY_SIZE] +=
				typename VECT::Scalar(5.7) * tab[(13*i+47)%ARRAY_SIZE];

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
			  << " microseconds." << std::endl;
}

template <typename SCALAR, int DIM>
void testLinear() {
	Eigen::Vector<SCALAR, DIM> tab[ARRAY_SIZE];
	for (int i=0; i<ARRAY_SIZE; ++i) tab[i].Random();

	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

	for (int i=0; i<ARRAY_LOOP; ++i)
		tab[(13*i+47)%ARRAY_SIZE] += SCALAR(5.7) * tab[(13*i+47)%ARRAY_SIZE];

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
			  << " microseconds." << std::endl;
}

template <typename SCALAR, int DIM>
void testProduct() {
	Eigen::Matrix<SCALAR, DIM, DIM> m = Eigen::Matrix<SCALAR, DIM, DIM>::Random();
	Eigen::Vector<SCALAR, DIM> tab[ARRAY_SIZE];
	for (int i=0; i<ARRAY_SIZE; ++i) tab[i].Random();

	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

	for (int i=0; i<ARRAY_LOOP; ++i)
		tab[(13*i+47)%ARRAY_SIZE] += SCALAR(5.7) * m * tab[(13*i+47)%ARRAY_SIZE];

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
			  << " microseconds." << std::endl;
}

template <typename SCALAR, int DIM>
void testBarycenter() {
	Eigen::Vector<SCALAR, DIM> tab[ARRAY_SIZE];
	for (int i=0; i<ARRAY_SIZE; ++i) tab[i].Random();

	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

	for (int i=0; i<ARRAY_LOOP; ++i)
		tab[(13*i+47)%ARRAY_SIZE] +=
				cgogn::barycenter(tab[(13*i+47)%ARRAY_SIZE],
				tab[(47*i+549)%ARRAY_SIZE],
				tab[(417*i+19)%ARRAY_SIZE],
				SCALAR(0.25),SCALAR(0.5),SCALAR(0.25));

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
			  << " microseconds." << std::endl;
}

template <typename SCALAR, int DIM>
void testTriangleNormal() {
	Eigen::Vector<SCALAR, DIM> tab[ARRAY_SIZE];
	for (int i=0; i<ARRAY_SIZE; ++i) tab[i].Random();

	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

	for (int i=0; i<ARRAY_LOOP; ++i)
		tab[(13*i+47)%ARRAY_SIZE] +=
				cgogn::triangleNormal(tab[(13*i+47)%ARRAY_SIZE],
				tab[(47*i+549)%ARRAY_SIZE],
				tab[(417*i+19)%ARRAY_SIZE]);

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
			  << " microseconds." << std::endl;
}

template <typename SCALAR, int DIM>
void testNorm3D() {
	Eigen::Vector<SCALAR, DIM> tab[ARRAY_SIZE];
	for (int i=0; i<ARRAY_SIZE; ++i) tab[i].Random();

	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

	SCALAR product = SCALAR(1);
	for (int i=0; i<ARRAY_LOOP; ++i)
		product *= Eigen::norm3D(tab[(13*i+47)%ARRAY_SIZE]);

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
			  << " microseconds." << std::endl;
}

template <typename SCALAR, int DIM>
void testAlignedBox() {
	Eigen::Vector<SCALAR, DIM> tab[ARRAY_SIZE];
	for (int i=0; i<ARRAY_SIZE; ++i) tab[i].Random();

	Eigen::AlignedBox<SCALAR, DIM> box[10];

	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

	for (int i=0; i<ARRAY_LOOP; ++i) {
		box[(13*i+47)%10].extend(tab[(47*i+549)%ARRAY_SIZE]);
		if (box[(417*i+19)%10].contains(box[(13*i+47)%10]))
			box[(417*i+19)%10].extend(tab[(417*i+19)%ARRAY_SIZE]);
	}

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
			  << " microseconds." << std::endl;
}

int main()
{
	std::cout << "Bench for linear combinations of Vectors :" << std::endl;
	std::cout << " - Eigen::Vector3f : ";
	testLinear<Eigen::Vector3f>();
	std::cout << " - Eigen::Vector3d : ";
	testLinear<Eigen::Vector3d>();
	std::cout << " - Eigen::Vector4f : ";
	testLinear<Eigen::Vector4f>();
	std::cout << " - Eigen::Vector4d : ";
	testLinear<Eigen::Vector4d>();
	std::cout << std::endl;

	std::cout << "Bench for linear combinations of Vectors<N> :" << std::endl;
	std::cout << " - Eigen::Vector3f : ";
	testLinear<float, 3>();
	std::cout << " - Eigen::Vector3d : ";
	testLinear<double, 3>();
	std::cout << " - Eigen::Vector4f : ";
	testLinear<float, 4>();
	std::cout << " - Eigen::Vector4d : ";
	testLinear<double, 4>();
	std::cout << std::endl;

	std::cout << "Bench for Vector / Matrix products :" << std::endl;
	std::cout << " - Eigen::Vector3f : ";
	testProduct<float, 3>();
	std::cout << " - Eigen::Vector3d : ";
	testProduct<double, 3>();
	std::cout << " - Eigen::Vector4f : ";
	testProduct<float, 4>();
	std::cout << " - Eigen::Vector4d : ";
	testProduct<double, 4>();
	std::cout << std::endl;

	std::cout << "Bench for barycenter :" << std::endl;
	std::cout << " - Eigen::Vector3f : ";
	testBarycenter<float, 3>();
	std::cout << " - Eigen::Vector3d : ";
	testBarycenter<double, 3>();
	std::cout << " - Eigen::Vector4f : ";
	testBarycenter<float, 4>();
	std::cout << " - Eigen::Vector4d : ";
	testBarycenter<double, 4>();
	std::cout << std::endl;

	std::cout << "Bench for triangleNormal :" << std::endl;
	std::cout << " - Eigen::Vector3f : ";
	testTriangleNormal<float, 3>();
	std::cout << " - Eigen::Vector3d : ";
	testTriangleNormal<double, 3>();
	std::cout << " - Eigen::Vector4f : ";
	testTriangleNormal<float, 4>();
	std::cout << " - Eigen::Vector4d : ";
	testTriangleNormal<double, 4>();
	std::cout << std::endl;

	std::cout << "Bench for alignedBox :" << std::endl;
	std::cout << " - Eigen::Vector3f : ";
	testAlignedBox<float, 3>();
	std::cout << " - Eigen::Vector3d : ";
	testAlignedBox<double, 3>();
	std::cout << " - Eigen::Vector4f : ";
	testAlignedBox<float, 4>();
	std::cout << " - Eigen::Vector4d : ";
	testAlignedBox<double, 4>();
	std::cout << std::endl;

	std::cout << "Bench for norm3D :" << std::endl;
	std::cout << " - Eigen::Vector3f : ";
	testNorm3D<float, 3>();
	std::cout << " - Eigen::Vector3d : ";
	testNorm3D<double, 3>();
	std::cout << " - Eigen::Vector4f : ";
	testNorm3D<float, 4>();
	std::cout << " - Eigen::Vector4d : ";
	testNorm3D<double, 4>();
	std::cout << std::endl;
}

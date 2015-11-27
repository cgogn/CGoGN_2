#include <iostream>
#include <geometry/geometry.h>


int main()
{
  Eigen::Matrix3d m = Eigen::Matrix3d::Random();
  m = (m + Eigen::Matrix3d::Constant(1.2)) * 50;

  Eigen::Matrix<double,3,1> u(1,2,3);

  cgogn::Vector<double,3> v(1,2,3);

  Eigen::Vector3d w;

  w = cgogn::lerp<double,3>(u,v,0.5);
  std::cout << "w1 =" << w << std::endl;

  w = cgogn::barycenter<Eigen::Vector3d>(u,v,0.5,0.5);
  std::cout << "w2 =" << w << std::endl;

  w = cgogn::lerp2<double,3>(u,v,0.5);
  std::cout << "w3 =" << w << std::endl;
}

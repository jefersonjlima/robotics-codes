/* ----------------------------------------------------------------------------
 * Copyright 2022, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   examples/1/hTransformation_ex.cpp
 *  @author Jeferson Lima
 *  @brief  Homogeneous Transformation Matrix Example 
 *  @date   Mar 11, 2022
 **/

#include <iostream>
#include <Eigen/Core>

void hTrans3dVec(const Eigen::Matrix<float, 4, 1>& v,
                Eigen::Matrix<float, 4, 1>& r,
		const Eigen::Matrix<float, 3, 1>& q,
		float theta)
{

  Eigen::Matrix<float, 4, 4> htMatrix;
  htMatrix << std::cos(theta), -std::sin(theta), 0.0, q[0],
              std::sin(theta),  std::cos(theta), 0.0, q[1],
	      0.0,              0.0,             1.0, q[2],
	      0.0,              0.0,             0.0, 1.0;
    
  std::cout << "v values:\n" << v << std::endl;
  std::cout << "translation values:\n" << htMatrix << std::endl;

  r = htMatrix * v;

}


int main(int argc, char* argv[])
{

  Eigen::Matrix<float, 4, 1> v;
  Eigen::Matrix<float, 4, 1> r;
  Eigen::Matrix<float, 3, 1> Q;
  float theta;

  // init matrix
  v << 0.5, 0.5, 0.0, 1.0;
  Q << 1.0, 1.0, 0.0;
  theta = 00.523599; // 30 degree -> radian

  hTrans3dVec(v, r, Q, theta);

  std::cout << "r:\n" << r << std::endl;

  return 0;
}

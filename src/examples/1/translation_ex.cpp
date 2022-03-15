/* ----------------------------------------------------------------------------
 * Copyright 2022, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   examples/translation_ex.cpp
 *  @author Jeferson Lima
 *  @brief  Translation Matrix Example 
 *  @date   Mar 11, 2022
 **/

#include <iostream>
#include <Eigen/Core>

void trans3dVec(const Eigen::Matrix<float, 4, 1>& v,
                Eigen::Matrix<float, 4, 1>& r,
		const Eigen::Matrix<float, 3, 1>& q)
{

  Eigen::Matrix<float, 4, 4> translation;
  translation << 1.0, 0.0, 0.0, q[0],
                 0.0, 1.0, 0.0, q[1],
		 0.0, 0.0, 1.0, q[2],
		 0.0, 0.0, 0.0, 1.0;
    
  std::cout << "v values: \n" << v << std::endl;
  std::cout << "translation values: \n" << translation << std::endl;

  r = translation * v;

}


int main(int argc, char* argv[])
{

  Eigen::Matrix<float, 4, 1> v;
  Eigen::Matrix<float, 4, 1> r;
  Eigen::Matrix<float, 3, 1> Q;

  // init matrix
  v << 0.5, 0.5, 0.0, 1.0;
  Q << 1.0, 1.0, 0.0;

  trans3dVec(v, r, Q);

  std::cout << "r:\n" << r << std::endl;

  return 0;
}

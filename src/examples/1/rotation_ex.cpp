/* ----------------------------------------------------------------------------
 * Copyright 2022, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   examples/rotation_ex.cpp
 *  @author Jeferson Lima
 *  @brief  Rotation Matrix Example 
 *  @date   Mar 7, 2022
 **/

#include <iostream>
#include <Eigen/Core>

void rot2dVec(const Eigen::Matrix<float, 4, 1>& v,
              Eigen::Matrix<float, 4, 1>& r,
	      float theta)
{

  Eigen::Matrix<float, 4, 4> rotation;
  rotation << std::cos(theta), -std::sin(theta), 0.0, 0.0, 
              std::sin(theta), std::cos(theta),  0.0, 0.0,
	      0.0,             0.0,              1.0, 0.0,
	      0.0,             0.0,              0.0, 1.0;

  std::cout << "v values: \n" << v << std::endl;
  std::cout << "rotation values: \n" << rotation << std::endl;

  r = rotation * v;

}


int main(int argc, char* argv[])
{

  Eigen::Matrix<float, 4, 1> v;
  Eigen::Matrix<float, 4, 1> r;
  float theta;

  // init matrix
  v << 0.5, 0.5, 0.0, 1.0;
  theta = 0.785398; // 45 degree -> radian

  rot2dVec(v, r, theta);

  std::cout << "r values:\n" << r << std::endl;

  return 0;
}

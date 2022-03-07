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

void rot2dVec(const Eigen::Matrix<float, 2, 1>& v, Eigen::Matrix<float, 2, 1>& r, float theta)
{

  Eigen::Matrix<float, 2, 2> rotation;
  rotation << std::cos(theta), -std::sin(theta),
              std::sin(theta), std::cos(theta);

  std::cout << "v value: \n" << v << std::endl;
  std::cout << "rotation value: \n" << rotation << std::endl;

  r = rotation * v;

}


int main(int argc, char* argv[])
{

  Eigen::Matrix<float, 2, 1> v;
  Eigen::Matrix<float, 2, 1> r;
  float theta;

  // init matrix
  v << 0.5, 0.5;
  theta = 0.785398;

  rot2dVec(v, r, theta);

  std::cout << "v(0): " << r[0] << std::endl;
  std::cout << "v(1): " << r[1] << std::endl;

  return 0;
}

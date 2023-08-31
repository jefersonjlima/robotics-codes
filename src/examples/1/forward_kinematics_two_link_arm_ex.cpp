/* ----------------------------------------------------------------------------
 * Copyright 2023, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   examples/1/forward_kinematics_two_link_arm_ex.cpp
 *  @author Jeferson Lima
 *  @brief  forward kinematics 2-link arm
 *  @date   Ago 21, 2023
 **/

#include <iostream>
#include <Eigen/Core>
#include <cmath>
#include <Eigen/Jacobi>
#define n_links 3

Eigen::IOFormat Fmt(2, 0, ", ", ";\n", "", "", "[", "]");

float deg2rad(float deg)
{
  return deg * M_PI / 180.0;
}

void hTrans3dVec(
    const Eigen::Matrix<float, 4, 1>& v,
    const Eigen::Matrix<float, 3, 1>& q,
    const float& theta,
    Eigen::Matrix<float, 4, 1>& out)
{
  Eigen::Matrix<float, 4, 4> htMatrix;
  htMatrix << std::cos(theta), -std::sin(theta), 0.0, q[0],
  std::sin(theta),  std::cos(theta), 0.0, q[1],
  0.0,              0.0,             1.0, q[2],
  0.0,              0.0,             0.0, 1.0;

  std::cout << "homogeneous transformation values:\n"
            << htMatrix.format(Fmt) << std::endl;

  out = htMatrix * v;
}

int main(int argc, char* argv[])
{
  Eigen::Matrix<float, 4, 1> v;
  Eigen::Matrix<float, 4, 1> out;
  Eigen::Matrix<float, 3, 1> Q;
  const float theta[n_links] {
    deg2rad(0.0),
    deg2rad(0.0),
    deg2rad(90.0)};
  const float link_size = 1.0;

  // robot base 
  v << 0.0, 0.0, 0.0, 1.0;
  Q << link_size, 0.0, 0.0;

  for (int i=0; i < n_links; i++)
  {
    hTrans3dVec(v, Q, theta[i], out);
    std::cout << "Global position of Link_"
              << i
              << "\n"
              << out.format(Fmt) << std::endl;
    v = out;
  }

  return 0;
}

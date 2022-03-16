/* ----------------------------------------------------------------------------
 * Copyright 2022, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   examples/rPlot_tst.cpp
 *  @author Jeferson Lima
 *  @brief  Rotation Matrix Plot Test 
 *  @date   Mar 11, 2022
 **/

#include <iostream>
#include <Eigen/Core>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

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
  std::cout << "matrix rotation values: \n" << rotation << std::endl;

  r = rotation * v;

}

void plotGraph(const Eigen::Matrix<float, 4, 1>& vec)
{
  std::vector<double> x,y;

  plt::figure();
  // standard axes
  x = {0,0.5,0,0,0,-0.1};
  y = {0,0,0,0.5,0,-0.1};
  plt::plot(x, y);

  // plot v
  x.clear(); y.clear();
  x = {0, vec[0]};
  y = {0, vec[1]};
  plt::plot(x, y);

  plt::show();

}

int main(int argc, char* argv[])
{

  Eigen::Matrix<float, 4, 1> v;
  Eigen::Matrix<float, 4, 1> r;
  float theta;

  // init matrix
  v << 0.5, 0.5, 0.0, 1.0;
  theta = 00.523599; // 30 degree -> radian

  rot2dVec(v, r, theta);

  std::cout << "r values:\n" << r << std::endl;

  plotGraph(v);

  return 0;
}

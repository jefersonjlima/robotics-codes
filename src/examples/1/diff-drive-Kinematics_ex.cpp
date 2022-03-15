/* ----------------------------------------------------------------------------
 * Copyright 2022, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   examples/1/diff-drive-kinematics_ex.cpp
 *  @author Jeferson Lima
 *  @brief  Differential drive kinematics Example 
 *  @date   Mar 14, 2022
 **/

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <boost/array.hpp>
#include <matplotlibcpp.h>

using namespace boost::numeric::odeint;
namespace plt = matplotlibcpp;

const double vR = 0.05; // right wheel
const double vL = 0.06; // left  wheel
const double L  = 0.1;  // distance between wheels 
typedef boost::array< double , 3 > state;
std::vector<double> X{0}, Y{0};

void car_model(const state& x, state& dxdt, double t)
{

  double v = (vR + vL)/2;
  double w = (vR - vL)/L;

  dxdt[0] = std::cos(x[2])*v;
  dxdt[1] = std::sin(x[2])*v;
  dxdt[2] = w;
}

void log_model(const state& x, const double t)
{
  X.push_back(x[0]);
  Y.push_back(x[1]);
  std::cout << t << '\t' << x[0] << '\t' << x[1] << '\t' << x[2] << std::endl;
}

int main(int argc, char** argv)
{
    state x = { 0.0 , 0.0 , 0.0 }; // initial conditions

    plt::figure();
    plt::named_plot("Initial Position", X, Y, "o");

    runge_kutta4< state > stepper;
    integrate_const( stepper , car_model, x , 0.0 , 20.0 , 0.1, log_model );

    plt::named_plot("Car position", X, Y);
    plt::legend();
    plt::show();

  return 0;
}

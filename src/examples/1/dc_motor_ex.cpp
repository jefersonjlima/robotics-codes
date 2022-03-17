/* ----------------------------------------------------------------------------
 * Copyright 2022, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   examples/1/dc_motor_ex.cpp
 *  @author Jeferson Lima
 *  @brief  DC Motor Simulation Example 
 *  @date   Mar 15, 2022
 **/

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <boost/array.hpp>
#ifdef USE_MATPLOTLIB
 #include <matplotlibcpp.h>
 namespace plt = matplotlibcpp;
#endif 

using namespace boost::numeric::odeint;

const double j = 0.01; 	// (J)     moment of inertia of the rotor     0.01 kg.m^2
const double b = 0.1;	// (b)     motor viscous friction constant    0.1 N.m.s
const double Ke = 0.01; // (Ke)    electromotive force constant       0.01 V/rad/sec
const double Kt = 0.01; // (Kt)    motor torque constant              0.01 N.m/Amp
const double R = 1;	// (R)     electric resistance                1 Ohm
const double L = 0.5; 	// (L)     electric inductance                0.5 H
const double V = 12; 	// (V)     motor votage                       12 V

typedef boost::array< double , 2 > state;
#ifdef USE_MATPLOTLIB
 std::vector<double> ts, dTheta, di;
#endif

void dc_motor_model(const state& x, state& dxdt, double t)
{
  dxdt[0] = -b*x[0]/j + Kt*x[1]/j;
  dxdt[1] = -R*x[1]/L + V/L - Ke*x[0];
}

void log_model(const state& x, const double t)
{
#ifdef USE_MATPLOTLIB 
  ts.push_back(t);
  dTheta.push_back(x[0]);
  di.push_back(x[1]);
#endif
  std::cout << t << '\t' << x[0] << '\t' << x[1]  << std::endl;
}

int main(int argc, char** argv)
{
    state x = { 0.0 , 0.0 }; // initial conditions
    runge_kutta4< state > stepper;
    integrate_const( stepper , dc_motor_model, x , 0.0 , 20.0 , 0.1, log_model );

#ifdef USE_MATPLOTLIB
    plt::figure();
    plt::named_plot("Velocity (rad/s)", ts, dTheta);
    plt::named_plot("Current (A)", ts, di);
    plt::legend();
    plt::show();
#endif
  return 0;
}

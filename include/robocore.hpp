/* ----------------------------------------------------------------------------
 * Copyright 2022, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   robocore.hpp
 *  @author Jeferson Lima
 *  @brief  Robotics Codes header file
 *  @date   Mar 14, 2022
 **/

#ifndef ROBOCORE_HPP
#define ROBOCORE_HPP

#ifndef PLOT_OFF
#include <matplotlibcpp.h>
#endif

#include <iostream>


class RoboCore{
  private:


  public:

    RoboCore();

    static void plotGraph(const Eigen::Matrix<float, 4, 1>& vec);

};

#endif

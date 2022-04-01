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

#ifdef SAVE_OUTPUT_CSV
 #include <string>
 #include <fstream>
 #include <vector>
 #include <utility>

 typedef std::vector<std::pair<std::string,
 			std::vector<double>>> columns;
#endif

class RoboCore{
  private:

  public:
    RoboCore();
    ~RoboCore(){};

#ifdef SAVE_OUTPUT_CSV
    static void write_csv(std::string filename, columns dataset);
#endif

};

#endif

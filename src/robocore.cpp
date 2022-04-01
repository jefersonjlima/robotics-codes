/* ----------------------------------------------------------------------------
 * Copyright 2022, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   robocode.cpp
 *  @author Jeferson Lima
 *  @brief  Robotics Codes source file
 *  @date   Mar 14, 2022
 **/

#include <robocore.hpp>


RoboCore::RoboCore()
{
}


#ifdef SAVE_OUTPUT_CSV
void RoboCore::write_csv(std::string filename, columns dataset)
{
  std::ofstream File(filename);

  // Send column names to the stream
  for (int j = 0; j < dataset.size(); ++j)
  {
    File << dataset.at(j).first;
    if(j != dataset.size() - 1) File << ";";
  }
  File << "\n";

  for(int i = 0; i < dataset.at(0).second.size(); ++i)
  {
    for(int j = 0; j < dataset.size(); ++j)
    {
      File << dataset.at(j).second.at(i);
      if(j != dataset.size() - 1) File << ","; // No comma at end of line
    }
    File << "\n";
  }
}
#endif


/* ----------------------------------------------------------------------------
 * Copyright 2022, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   examples/3/sensor_node.cpp
 *  @author Jeferson Lima
 *  @brief  Temperature Sensor Example 
 *  @date   Mar 28, 2022
 **/

#include <cstdlib>
#include "ros/ros.h"
#include "std_msgs/Float64.h"


/* @brief minimal ros publish source */
int main(int argc, char **argv)
{

  std_msgs::Float64 sensor;

  ros::init(argc, argv, "temperature_sensor_node");

  ros::NodeHandle n;

  ros::Publisher sensor_pub = n.advertise<std_msgs::Float64>("sensor_node", 10);

  ros::Rate loop_rate(2);

  while(ros::ok())
  {
    // simutale sensor
    sensor.data = 10.0/(rand() % 10 + 1) + 20.0; // 20 -> 30 degree
    ROS_INFO("Temperature: %.2f", sensor.data);

    sensor_pub.publish(sensor);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}


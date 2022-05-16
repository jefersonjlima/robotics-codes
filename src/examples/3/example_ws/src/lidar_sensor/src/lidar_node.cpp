/* ----------------------------------------------------------------------------
 * Copyright 2022, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   examples/3/lidar_sensor/src/lidar_node.cpp
 *  @author Jeferson Lima
 *  @brief  Node to plot lidar data
 *  @date   May 2, 2022
 **/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <matplot/matplot.h>

using namespace matplot;

void lidarShow(const sensor_msgs::LaserScan::ConstPtr msg)
{
  std::vector<double> theta = linspace(0, 2 * pi, 360);
  auto sensor = std::vector<float>(msg->ranges.size());
  ROS_INFO("Message Length: %ld", msg->ranges.size());

  std::transform(
      msg->ranges.begin(), msg->ranges.end(), sensor.begin(),
      [](float it) { return (it > 3.5)? 3.5: it; });

  ROS_INFO("Sensor Data:");
  for(auto it = std::begin(sensor); it != std::end(sensor); ++it)
  {
    std::cout << *it << ", ";
  }
  std::cout << std::endl;

  polarplot(theta, sensor);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_node");
  ros::NodeHandle nh;

  ros::Subscriber sensor_sub = nh.subscribe("/scan", 100, lidarShow);
  ros::Rate loop_rate(1);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

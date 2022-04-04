/* ----------------------------------------------------------------------------
 * Copyright 2022, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   examples/3/temperature_sensor/src/app_node.cpp
 *  @author Jeferson Lima
 *  @brief  App for Temperature Sensor node Example 
 *  @date   Apr 4, 2022
 **/

#include <ros/ros.h>
#include <std_msgs/Float64.h>

class App{
  //variables
  ros::Subscriber sensor_sub;
  ros::NodeHandle _nh;

  void sensorCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    ROS_INFO("I heard too %.2f", msg->data);
  }

  public:
    //construct
    App(ros::NodeHandle nh): _nh(nh)
    {
      sensor_sub = nh.subscribe("sensor/value", 10,
	  &App::sensorCallback, this);
    }
    //destructor
    ~App()
    {
      _nh.shutdown();
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cpp_app_node");
  ros::NodeHandle nh;
  App app(nh);
  ros::Rate loop_rate(2);

  while(ros::ok())
    ros::spin();

  return 0;
}


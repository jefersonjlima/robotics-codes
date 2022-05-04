/* ----------------------------------------------------------------------------
 * Copyright 2022, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   examples/3/motor_control/src/control_node.cpp
 *  @author Jeferson Lima
 *  @brief  Simulate a pwm control
 *  @date   May 4, 2022
 **/

#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char **argv)
{ 

  ros::init(argc, argv, "control_node");
  ros::NodeHandle n;

  // --------------------------------------------------------------------------
  // TODO
  // create publish topic  motor/control/pwm 
  // add publish frequence


  // --------------------------------------------------------------------------
  
  std_msgs::Int32 pwm;
  pwm.data = 0;

  while(ros::ok())
  {

  // --------------------------------------------------------------------------
  // TODO
  // Publish pwm variable
  // update pwm variable


  // --------------------------------------------------------------------------
  }

  return 0;
}

/* ----------------------------------------------------------------------------
 * Requirements
 *
 * pkg             motor_control
 * topic           motor/control/pwm publisher
 * message type    std_msgs/Int32
 * frequence       10Hz
 * -------------------------------------------------------------------------- */

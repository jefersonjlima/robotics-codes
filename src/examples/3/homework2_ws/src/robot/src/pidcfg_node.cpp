/* ----------------------------------------------------------------------------
 * Copyright 2022, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   examples/3/robot/src/pidcfg_node.cpp
 *  @author Jeferson Lima
 *  @brief  Create topics to configure Robot PID
 *  @date   May 4, 2022
 **/


#include <ros/ros.h>
#include <std_msgs/Float32.h>

class PIDconfig
{
  private:
    ros::Subscriber getP_;

    // --------------------------------------------------------------------------
    // TODO
    // Continue ...
    

    // --------------------------------------------------------------------------

    void setPGain(const std_msgs::Float32::ConstPtr& msg)
    {
      ROS_INFO("Setting P Gain to %.2f", msg->data);
      p_ = msg->data;
    }
    // --------------------------------------------------------------------------
    // TODO
    // Continue ...
    

    // --------------------------------------------------------------------------

  public:
    PIDconfig(ros::NodeHandle nh)
    {
      getP_ = nh.subscribe("robot/control/p", 10, &PIDconfig::setPGain, this);
      getI_ = nh.subscribe("robot/control/i", 10, &PIDconfig::setIGain, this);
      getD_ = nh.subscribe("robot/control/d", 10, &PIDconfig::setDGain, this);

    };
    ~PIDconfig(){};
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "pidcfg_node");
  ros::NodeHandle nh;
  PIDconfig cfg(nh);


  while(ros::ok())
  {
    // mesurement sensors
    
    // calc error
 
    // apply control
 
    ros::spin();
  }

  return 0;
}

/* ----------------------------------------------------------------------------
 * Requirements
 *
 * pkg          robot
 * topics       robot/control/p
                robot/control/i
                robot/control/d
 * message      std_msgs/Float32
 * -------------------------------------------------------------------------- */

		

#!/usr/bin/env python

# ----------------------------------------------------------------------------
# Copyright 2022, Jeferson Lima
# All Rights Reserved
# See LICENSE for the license information
# -------------------------------------------------------------------------- */
# 
#  @file   examples/3/sensor_node.cpp
#  @author Jeferson Lima
#  @brief  Temperature Sensor Example 
#  @date   April 04, 2022

import rospy
from std_msgs.msg import Float64

class App:
    def __init__(self):

        rospy.init_node('py_app_node', anonymous=False)
        rospy.Subscriber('sensor/value', Float64, self.update)
        self.value = Float64()

    def update(self, msg):

        self.value.data = msg.data
        rospy.loginfo(f"I heard: {self.value.data}")

    def run(self):

        rospy.spin()

if __name__ == "__main__":
    try:
        app = App()
        app.run()
    except rospy.ROSInterruptException:
        pass

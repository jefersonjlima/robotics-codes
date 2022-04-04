#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from numpy import random

def sensor():

    rospy.init_node('sensor_node', anonymous=False)
    pub = rospy.Publisher('sensor/value', Float64, queue_size = 10)
    rate = rospy.Rate(2) #Hz

    value = Float64()

    while not rospy.is_shutdown():

        value.data = 20 + 10*random.rand() # 20 <-> 30 degree
        rospy.loginfo(f"The value is: {value.data}")
        pub.publish(value)

        rate.sleep()


if __name__ == "__main__":
    try:
        sensor()
    except rospy.ROSInterruptException:
        pass

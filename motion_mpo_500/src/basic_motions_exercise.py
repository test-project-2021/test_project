#! /usr/bin/env python

import rospy, math, numpy as np
from std_msgs.msg import Float32MultiArray
from utilities import reset_world
import rospy 
import time 

rospy.init_node("swedish_wheel")

# rospy.init_node('holonomic_controller', anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
rate = rospy.Rate(0.1)
rospy.sleep(1)
while not rospy.is_shutdown():
    print("hi")
    forward = [1, 1, 1, 1]
    msg_1 = Float32MultiArray(data=forward)
    pub.publish(msg_1)
    rate.sleep()

    backward = [-1, -1, -1, -1]
    msg_1 = Float32MultiArray(data=backward)
    pub.publish(msg_1)
    rate.sleep()


    left_trans = [-1, 1, -1, 1]
    msg_1 = Float32MultiArray(data=left_trans)
    pub.publish(msg_1)
    rate.sleep()

    right_trans = [1, -1, 1, -1]
    msg_1 = Float32MultiArray(data=right_trans)
    pub.publish(msg_1)
    rate.sleep()

    clockwise = [-1, 1, 1, -1]
    msg_1 = Float32MultiArray(data=clockwise)
    pub.publish(msg_1)
    rate.sleep()

    rate.sleep()
    counter_clock = [1, -1, -1, 1]
    msg_1 = Float32MultiArray(data=counter_clock)
    pub.publish(msg_1)


    rate.sleep()

    # stop = [0, 0, 0, 0]
    # msg = Float32MultiArray(data=stop)
    # pub.publish(msg)
    # rate.sleep()
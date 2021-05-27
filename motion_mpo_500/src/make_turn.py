#! /usr/bin/env python

import rospy, math, numpy as np
from std_msgs.msg import Float32MultiArray
from utilities import reset_world
import rospy 
import time 

rospy.init_node("swedish_wheel")

pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)

rospy.sleep(1)
l=0.500/2
r= 0.274/2
w=0.548/2

H = np.array([[-l-w, 1, -1],[l+w, 1, 1],[l+w, 1, -1],[-l-w, 1, 1]])/r
rospy.sleep(1)


def twist2wheels(wz, vx, vy):
    twist = np.array([wz, vx, vy])
    twist.shape=(3,1)
    u = np.dot(H,twist)

    return u.flatten().tolist()
rospy.sleep(1)

u = twist2wheels(wz=-1.5, vx=1, vy=0)
msg = Float32MultiArray(data=u)
pub.publish(msg)
rospy.sleep(3)
stop = [0,0,0,0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)






# while not rospy.is_shutdown():
#     print("hi")
#     forward = [1, 1, 1, 1]
#     msg_1 = Float32MultiArray(data=forward)
#     pub.publish(msg_1)
#     rate.sleep()

#     backward = [-1, -1, -1, -1]
#     msg_1 = Float32MultiArray(data=backward)
#     pub.publish(msg_1)
#     rate.sleep()


#     left_trans = [-1, 1, -1, 1]
#     msg_1 = Float32MultiArray(data=left_trans)
#     pub.publish(msg_1)
#     rate.sleep()

#     right_trans = [1, -1, 1, -1]
#     msg_1 = Float32MultiArray(data=right_trans)
#     pub.publish(msg_1)
#     rate.sleep()

#     clockwise = [-1, 1, 1, -1]
#     msg_1 = Float32MultiArray(data=clockwise)
#     pub.publish(msg_1)
#     rate.sleep()

#     rate.sleep()
#     counter_clock = [1, -1, -1, 1]
#     msg_1 = Float32MultiArray(data=counter_clock)
#     pub.publish(msg_1)


#     rate.sleep()

#     # stop = [0, 0, 0, 0]
#     # msg = Float32MultiArray(data=stop)
#     # pub.publish(msg)
#     # rate.sleep()
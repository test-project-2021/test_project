#! /usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist



rospy.init_node("publish_vel")
pub = rospy.Publisher('/twist_vels', Twist, queue_size=1)
vel = Twist()
vel.linear.x = 0.5
vel.angular.z = 0.5
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    # print("hi")
    pub.publish(vel)
    rate.sleep
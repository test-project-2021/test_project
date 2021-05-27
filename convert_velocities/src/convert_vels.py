#! /usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class Vel_Converter: 

    def __init__(self):
        self.sub = rospy.Subscriber('/twist_vels', Twist, self.callback)
        self.pub_left = rospy.Publisher('/left_wheel_controller/command', Float64)
        self.pub_right = rospy.Publisher('/right_wheel_controller/command', Float64)
        self.v_r = Float64()
        self.v_l = Float64()
        self.velo = Twist()
        self.rate = rospy.Rate(1)

    def callback(self, msg):
        self.velo = msg


    # the lenght between the wheels L = 0.3
    # the radius of the wheel R=0.1
    def Converter(self):
        self.v_r = ((2*self.velo.linear.x)+(self.velo.angular.z*0.3))/(2*0.1)
        self.v_l = ((2*self.velo.linear.x)- (self.velo.angular.z*0.3))/(2*0.1)

    def vel_publisher(self):

        while not rospy.is_shutdown():
            self.Converter()
            self.pub_left.publish(self.v_l)
            self.pub_right.publish(self.v_r)
            # print("hi")
            self.rate.sleep()



if __name__=='__main__':
    rospy.init_node('vel_subscriber', anonymous=True)
    conver= Vel_Converter()
    conver.vel_publisher()






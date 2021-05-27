#! /usr/bin/env python
#===============================================================
#Name and Surname: Roman Ibrahimov
#Email address: ibrahir@purdue.edu
#Program Description: 
'''
Given the coordinates of the room, based on the set points, the robot 
fulfils the movement of an autonomous lawn maker. Closed loop control 
strategy is used to coreect the path of the robot.  
'''
#=================================================================

#importing impart packages and libraries
import rospy, math, numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

#initializing "vacuum_cleaner" node
rospy.init_node('vacuum_cleaner', anonymous=True)

#class used to publish the angular and linear velocities to the robot
class VelocityController():
    def __init__(self, topic):
        self.cmd_vel = rospy.Publisher(topic, Twist, queue_size=10)
        self.max_vel = 0.65
        rospy.sleep(0.1)

    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        abs_v = abs(linear_velocity)
        if abs_v <= self.max_vel:
            vx = linear_velocity
            wz = angular_velocity
        else:
            vx = linear_velocity / abs_v * self.max_vel
            wz = angular_velocity / abs_v * self.max_vel
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.cmd_vel.publish(msg)


#class that is used for odometry data for closed loop control
class OdometryReader():
    def __init__(self, topic):
        self.pose = {}
        self.trajectory = []
        self.topic = topic
        self.subscribe()

    def callback(self, msg):
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y
        self.trajectory.append((self.pose['x'], self.pose['y']))
        (_, _, self.pose['theta']) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                            msg.pose.pose.orientation.y, 
                                                            msg.pose.pose.orientation.z, 
                                                            msg.pose.pose.orientation.w])
    def subscribe(self):
        self.subscriber = rospy.Subscriber(self.topic, Odometry, self.callback)
        rospy.sleep(0.1)

    def unregister(self):
        np.save('trajectory',self.trajectory)
        self.subscriber.unregister()
#simple function to find the arc tanges    
def normalize(angle):
    return math.atan2(math.sin(angle),math.cos(angle))
#the closed loop controller to reach the goal position with minimal errors
def go_to(xg, yg, thetag_degrees, constant_vel = None):
    #controller coefficients
    k_rho = 0.3
    k_alpha = 0.8
    k_beta = -0.15
    #the distance to the goal
    rho = float("inf")
    #desired angular position
    thetag = math.radians(thetag_degrees)
    #the loop used to correct the error
    while rho>0.01:
        dx = xg - odometry.pose['x']
        dy = yg - odometry.pose['y']
        rho = math.sqrt(dx**2 + dy**2)
        theta = odometry.pose['theta']
        alpha = normalize(math.atan2(dy, dx) - theta)
        beta = normalize(thetag - math.atan2(dy, dx))
        v = k_rho * rho
        w = k_alpha * alpha + k_beta * beta
        if constant_vel:
            abs_v = abs(v)
            v = v / abs_v * constant_vel
            w = w / abs_v * constant_vel
        velocity.move(v, w)
        rospy.sleep(0.01)

#all the initial Euler angles are set to be 0
roll = pitch = yaw = 0.0
target = 90
kp=0.5


# convertion to quaternions
def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
  

#subscribtion for odometry 
sub = rospy.Subscriber ('/odom', Odometry, get_rotation) 
#publisher for the velocity of the robot
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
r = rospy.Rate(10)
command =Twist()

#this function is used to rotate the robot along Z-axis
#the function takes target angle and rotates the robot towards this angle
def rotate(target):
    target_rad = target*math.pi/180
    #while there is an error of 0.1 between the target and current angle
    # we are incrementing the current angle to reach the target
    #the coefficient is kp
    while (np.abs(target_rad - yaw) >0.1):
        target_rad = target*math.pi/180
        command.angular.z = kp * (target_rad-yaw)
        pub.publish(command)
        r.sleep()
    return True



from utilities import set_position, cleaned_area
set_position(0,0,0)
#getting the topics for classes
velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')
start_time = rospy.get_time()

try:
    #our set point was (0,0)
    #starting from the set point we go to the left edge of the smaller room 
    rotate(-90)
    go_to(0,-2,-90, constant_vel=0.3)
    rotate(180)
    go_to(-4.5,-2,-180, constant_vel=0.3)

#each time the sequence of the movements are the same. The robot makes lawn maker movement along the room
# each step is enumareted
#1
#go up
    rotate(90)
    go_to(-4.5,1.1, -270, constant_vel=0.3)
#go right 
    rotate(0)
    go_to(-4.2,1.1,0, constant_vel=0.2)

#go down
    rotate(-90)
    go_to(-4.2,-1.8, -90, constant_vel=0.3)
#go right 
    rotate(0)
    go_to(-3.9,-1.8,0, constant_vel=0.2)



#2

    #from left to right
    rotate(0)
    go_to(4.5,-1.4,0, constant_vel=0.3)
    
    #right up
    rotate(90)
    go_to(4.5,-1.1,-270, constant_vel=0.2)

    #from right to left 
    rotate(180)
    go_to(-4.5,-1.1,-180, constant_vel=0.3)

    #left up
    rotate(90)
    go_to(-4.5,-0.8,-270, constant_vel=0.2)
#3 

    #from left to right
    rotate(0)
    go_to(4.5,-0.8,0, constant_vel=0.3)
    
    #right up
    rotate(90)
    go_to(4.5,-0.5,-270, constant_vel=0.2)

    #from right to left 
    rotate(180)
    go_to(-4.5,-0.5,-180, constant_vel=0.3)

    #left up
    rotate(90)
    go_to(-4.5,-0.2,-270, constant_vel=0.2)


#4 
    #from left to right
    rotate(0)
    go_to(4.5,-0.2,0, constant_vel=0.3)
    
    #right up
    rotate(90)
    go_to(4.5,0.1,-270, constant_vel=0.2)

    #from right to left 
    rotate(180)
    go_to(-4.5,0.1,-180, constant_vel=0.3)

    #left up
    rotate(90)
    go_to(-4.5,0.3,-270, constant_vel=0.2)

    #5
    #from left to right
    rotate(0)
    go_to(4.5,0.3,0, constant_vel=0.3)
    
    #right up
    rotate(90)
    go_to(4.5,0.6,-270, constant_vel=0.2)

    #from right to left 
    rotate(180)
    go_to(-4.5,0.6,-180, constant_vel=0.3)

    #left up
    rotate(90)
    go_to(-4.5,0.9,-270, constant_vel=0.2)


except KeyboardInterrupt:
    pass

end_time = rospy.get_time()    
velocity.move(0,0)
odometry.unregister()
#calculating the time spent and the area covered 
t = end_time-start_time
m = int(t/60)
s = t - m*60
print("You cleaned %.2f m2 in %d minutes and %d seconds." % 
      (cleaned_area(odometry.trajectory), m, s))
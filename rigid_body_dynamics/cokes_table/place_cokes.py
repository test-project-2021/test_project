import math, rospy
from utilities import set_model_state, get_model_state, \
                      pause_physics, unpause_physics
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_about_axis, quaternion_multiply

from utilities import set_model_state, get_model_state, \
                      spawn_coke_can

# position = Point(x=0.5, y=0, z=0.5)
model_state = get_model_state('table')
pose = model_state.pose
print(pose.position.x)
print(pose.position.y)
print(pose.position.z)
x, y, z = 0.4,0,1.05
for i in range(12):
    theta = math.radians(i*30)
    xp = x*math.cos(theta)- y*math.sin(theta)+1
    yp = x*math.sin(theta) + y*math.cos(theta)
    # xp = x*math.cos(theta)+1
    # yp = x*math.sin(theta) + 0.1
    alpha = math.radians(90)
    name = 'coke_can'+'_'+str(i)
    print(name)
    pose = Point(xp, yp, z)
    q_z = quaternion_about_axis(theta, (0,0,1))
    q_y = quaternion_about_axis(alpha, (0,1,0))
    q_zy = quaternion_multiply(q_z, q_y)
    orientation = Quaternion(*q_zy)
    set_model_state(name, Pose(pose, orientation))
    rospy.sleep(1)


#solution in the text
# for angle in range(0,360,30):
#     can = angle/30
#     theta = math.radians(angle)
#     xp = 0.2 * math.cos(theta) + 1
#     yp = 0.2 * math.sin(theta)
#     model_name = 'coke_can_' + str(can)
#     position = Point(xp,yp,1.05)
#     q_z = quaternion_about_axis(theta, (0,0,1))
#     q_y = quaternion_about_axis(math.radians(90), (0,1,0))
#     q_zy = quaternion_multiply(q_z, q_y)
#     orientation = Quaternion(*q_zy)
#     set_model_state(model_name, Pose(position, Quaternion(*q_zy)))
#     rospy.sleep(0.1)
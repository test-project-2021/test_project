import numpy as np
from plot_functions import plot_trajectory

path_to_file = '/home/user/catkin_ws/src/kinematic_control/src/'
open_loop_trajectory = np.load(path_to_file+'trajectory.npy')
plot_trajectory(open_loop_trajectory)
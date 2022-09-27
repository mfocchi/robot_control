from __future__ import print_function
import numpy as np
from  inv_kinematics_pinocchio import robotKinematics
import time
import rospkg

import sys
sys.path.append('../utils')#allows to incude stuff on the same level
from base_controllers.utils.common_functions import getRobotModel
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
from termcolor import colored
from base_controllers.utils.math_tools import Math

math_utils = Math()
#use a reasonable guess
q_guess = np.array([0.5, -0.7, 1.0, -1.57, -1.57, 0.5])
robot_name = "ur5"
xacro_path = rospkg.RosPack().get_path('ur_description') + '/urdf/ur5.xacro'
robot = getRobotModel(robot_name, generate_urdf=True, xacro_path=xacro_path)
robot = getRobotModel(robot_name)
ee_frame = 'tool0'

## IMPORTANT these value is reasonable only for ur_description urdf ! not for the one in example robot data! they have base frames rotated!
ee_pos_des = np.array([0.58281, 0.48628, 0.08333])
#This is the result it should get close (not that there are infinite solutions without the postural task!)
qtest = np.array([ 0.55053, -0.65785,  1.05608, -1.55222, -1.56608,  0.5])

#orient reference
rpy_des = np.array([np.pi/2, 0.1 , 0.2])
# compute rotation matrix representing the desired orientation from Euler Angles
w_R_e_des = math_utils.eul2Rot(rpy_des)   
print(w_R_e_des)
kin = robotKinematics(robot, ee_frame)
start_time = time.time()
q, ik_success, out_of_workspace = kin.endeffectorFrameInverseKinematicsLineSearch(ee_pos_des, w_R_e_des, ee_frame, q_guess, verbose = True)

#chek orient
#print( robot.framePlacement(q, robot.model.getFrameId(ee_frame)).rotation)

print('total time is ',time.time()-start_time)
#print('q is:\n', q)
print('Result is: ' , q)
#last joint does not affect end effector position (only orientation) so I will discard from test
if np.allclose(q[:-1], qtest[:-1], 0.001):
    print(colored("TEST PASSED", "green"))



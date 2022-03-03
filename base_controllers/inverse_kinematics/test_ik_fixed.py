from __future__ import print_function
import numpy as np
from  inv_kinematics_pinocchio import robotKinematics
import time
import rospkg

import sys
sys.path.append('../utils')#allows to incude stuff on the same level
from common_functions import getRobotModel
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)

ee_pos_des = np.array([-0.06745,  0.85615 , 1.66109])
#use a reasonable guess
q0 = np.array([0.5, -0.7, 1.0, -1.57, -1.57, 0.5])

robot_name = "ur5"
xacro_path = rospkg.RosPack().get_path('ur_description') + '/urdf/ur5.xacro'
robot = getRobotModel(robot_name, generate_urdf=True, xacro_path=xacro_path)
robot = getRobotModel(robot_name)
ee_frame = 'tool0'

qtest = np.array([ 0.54593 ,-0.6541 ,  1.04593 ,-1.52407 ,-1.52407 , 0.54593])
#Jee= [[ 0.50615  0.02016  0.24117  0.11317  0.04997  0.     ]
#  [ 0.56745 -0.01225 -0.14651 -0.06875  0.08606  0.     ]
#  [-0.       0.74777  0.41049  0.04802 -0.00421  0.     ]]


kin = robotKinematics(robot, ee_frame)
start_time = time.time()
q, ik_success = kin.endeffectorInverseKinematicsLineSearch(ee_pos_des,ee_frame, qtest, verbose = True)
print('total time is ',time.time()-start_time)
#print('q is:\n', q)
print('\n flatten result is: \n' , q)


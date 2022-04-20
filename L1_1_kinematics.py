#common stuff 
from __future__ import print_function
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm

from base_controllers.utils.common_functions import *
from base_controllers.utils.ros_publish import RosPub
from base_controllers.utils.kin_dyn_utils import directKinematics
from base_controllers.utils.kin_dyn_utils import computeEndEffectorJacobian
from base_controllers.utils.kin_dyn_utils import numericalInverseKinematics as ik
from base_controllers.utils.kin_dyn_utils import fifthOrderPolynomialTrajectory as coeffTraj
from base_controllers.utils.kin_dyn_utils import geometric2analyticJacobian as g2a
from base_controllers.inverse_kinematics.inv_kinematics_pinocchio import robotKinematics
from base_controllers.utils.math_tools import Math
import matplotlib.pyplot as plt

import L1_conf as conf

os.system("killall rosmaster rviz")
#instantiate graphic utils
ros_pub = RosPub("ur4")
robot = getRobotModel("ur4")

# Init variables
zero = np.array([0.0, 0.0, 0.0, 0.0])
time = 0.0

# Init loggers
q_log = np.empty((4))*nan
q_des_log = np.empty((4))*nan
qd_log = np.empty((4))*nan
qd_des_log = np.empty((4))*nan
qdd_log = np.empty((4))*nan
qdd_des_log = np.empty((4))*nan
tau_log = np.empty((4))*nan
f_log = np.empty((3,0))*nan
x_log = np.empty((3,0))*nan
time_log =  np.empty((0,0))*nan

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des = conf.q0
qd_des = conf.qd0
qdd_des = conf.qdd0

math_utils = Math()

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

#################
# exercise 2.1 - Direct kinematics
#################
# direct kinematics function
T_01, T_02, T_03, T_04, T_0e = directKinematics(q)
# compare with Pinocchio built-in functions 
robot.computeAllTerms(q, qd)
x = robot.framePlacement(q, frame_ee).translation
o = robot.framePlacement(q, frame_ee).rotation
position_diff = x - T_0e[:3,3]
rotation_diff = o - T_0e[:3,:3]
print("Direct Kinematics - ee position, differece with Pinocchio library:", position_diff)
print("Direct Kinematics - ee orientation, differece with Pinocchio library:\n", rotation_diff)

#################
# exercise 2.2
#################
J,z1,z2,z3,z4 = computeEndEffectorJacobian(q)
# compare with Pinocchio
Jee = robot.frameJacobian(q, frame_ee, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
jacobian_diff = J - Jee
print("Direct Kinematics - ee Gometric Jacobian (6X4 matrix), differece with Pinocchio library:\n", jacobian_diff)

##################
# exercise 2.3
##################
J_a = g2a(J, T_0e)
print("Analytic Jacobian:\n", J_a)

##################
##exercise 2.4
##################
## desired task space position
p = np.array([-0.5, -0.2, 0.5, math.pi/3])
# outside of workspace, gets the solution with minumum error
p = np.array([-1.0, -0.2, 0.5, math.pi/3])

# initial guess (elbow up)
q_i  = np.array([ 0.5, -1.0, -0.8, -math.pi])
# initial guess (elbow down)
#q_i  = np.array([ -0.5, 1.0, -0.8, -math.pi])
# initial guess (bad initialization)
#q_i  = np.array([ -5.0, 5.0, -0.8, -math.pi])

# solution of the numerical ik
q_f, log_err, log_grad = ik(p, q_i, line_search = True)
# compare solution with values obtained through direct kinematics
T_01, T_02, T_03, T_04, T_0e = directKinematics(q_f)
rpy = math_utils.rot2eul(T_0e[:3,:3])
task_diff = p - np.hstack((T_0e[:3,3],rpy[0]))


print("Desired End effector \n", p)
print("Point obtained with IK solution \n", np.hstack((T_0e[:3, 3], rpy[0])))
print("Error at the end-effector: \n", np.linalg.norm(task_diff))
print("Final joint positions\n", q_f)

# Plots
plt.subplot(2, 1, 1)
plt.ylabel("err")
plt.semilogy(log_err, linestyle='-', color='blue')
plt.grid()
plt.subplot(2, 1, 2)
plt.ylabel("grad")
plt.xlabel("number of iterations")
plt.semilogy(log_grad, linestyle='-', color='blue')
plt.grid()


ros_pub.add_marker(p)
ros_pub.publish(robot, q_i)
ros.sleep(5.0)
ros_pub.publish(robot, q_f)



#######################################
##exercise 2.5: polynomial trajectory
#########################################
# while np.count_nonzero(q - q_f) :
#
#     # Polynomial trajectory
#     for i in range(4):
#         a = coeffTraj(3,conf.q0[i],q_f[i])
#         qdd[i] = 2*a[2] + 6*a[3]*time + 12*a[4]*time**2 + 20*a[5]*time**3
#         qd[i] = a[1] + 2*a[2]*time + 3*a[3]*time**2 + 4*a[4]*time**3 + 5*a[5]*time**4
#         q[i] = a[0] + a[1]*time + a[2]*time**2 + a[3]*time**3 + a[4]*time**4 + a[5]*time**5
#
#     # Log Data into a vector
#     time_log = np.append(time_log, time)
#     q_log = np.vstack((q_log, q ))
#     q_des_log= np.vstack((q_des_log, q_des))
#     qd_log= np.vstack((qd_log, qd))
#     qd_des_log= np.vstack((qd_des_log, qd_des))
#     qdd_log= np.vstack((qdd_log, qdd))
#     qdd_des_log= np.vstack((qdd_des_log, qdd_des))
#     # tau_log = np.vstack((tau_log, tau))
#
#     # update time
#     time = time + conf.dt
#     #publish joint variables
#     ros_pub.publish(robot, q, qd)
#     ros_pub.add_marker(p)
#     ros.sleep(conf.dt*conf.SLOW_FACTOR)
#
#     # stops the while loop if  you prematurely hit CTRL+C
#     if ros_pub.isShuttingDown():
#         print ("Shutting Down")
#         break


ros_pub.deregister_node()  
plt.show(block=True)








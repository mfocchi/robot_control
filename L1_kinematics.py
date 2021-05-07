#common stuff 
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm
import eigenpy

import os
from base_controller.utils.common_functions import *
from base_controller.utils.optimTools import quadprog_solve_qp
from base_controller.utils.ros_publish_ur4 import RosPub

import ex_1_conf_kin as conf

#instantiate graphic utils
ros_pub = RosPub()
robot = getRobotModel4()


# Init variables
zero = np.array([0.0, 0.0, 0.0, 0.0])
time = 0.0

two_pi_f             = 2*np.pi*conf.freq   # frequency (time 2 PI)
two_pi_f_amp         = np.multiply(two_pi_f, conf.amp) 
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

# Init loggers
q_log = np.empty((4))*nan
qd_log = np.empty((4))*nan
qdd_log = np.empty((4))*nan
x_log = np.empty((3,0))*nan
time_log =  np.empty((0,0))*nan



# EXERCISE 9: 
conf.qd0 = two_pi_f_amp

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

# CONTROL LOOP
while True:          
 
    # Decimate print of time
    #if (divmod(time ,1.0)[1]  == 0):
       #print('Time %.3f s'%(time))
    if time >= conf.exp_duration:
        break
       

   
    x = robot.framePlacement(q, frame_ee).translation 
    o = robot.framePlacement(q, frame_ee).rotation
				    # compute jacobian of the end effector (in the WF)        
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    # take first 3 rows of J6 cause we have a point contact            
    J = J6[:3,:] 

    print("frame ee:\n", o)
    print("End-effector Jacobian:\n", J6)
       
    # Forward Euler Integration    
    # qd[2] = 0.01*math.sin(time)
    # qd = qd + 0*conf.dt    
    # q = q + conf.dt*qd  + 0.5*conf.dt*conf.dt*0

    

    # Log Data into a vector
    time_log = np.append(time_log, time)	
    q_log = np.vstack((q_log, q ))
    qd_log= np.vstack((qd_log, qd))            
 
    # update time
    time = time + conf.dt                  
    #publish joint variables
    ros_pub.publish(robot, q, qd)                   
    tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    # stops the while loop if  you prematurely hit CTRL+C                    
    if ros_pub.isShuttingDown():
        print ("Shutting Down")                    
        break;
            
ros_pub.deregister_node()
        
                 
                
# plot joint variables                                                                              
# plotJoint('position', 0, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('velocity', 1, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('acceleration', 2, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('torque', 3, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)






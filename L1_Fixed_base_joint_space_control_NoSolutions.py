#common stuff 
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm
import eigenpy
eigenpy.switchToNumpyMatrix()
import os
from utils.common_functions import *
from utils.optimTools import quadprog_solve_qp
from ros_publish import RosPub

import ex_1_conf as conf

#instantiate graphic utils
ros_pub = RosPub()
robot = importDisplayModel(False, False)


# Init variables
zero = np.matrix([0.0, 0.0,0.0, 0.0, 0.0, 0.0]).T
time = 0.0

two_pi_f             = 2*np.pi*conf.freq   #omega (frequency times 2*PI)
two_pi_f_amp         = np.multiply(two_pi_f, conf.amp) 
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

# Init loggers
q_log = np.empty((6,0))*nan
q_des_log = np.empty((6,0))*nan
qd_log = np.empty((6,0))*nan
qd_des_log = np.empty((6,0))*nan
qdd_log = np.empty((6,0))*nan
qdd_des_log = np.empty((6,0))*nan
tau_log = np.empty((6,0))*nan
f_log = np.empty((3,0))*nan
x_log = np.empty((3,0))*nan
time_log =  np.array([])


# EXERCISE 9: 
#...

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des  = conf.q0        # joint reference velocity
qd_des = zero        # joint reference acceleration
qdd_des = zero        # joint desired acceleration

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

# CONTROL LOOP
while True:
    # EXERCISE 1: Sinusoidal reference Generation
    #...

    # EXERCISE 2: Step reference Generation
    #...

    #Stops the simulation
    if time >= conf.exp_duration:
        break         

    #Compute robot dynamics matrixes  
    robot.computeAllTerms(q, qd) 

    #Get joint space inertia matrix                
    M = robot.mass(q, False)
    #Get Coriolis term               
    h = robot.nle(q, qd, False)
    #Get gravity terms                
    g = robot.gravity(q)
				
        
    # EXERCISE  5: PD control critical damping
    #...
                                
    #CONTROLLERS 
                                   
    # Exercise 3:  PD control
    # tau = ...
    
    # Exercise 6: PD control + Gravity Compensation
    #...
    
    # Exercise 7: PD + gravity + Feed-Forward term
    #...

    # EXERCISE 8_ Inverse Dynamics
    #...
   
    # Add external force if any (EXERCISE 11)
    if conf.EXTERNAL_FORCE  and time>2.0:
     #tau =...
    
     # (for plotting purposes) compute frame end effector position and velocity in the WF   
     x = robot.framePlacement(q, frame_ee).translation    
     ros_pub.add_arrow(x.A1.tolist(),conf.extForce/100) 
    
    # ------------------ SIMULATION ---------------
    #SIMULATION of the forward dynamics    
    M_inv = np.linalg.inv(M)  
    qdd = M_inv*(tau-h)    
    
    # Forward Euler Integration    
    qd = qd + qdd*conf.dt
    q = q + qd*conf.dt + 0.5*conf.dt*conf.dt*qdd
    
    # Log Data into a vector
    time_log = np.hstack((time_log, time))				
    q_log = np.hstack((q_log, q ))
    q_des_log= np.hstack((q_des_log, q_des))
    qd_log= np.hstack((qd_log, qd))
    qd_des_log= np.hstack((qd_des_log, qd_des))
    qdd_log= np.hstack((qdd_log, qd))
    qdd_des_log= np.hstack((qdd_des_log, qdd_des))
    tau_log = np.hstack((tau_log, tau))                
 
    # update time
    time = time + conf.dt                  
                
    #publish joint variables
    ros_pub.publish(robot, q, qd, tau)                   
    tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    # stops the while loop if  you prematurely hit CTRL+C                    
    if ros_pub.isShuttingDown():
        print ("Shutting Down")                    
        break;
            
ros_pub.deregister_node()
                
                
# plot joint variables                                                                              
plotJoint('position', 0, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('velocity', 1, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('acceleration', 2, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('torque', 3, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)






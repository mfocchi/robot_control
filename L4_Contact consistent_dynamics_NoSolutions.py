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


import ex_4_conf as conf
    
#instantiate graphic utils
ros_pub = RosPub()
robot = importDisplayModel(False, False)

# Init variables
zero = np.matrix([0.0, 0.0,0.0, 0.0, 0.0, 0.0]).T
time = 0.0

two_pi_f             = 2*np.pi*conf.freq   # frequency (time 2 PI)
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

z_gnd = 0.0
ground_contact = False
counter_lo = 0
counter_td=0

#init nullspace projector
N = np.zeros((6,6))

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

    # EXERCISE 1: Generate sinusoidal reference for Shoulder lift joint 
    # ...

    # EXERCISE 6: Constraint consistent joint reference
    # ...
				
    # Set constant reference after a while
    if time >= conf.exp_duration_sin:
        q_des  = conf.q0
        qd_des = zero
        qdd_des = zero         
 
    # Decimate print of time
    #if (divmod(time ,1.0)[1]  == 0):
       #print('Time %.3f s'%(time))
    if time >= conf.exp_duration:
        break
                            
    robot.computeAllTerms(q, qd) 
    # joint space inertia matrix				
    M = robot.mass(q, False)
    # bias terms				
    h = robot.nle(q, qd, False)
    #gravity terms				
    g = robot.gravity(q)
				
    #CONTROLLER				
    #PD controller + gravity compensation
    tau = conf.kp*(q_des-q) + conf.kd*(qd_des-qd) + g              

    # compute jacobian of the end effector (in the WF)
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    # take first 3 rows of J6 cause we have a point contact            
    J = J6[:3,:] 
    # compute the derivativeof the jacobian (in the WF)			
    dJdq = robot.frameClassicAcceleration(q, qd, None, frame_ee).linear    
    # (for plotting purposes) compute frame end effector position and velocity in the WF   
    x = robot.framePlacement(q, frame_ee).translation                
    # compute  twist at the end-effector 
    v_frame = robot.frameVelocity(q, qd, frame_ee, False)   
    xd = v_frame.linear
#    print (J*qd     - v_frame.linear)
            
    # EXERCISE 2: Compute Constraint Consistent Dynamics with  contact at the end-effector 												
    # ...
                
    # touch-down
    if (counter_lo==0) and not (ground_contact) and (x[2]<=z_gnd):                         
        counter_td = 30     
        ground_contact = True
                                
        # EXERCISE 3: update the joint velocity after inlastic  impact with null-space projector for velocities                               
        #...
                           
        #recompute the velocity dependent terms...                                
        robot.computeAllTerms(q, qd)    
        h = robot.nle(q, qd, False)  
        dJdq = robot.frameClassicAcceleration(q, qd, None, frame_ee, False).linear 
        print("contact ON")  

    # lift-off  
    if ((counter_td==0) and(ground_contact) and (f[2]<=0.0)):#(x[2]>z_gnd)):  
        #hysteresis
        counter_lo = 200                
        ground_contact = False;    
        print ("contact OFF")
                                
    if counter_td > 0:
        counter_td =counter_td-1
    if counter_lo > 0:
        counter_lo =counter_lo-1                                                                
                            
    # compute forward (unconstraint) dynamics     
    qdd_uc = M_inv*(tau - h)              

    if (not ground_contact):                    
        # qdd = ...                                
        # f = ...
    else:  
        #qdd = ...
        # compute contact force    
        #...
                       
        # EXERCISE 5: Check that contact force disappears in projected dynamics
        # ...
                             
    # EXERCISE 4: Simulation of the constraint consistent dynamics      
    qd = qd + qdd*conf.dt
    q = q + qd*conf.dt + 0.5*conf.dt*conf.dt*qdd

    # EXERCISE 7: Gauss principle of least constraint holds when in contact
    # Minimize     1/2 x^T M x - q_ucT M x
    # Subject to   J x = -Jdqd 
    # ...

    # EXERCISE 8: check the shifting law 
    #...

    # Log Data into a vector
    time_log = np.hstack((time_log, time))				
    q_log = np.hstack((q_log, q ))
    q_des_log= np.hstack((q_des_log, q_des))
    qd_log= np.hstack((qd_log, qd))
    qd_des_log= np.hstack((qd_des_log, qd_des))
    qdd_log= np.hstack((qdd_log, qd))
    qdd_des_log= np.hstack((qdd_des_log, qdd_des))
    tau_log = np.hstack((tau_log, tau))            		
    f_log = np.hstack((f_log, f))				
    x_log = np.hstack((x_log, x))				

    # update time
    time = time + conf.dt 

    # plot contact force
    # scale f for plotting purposes                
    scaled_f = f/100                 
    ros_pub.add_arrow(x.A1.tolist(), scaled_f.A1.tolist()) 

    # plot ball at the end-effector
    ros_pub.add_marker(x.A1.tolist())                 
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

# plot end-effector variables    
#plotEndeff('position', 4,time_log, x_log)
plotEndeff('force', 5, time_log, x_log, None, None, None, None,None, f_log)



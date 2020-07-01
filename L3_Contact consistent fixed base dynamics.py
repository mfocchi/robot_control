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


import ex_3_conf as conf
    
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

    # Reference Generation
    q_des  = conf.q0 +   np.multiply( conf.amp,np.sin(two_pi_f*time + conf.phi))
    qd_des = np.multiply(two_pi_f_amp , np.cos(two_pi_f*time + conf.phi))
    qdd_des = np.multiply( two_pi_f_squared_amp , -np.sin(two_pi_f*time + conf.phi))

    # EXERCISE 3: Constraint consistent joint reference
#    amplitude =   np.array([ 0.0, 1.2, 0.0, 0.0, 0.0, 0.0])
#    qd_des =  np.matrix([ 0.0,  amplitude[1]*w_rad[1]*np.cos(w_rad[1]*time), 0.0, 0.0 , amplitude[4]*w_rad[4]*np.cos(w_rad[4]*time), 0.0]).T
#    qdd_des = np.matrix([ 0.0, -amplitude[1]*w_rad[1]*w_rad[1]*np.sin(w_rad[1]*time), 0.0, 0.0, -amplitude[4]*w_rad[4]*w_rad[4]*np.sin(w_rad[4]*time), 0.0]).T  
#    if (ground_contact):    
#        qd_des = N*qd_des
#        qdd_des =  N*qdd_des
#    q_des += qd_des*conf.dt	
				
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



    # EXERCISE 1: contact at the end-effector 
    # compute jacobian of the end effector (in the WF)
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    # take first 3 rows of J6 cause we have a point contact            
    J = J6[:3,:] 
    # compute the derivativeof the jacobian (in the WF)			
    dJdq = robot.frameClassicAcceleration(q, qd, None, frame_ee).linear    
    # (for plotting purposes) compute frame end effector position and velocity in the WF   
    x = robot.framePlacement(q, frame_ee).translation                
    xd = J*qd    
    # check twist at the end-effector			
    v_frame = robot.frameVelocity(q, qd, frame_ee, False)   
#    print (J*qd     - v_frame.linear)
            
                    
    # EXERCISE 1: Simulation of a constraint consistent dynamics 
    M_inv = np.linalg.inv(M)
                
    #Moore-penrose pseudoinverse of A = J^T => (A^TA)^-1 * A^T
    #JTpinv = np.linalg.inv(J*J.T)*J             
   
    #lambda_ contact inertia
    lambda_= np.linalg.inv(J*M_inv* J.T + np.eye(3)*1e-09) 
    #dynamicaaly consistent psdinv
    JTpinv = lambda_*J*M_inv
    #null-space projector of J^T    
    Nt = (np.eye(6)- J.T * JTpinv) 
                
    # touch-down
    if (counter_lo==0) and not (ground_contact) and (x[2]<=z_gnd):                         
        counter_td = 30     
        ground_contact = True
                                
        #EXERCISE 1 :compute null-space for velocities to update the joint velocity accounting  for the impact                                
        Jpinv = M_inv * J.T * lambda_                
        N = np.eye(6) - Jpinv*J                                        
        qd = N*qd                            
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
        qdd = qdd_uc                                
        f = np.matrix([0.0, 0.0,0.0]).T                                
    else:  
        qdd = M_inv*( Nt*(tau -    h) - J.T*    lambda_*dJdq)    
        # compute contact force    
        f = lambda_*( -dJdq + J*M_inv*(h - tau) )
                               
        ## EXERCISE 2: Check that contact force disappears in projected dynamics
        #print (N_dyn* J.T*f).T                    
        #torques in row space are almost zero if there are not internal motions                                                                                                                            
        
        #torques_row_space =  Nt*(tau   -    h)    
        #print  torques_row_space    
                             
    # EXERCISE 1: Forward Euler Integration     
    qd = qd + qdd*conf.dt
    q = q + qd*conf.dt + 0.5*conf.dt*conf.dt*qdd


    ##  EXERCISE 4: Gauss principle of least constraint holds when in contact
    # Minimize     1/2 x^T M x - q_ucT M x
    # Subject to   J x = -Jdqd 
#    A = J
#    b = -dJdq	
#    G = M
#    g = -qdd_uc.T*M	
#    qdd_check = quadprog_solve_qp(G, g, None, None, A, b)                                
##        
#    print("proj - gauss")                     
#    print( qdd.A1 - qdd_check)
#    
				

#    #EXERCISE 6: check the shifting law 
#    frame_ee = robot.model.getFrameId('ee_link')     
#    #get the frame of the supporting parent joint (wrist_3_joint)
#    frame_o = robot.model.frames[frame_ee].parent
#    #print(robot.model.names[frame_o]	)															
#                      
#    # find transform from frame_ee to frame_o				
#    iMf = robot.model.frames[frame_ee].placement
#    #rotation matrix from frame_ee to  frame_o (columns are the axis of frame_ee expressed in frame_o )
#    o_R_ee = iMf.rotation
#    #vector from frame_o to frame_ee expressed in frame_o
#    o_t = iMf.translation		
#    #vector from frame_ee to frame_o epxressed in frame_ee
#    ee_t = -o_R_ee.T * o_t				
#				
#    #--------------------------------------------------------------------------------------------------------
#    # compute the motion tranform from frame_o to frame_ee (we  will use this matrix if I work with 6D vectors)
#    #--------------------------------------------------------------------------------------------------------
#    o_X_ee = np.zeros((6, 6))				
#    o_X_ee[:3,:3] = o_R_ee
#    o_X_ee[3:,3:] = o_R_ee
#    o_X_ee[:3,3:] = -o_R_ee*pin.skew(ee_t)	
#			
#    # let's do the same with Pinocchio native functions
#    # compute the motion tranform from frame_ee to frame_o
#    o_X_ee_pin = pin.SE3(iMf.rotation,  iMf.translation) 			
#    #check they are the same 
#    print "TEST1:\n",  o_X_ee - o_X_ee_pin.toActionMatrix()
#    print "TEST2:\n",  o_X_ee - iMf.toActionMatrix()
#			
#    #--------------------------------------------------------------------------------------------------------					
#    #let's compare the twists (for joints you should use robot.velocity or robot.data.v not frameVelocity)				
#    #--------------------------------------------------------------------------------------------------------
#    v_o = robot.velocity(q, qd, frame_o, True, pin.ReferenceFrame.LOCAL)	#analog to     v_o = robot.data.v[robot.model.frames[frame_ee].parent]				
#    v_ee = robot.frameVelocity(q, qd, frame_ee, True, pin.ReferenceFrame.LOCAL)
#					
#    #compute the twist at the end-effector trough the shifting law (note you need to use the inverse of o_X_ee_pin)
#    #using pinocchio native function
#    v_ee_test3 = o_X_ee_pin.actInv(v_o)  
#    print "TEST3:\n",v_ee_test3 - v_ee
#				
#    #doing with 6D vectors				
#    v_ee_test4 =  np.linalg.inv(o_X_ee)*v_o.vector
#    print "TEST4:\n", v_ee_test4 - v_ee				
#			
#    #-------------------------------------------------------------------------------------------------------- 
#    #let's compare the jacobians	
#    #--------------------------------------------------------------------------------------------------------
#    Je = robot.frameJacobian(q, frame_ee, True, pin.ReferenceFrame.LOCAL)       
#    robot.computeJointJacobians(q)         
#    Jo = robot.getJointJacobian(frame_o,  pin.ReferenceFrame.LOCAL)  
#    print "TEST5:\n", Je  - o_X_ee_pin.toActionMatrixInverse()*Jo				

#------------------------------------------------------------------------------------------------------------
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
plotEndeff('position', 4,time_log, x_log)
#plotEndeff('force', 5, time_log, x_log, f_log)



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

import ex_2_conf as conf

#instantiate graphic utils
ros_pub = RosPub()
robot = importDisplayModel(False, False)


# Init variables
zero = np.matrix([0.0, 0.0,0.0, 0.0, 0.0, 0.0]).T
zero_cart = np.matrix([ 0.0, 0.0,0.0]).T
time = 0.0

two_pi_f             = 2*np.pi*conf.freq   # frequency (time 2 PI)
two_pi_f_amp         = np.multiply(two_pi_f, conf.amp) 
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

# Init loggers
x_log = np.empty((3,0))*nan
x_des_log = np.empty((3,0))*nan
xd_log = np.empty((3,0))*nan
xd_des_log = np.empty((3,0))*nan
xdd_log = np.empty((3,0))*nan
xdd_des_log = np.empty((3,0))*nan
tau_log = np.empty((6,0))*nan
time_log =  np.array([])

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des  = conf.q0        # joint reference velocity
qd_des = zero        # joint reference acceleration
qdd_des = zero        # joint desired acceleration

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

#end effector ID
jid = robot.model.getJointId('wrist_3_joint')

# compute initial end effector position and velocity
x0 = robot.placement(q, jid, True).translation + np.matrix([0.0, 0.0, 0.0]).T
print q
pin.computeAllTerms(robot.model, robot.data, q, qd)
x0 = robot.data.oMi[jid].translation + np.matrix([0.0, 0.0, 0.0]).T
print x0
xd0 = np.matrix([ 0.0, 0.0, 0.0]).T
xdd0 = np.matrix([ 0.0, 0.0, 0.0]).T
x = x0
xd = xd0
xdd = xdd0
x_des = x0
xd_des = zero_cart
xdd_des = zero_cart

# CONTROL LOOP
while True:
    
    # EXERCISE 1: Sinusoidal reference for end effector   
    x_des  = x0  + np.multiply( conf.amp, np.sin(two_pi_f*time + conf.phi))
    xd_des = np.multiply(two_pi_f_amp , np.cos(two_pi_f*time + conf.phi))
    xdd_des = np.multiply( two_pi_f_squared_amp , -np.sin(two_pi_f*time + conf.phi))
    # Set constant reference after a while
    if time >= conf.exp_duration_sin:
        x_des  = x0
        xd_des = xd0
        xdd_des = xdd0
        
#     # EXERCISE 2: Step reference
#    if time > 2.0:
#        x_des = x0 + np.matrix([ 0.0, 0.0, 0.1]).T 
#        xd_des =  zero_cart
#        xdd_des = zero_cart 
#    else:
#        x_des = x0
#        xd_des =  zero_cart
#        xdd_des = zero_cart 

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
    
       
    #compute Jacobian and its derivative in the world frame  
    pin.computeJointJacobians(robot.model, robot.data, q)
    robot.computeJointJacobians(q)
    #get local jacobian
    J_i = robot.getJointJacobian(jid)[:3,:]
    R = robot.placement(q, jid).rotation
    #map jacobian to world frame
    J = R * J_i    
    pin.computeJointJacobiansTimeVariation(robot.model, robot.data, q, qd)
    J_i_dot=pin.getJointJacobianTimeVariation(robot.model, robot.data, jid, pin.ReferenceFrame.LOCAL)[:3,:]
    Jdot = R * J_i_dot
    
    # compute end effector position and velocity    
    x = robot.placement(q, jid).translation
    xd = J*qd
 


        
    M_inv = np.linalg.inv(M)
    
    #    #pseudoinverse of J^T = (A^TA)^-1 * A^T
    JTpinv = np.linalg.inv(J*J.T)*J
 
    #lambda_xdd+u=F
    lambda_= np.linalg.inv(J*M_inv* J.T)
    
    #dynamicaaly consistent psdinv
    JTpinv_dyn = lambda_*J*M_inv
    N_dyn = (eye(6)-J.T*JTpinv_dyn) 
    
    u_dyn =   -lambda_*(Jdot*qd)   + JTpinv_dyn*h
#    # EXERCISE 4: 
#    # compute virtual force
#    F_des = conf.Kx*(x_des-x)+conf.Dx*(xd_des-xd)
#    tau = J.T*F_des  
    
#    # EXERCISE 5:     
#    F_des = conf.Kx*(x_des-x)+conf.Dx*(xd_des-xd) 
#    tau = J.T*F_des + -g  5*qd
        
#    # EXERCISE 6:
#    F_des = lambda_* xdd_des + conf.Kx*(x_des-x)+conf.Dx*(xd_des-xd) 
#    tau = J.T*F_des + g - 5*qd
       
    #Null space projector
    N = (eye(6)-J.T*JTpinv)  
    # null space torques
    tau0 = 100*(conf.q0-q) - 10*qd
    tau_null = N*tau0
    F_des = xdd_des+conf.Kx*(x_des-x)+conf.Dx*(xd_des-xd)
    u = -lambda_*(Jdot*qd)   + JTpinv*h 


    # EXERCISE 7:
    tau = J.T*(lambda_*F_des + u) + tau_null    
#    
      # EXERCISE 8:
##     OSID with bias compensation in joint space (simpler to compute)
#    tau = h + J.T*(lambda_*F_des) + tau_null

    #EXERCISE 12: dyn consisten pseudoninverse
#    tau_null = N_dyn*tau0
#    tau = J.T*(lambda_*F_des + u_dyn) + tau_null   
    
    # EXERCISE 10:
    #compute total jacibian
#    J_lin = robot.getJointJacobian(jid)[:3,:]
#    J_ang = robot.getJointJacobian(jid)[3:6,:]
#    #map jacobian to world frame
#    Jt = np.matrix(np.zeros((6, 6)))
#    Jt[:3,:]= R * J_lin  
#    Jt[3:6,:]= R * J_ang  
#    twist = Jt*qd
#    Ktheta=eye(3)
#    Ktheta[0,0] = 1000
#    Ktheta[1,1] = 1000
#    Ktheta[2,2] = 1000
#    
#    Dtheta=eye(3)
#    Dtheta[0,0] = 100
#    Dtheta[1,1] = 100
#    Dtheta[2,2] = 100
#    
#    omega_des = np.matrix([0,0,0]).T
#    omega = twist[3:6]
#    #pinocchio default rotation matric: va = R * Vb, so I have to change the sign in r     
#    cos_theta = (R[0,0]+ R[1,1]+ R[2,2]-1)/2
#    theta = np.arccos( cos_theta) 
#    r_hat = 1/(2*sin(theta))*np.matrix([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]).T 
#    
#    e_o = theta* r_hat
#    M_des = - Ktheta* e_o + Dtheta*(omega_des - omega)
#    W_des = np.vstack([F_des, M_des])
#    #FFWD?
#    tau = Jt.T*W_des      
    

    #SIMULATION
    
#    EXERCISE 9
    if conf.EXTERNAL_FORCE  and time>2.0:
     tau += J.transpose()*conf.extForce
     # (for plotting purposes) compute frame end effector position and velocity in the WF   
     x = robot.framePlacement(q, frame_ee).translation    
     ros_pub.add_arrow(x.A1.tolist(),conf.extForce/100) 					
    
    
    #SIMULATION of the forward dynamics    

    qdd = M_inv*(tau-h)    
    
    # Forward Euler Integration    
    qd = qd + qdd*conf.dt
    q = q + qd*conf.dt + 0.5*conf.dt*conf.dt*qdd
    
    
    # Log Data into a vector
    time_log = np.hstack((time_log, time))				
    x_log = np.hstack((x_log, x ))
    x_des_log= np.hstack((x_des_log, x_des))
    xd_log= np.hstack((xd_log, xd))
    xd_des_log= np.hstack((xd_des_log, xd_des))
    xdd_log= np.hstack((xdd_log, xdd))
    xdd_des_log= np.hstack((xdd_des_log, xdd_des))
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
        

#plot position
plotEndeff('position', 1,time_log, x_log, x_des_log)
plotEndeff('velocity', 2,time_log, None, None, xd_log, xd_des_log)

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
from base_controller.utils.ros_publish import RosPub
from base_controller.utils.math_tools import Math
import ex_3_conf as conf

#instantiate graphic utils
os.system("killall rosmaster")
ros_pub = RosPub()
robot = getRobotModel("ur5")

math = Math()
# Init variables
zero = np.array([0.0, 0.0,0.0, 0.0, 0.0, 0.0])
zero_cart = np.array([ 0.0, 0.0,0.0])
time = 0.0

two_pi_f             = 2*np.pi*conf.freq   # frequency (time 2 PI)
two_pi_f_amp         = np.multiply(two_pi_f, conf.amp) 
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

# Init loggers
x_log = np.empty((3))*nan
x_des_log = np.empty((3))*nan
xd_log = np.empty((3))*nan
xd_des_log = np.empty((3))*nan
xdd_log = np.empty((3))*nan
xdd_des_log = np.empty((3))*nan
euler_log = np.empty((3))*nan
euler_des_log = np.empty((3))*nan
tau_log = np.empty((6))*nan
time_log =  np.empty((0,0))*nan

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des  = conf.q0        # joint reference velocity
qd_des = zero        # joint reference acceleration
qdd_des = zero        # joint desired acceleration

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

# compute initial end effector position and velocity
x0 = robot.framePlacement(q, frame_ee, True).translation + np.array([0.0, 0.0, 0.0])
xd0 = np.array([ 0.0, 0.0, 0.0])
# to avoid having tracking errors at the start
#xd0 = two_pi_f_amp
xdd0 = np.array([ 0.0, 0.0, 0.0])
x = x0
xd = xd0
xdd = xdd0
x_des = x0
xd_des = zero_cart
xdd_des = zero_cart
euler = zero_cart
euler_des = zero_cart

# CONTROL LOOP
while True:
    
    # EXERCISE 1: Sinusoidal reference generation for end effector   
    x_des  = x0  + np.multiply( conf.amp, np.sin(two_pi_f*time + conf.phi))
    xd_des = np.multiply(two_pi_f_amp , np.cos(two_pi_f*time + conf.phi))
    xdd_des = np.multiply( two_pi_f_squared_amp , -np.sin(two_pi_f*time + conf.phi))
    # Set constant reference after a while
    if time >= conf.exp_duration_sin:
        x_des  = x0
        xd_des = xd0
        xdd_des = xdd0
        
     # EXERCISE 2: Step reference generation  for end effector 
#    if time > 2.0:
#        x_des = x0 + np.array([ 0.0, 0.0, 0.1])
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
    
    # compute jacobian of the end effector (in the WF)        
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    # take first 3 rows of J6 cause we have a point contact            
    J = J6[:3,:] 
    # compute  the end-effector acceleration due to joint velocity            
    dJdq = robot.frameClassicAcceleration(q, qd, None, frame_ee).linear    
    # compute frame end effector position and velocity in the WF   
    x = robot.framePlacement(q, frame_ee).translation                      
    xd = J.dot(qd)  

    M_inv = np.linalg.inv(M)
    # Moore-penrose pseudoinverse  A^# = (A^TA)^-1 * A^T with A = J^T
    JTpinv = np.linalg.inv(J.dot(J.T)).dot(J)
    lambda_= np.linalg.inv(J.dot(M_inv).dot(J.T))  #better this if J is not square  
    #lambda_ = np.linalg.pinv(J.T)*M*np.linalg.pinv(J)

    
    #Null space projector I - (JTpinv )^-1 * JTpinv => I  - JT *JTpiv
    N = (eye(6)-J.T.dot(JTpinv))
  				
    # null space torques (postural task)
    tau0 = 50*(conf.q0-q) - 10*qd
    tau_null = N.dot(tau0)
   	
    # EXERCISE 4: PD control (cartesian task)									
    F_des = conf.Kx.dot(x_des-x) + conf.Dx.dot(xd_des-xd)
    tau = (J.T).dot(F_des)     			
    tau += tau_null 
    
    # EXERCISE 5: PD control + Gravity Compensation:
#    F_des = conf.Kx.dot(x_des-x) + conf.Dx.dot(xd_des-xd)	+ JTpinv.dot(g)		
#    tau = J.T.dot(F_des) + tau_null 
        
    # EXERCISE 6: PD control  + Gravity Compensation + Feed-Forward term
#    F_des = lambda_.dot(xdd_des) + conf.Kx.dot(x_des-x)+conf.Dx.dot(xd_des-xd) + JTpinv.dot(g)
#    tau = J.T.dot(F_des) + tau_null 
     
    # EXERCISE 7: Operational space inverse dynamics
#    F_des = xdd_des + conf.Kx.dot(x_des-x)+conf.Dx.dot(xd_des-xd)
#    u = -lambda_.dot(dJdq)  + JTpinv.dot(h) 
#    tau = J.T.dot(lambda_.dot(F_des) + u) + tau_null    
    
     # EXERCISE 8: OSID with bias compensation in joint space (simpler to compute)
#    F_des = xdd_des + conf.Kx.dot(x_des-x)+conf.Dx.dot(xd_des-xd)    
#    tau =  J.T.dot(lambda_.dot(F_des)) + h + tau_null

     # dyn consistent pseudo-inverse (TODO CHECK )
#    JTpinv_dyn = (lambda_.dot(J)).dot(M_inv)
#    N_dyn = (eye(6)-J.T.dot(JTpinv_dyn))    
#    u =   - lambda_.dot(dJdq)   + JTpinv_dyn.dot(h    )
#    tau_null = N_dyn*tau0
#    tau = J.T.dot(lambda_.dot(F_des) + u) + tau_null   
    
    # EXERCISE 10: Control of orientation
#    # actual end-effector orientation (columns are the axis of frame_ee expressed in WF (check rviz TF) )				
#    w_R_e = robot.framePlacement(q, frame_ee).rotation 
#    e_R_w = w_R_e.T				
#    #compute actual end-effector twist
#    twist = J6.dot(qd)
#    # extract omega				
#    omega = twist[3:6]
#    #des orientation  (horizontal with X pointing left ) NB the axis are the rows of the matrix 
#    des_x_axis = np.array([0, 1, 0])
#    des_y_axis = np.array([0, 0, -1])
#    des_z_axis = np.array([-1, 0, 0])			
#    des_R_w = np.vstack((des_x_axis, des_y_axis, des_z_axis))
#    # desired angular velocity	(constant)			            
#    omega_des = np.array([0,0,0])
#    # compute  the orientation error Rotation matrix
#    des_R_e = des_R_w * e_R_w.T
#    # compute the angle-axis representation of the orientation error				
#    cos_theta = (des_R_e[0,0]+ des_R_e[1,1]+ des_R_e[2,2]-1)/2
#    delta_theta = np.arccos( cos_theta) 
#    r_hat = 1/(2*np.sin(delta_theta))*np.array([des_R_e[2,1]-des_R_e[1,2], des_R_e[0,2]-des_R_e[2,0], des_R_e[1,0]-des_R_e[0,1]])    
#    e_error_o = delta_theta * r_hat #the error is in the endeffector frame
#    # we need to map it in the world frame to compute the wrench because the jacobian is in the WF
#    w_error_o = w_R_e.dot(e_error_o)			  				
#    # compute the linear part of the wrench	
#    F_des = xdd_des + conf.Kx.dot(x_des-x)+conf.Dx.dot(xd_des-xd)  
#    # compute the angular part of the wrench				
#    Tau_des = - conf.Ktheta.dot(w_error_o) + conf.Dtheta.dot(omega_des - omega)
#    W_des = np.hstack([F_des, Tau_des])
#    tau = J6.T.dot(W_des)      
#    #to log 
#    euler = math.rotTorpy(e_R_w)				
#    euler_des = math.rotTorpy(des_R_w)
    
#    #EXERSISE 11 : full inv. dynamics
#    #compute lambda for both orientation and position
#    #lambda6_= np.linalg.inv(J6.dot(M_inv).dot( J6.T)  
#    lambda6_ = np.linalg.pinv(J6.T,0.0001).dot(M).dot(np.linalg.pinv(J6,0.001))
#    dJdq6 = robot.frameClassicAcceleration(q, qd, None, frame_ee).vector   
#    u6 = -lambda6_.dot(dJdq6)  + np.linalg.pinv(J6.T,0.0001).dot(h)    
#    tau = J6.T.dot(lambda6_.dot(W_des) + u6) 
    

				
#    EXERCISE 9: Add external force
    if conf.EXTERNAL_FORCE  and time>2.0:
        tau += J.transpose().dot(conf.extForce)
        ros_pub.add_arrow(x,conf.extForce/100)                    
    
    #SIMULATION of the forward dynamics    
    qdd = M_inv.dot(tau-h)    
    
    # Forward Euler Integration    
    qd = qd + qdd*conf.dt
    q = q + qd*conf.dt + 0.5*conf.dt*conf.dt*qdd
				
    
    # Log Data into a vector				
    time_log = np.append(time_log, time)	           
    x_log = np.vstack((x_log, x ))
    x_des_log= np.vstack((x_des_log, x_des))
    xd_log= np.vstack((xd_log, xd))
    xd_des_log= np.vstack((xd_des_log, xd_des))
    xdd_log= np.vstack((xdd_log, xdd))
    xdd_des_log= np.vstack((xdd_des_log, xdd_des))
    euler_log= np.vstack((euler_log, euler))
    euler_des_log= np.vstack((euler_des_log, euler_des))
    tau_log = np.vstack((tau_log, tau))                
 
    # update time
    time = time + conf.dt                  
    
    # plot ball at the end-effector
    ros_pub.add_marker(x)                   
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
plotEndeff('velocity', 2, time_log, x_log, x_des_log, xd_log, xd_des_log, euler_log, euler_des_log)
plotEndeff('orientation', 3,time_log, x_log, x_des_log, xd_log, xd_des_log, euler_log, euler_des_log)
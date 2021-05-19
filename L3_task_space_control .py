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
os.system("killall rosmaster rviz")
ros_pub = RosPub("ur5")
robot = getRobotModel("ur5")

math_utils = Math()
# Init variables
zero = np.array([0.0, 0.0,0.0, 0.0, 0.0, 0.0])
zero_cart = np.array([ 0.0, 0.0,0.0])
time = 0.0

two_pi_f             = 2*np.pi*conf.freq   # frequency (time 2 PI)
two_pi_f_amp         = np.multiply(two_pi_f, conf.amp) 
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

# Init loggers
p_log = np.empty((3))*nan
p_des_log = np.empty((3))*nan
pd_log = np.empty((3))*nan
pd_des_log = np.empty((3))*nan
pdd_des_log = np.empty((3))*nan
rpy_log = np.empty((3))*nan
rpy_des_log = np.empty((3))*nan
error_o_log = np.empty((3))*nan
tau_log = np.empty((6))*nan
time_log =  np.empty((0,0))*nan

rpy_old = np.zeros((3))
rpy_unwrapped = np.zeros((3))
rpy_des_old = np.zeros((3))
rpy_des_unwrapped = np.zeros((3))


q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des  = conf.q0        # joint reference velocity
qd_des = zero        # joint reference acceleration
qdd_des = zero        # joint desired acceleration

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

# compute initial end effector position and velocity from q0
p0 = robot.framePlacement(conf.q0, frame_ee, True).translation + np.array([0.0, 0.0, 0.0])
pd0 = np.array([ 0.0, 0.0, 0.0])
pdd0 = np.array([ 0.0, 0.0, 0.0])

p = p0
pd = pd0
pdd = pdd0
p_des = p0
pd_des = zero_cart
pdd_des = zero_cart
rpy = zero_cart
rpy_des = zero_cart
FirstTime = True

# CONTROL LOOP
while True:
    
    # EXERCISE 1.1: Sinusoidal reference generation for the end effector   
    p_des  = p0  + np.multiply( conf.amp, np.sin(two_pi_f*time + conf.phi))
    pd_des = np.multiply(two_pi_f_amp , np.cos(two_pi_f*time + conf.phi))
    pdd_des = np.multiply( two_pi_f_squared_amp , -np.sin(two_pi_f*time + conf.phi))
    # Set constant reference after a while
    if time >= conf.exp_duration_sin:
        p_des  = p0
        pd_des = pd0
        pdd_des = pdd0
        
    #  EXERCISE 1.2: Step reference generation for the end effector 
#    if time > 2.0:
#        p_des = p0 + np.array([ 0.0, 0.0, 0.1])
#        pd_des =  zero_cart
#        pdd_des = zero_cart 
#    else:
#        p_des = p0
#        pd_des =  zero_cart
#        pdd_des = zero_cart 

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
    
    # compute jacobian of the end effector in the world frame    
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    # take first 3 rows of J6 cause we have a point contact            
    J = J6[:3,:] 
    # compute  the end-effector acceleration due to joint velocity Jdot*qd         
    Jdqd = robot.frameClassicAcceleration(q, qd, None, frame_ee).linear    
    # compute frame end effector position and velocity in the WF   
    p = robot.framePlacement(q, frame_ee).translation  

    # with sine reference: to avoid having tracking errors in velocity at the initial point
#    if FirstTime:    
#        qd = J.T.dot(np.linalg.inv(J.dot(J.T))).dot(two_pi_f_amp)
#        FirstTime = False
                    
    pd = J.dot(qd)  


    M_inv = np.linalg.inv(M)     
    # Moore-penrose pseudoinverse  A^# = (A^TA)^-1 * A^T with A = J^T
    JTpinv = np.linalg.inv(J.dot(J.T)).dot(J)
    
    # joint space inertia matrix reflected at the end effector (J*M^-1*Jt)^-1
    lambda_= np.linalg.inv(J.dot(M_inv).dot(J.T))  # J should be full row rank  otherwise add a damping term
     
    #Null space projector I - (JTpinv )^-1 * JTpinv => I  - JT *JTpiv
    N = eye(6)-J.T.dot(JTpinv)

    # EXERCISE 1.4: PD control (cartesian task)  
    F_des = conf.Kx.dot(p_des-p) + conf.Dx.dot(pd_des-pd)
    tau = (J.T).dot(F_des)      				

    # EXERCISE 1.5: PD control (cartesian task) + postural task  
    # null space torques (postural task)
    tau0 = 50*(conf.q0-q) - 10*qd
    tau_null = N.dot(tau0)						
#    tau = (J.T).dot(F_des)  + tau_null 
    
    # EXERCISE 1.6: PD control + Gravity Compensation:
#    F_des = conf.Kx.dot(p_des-p) + conf.Dx.dot(pd_des-pd)	+ JTpinv.dot(g)		
#    tau = J.T.dot(F_des) + tau_null 
        
        
    # EXERCISE 1.7: PD control  + Gravity Compensation + Feed-Forward term
#    F_des = lambda_.dot(pdd_des) + conf.Kx.dot(p_des-p)+conf.Dx.dot(pd_des-pd) + JTpinv.dot(g)
#    tau = J.T.dot(F_des) + tau_null 
     
     
    # EXERCISE 2.1: Cartesian space inverse dynamics
#    F_des = pdd_des + conf.Kx.dot(p_des-p) + conf.Dx.dot(pd_des-pd)
#    mu = -lambda_.dot(Jdqd)  + JTpinv.dot(h) 
#    tau = J.T.dot(lambda_.dot(F_des) + mu) + tau_null    
    
    
     # EXERCISE 2.2: Cartesian space inverse dynamics with bias compensation in joint space (simpler to compute)
#    F_des = pdd_des + conf.Kx.dot(p_des-p) + conf.Dx.dot(pd_des-pd)    
#    tau =  J.T.dot(lambda_.dot(F_des)) + h + tau_null


     # EXERCISE 3.1: Control of orientation with PD
#    ORIENTATION_CONTROL = True
#    # get the actual end-effector orientation (columns are the axis of frame_ee expressed in WF (check rviz TF) )				
#    w_R_e = robot.framePlacement(q, frame_ee).rotation 		
#    #compute the actual end-effector twist (i.e. linear and angula velocity)
#    twist = J6.dot(qd)
#    # extract omega				
#    omega = twist[3:6]    
#    # compose the des orientation rotation matrix (x axis along x, y axis along -y, z axis along -z) NB the axis are the columns of the matrix w_R_des
#    des_x_axis = np.array([1, 0, 0])
#    des_y_axis = np.array([0, -1, 0])
#    des_z_axis = np.array([0, 0 , -1 ])			
#    w_R_des = np.vstack((des_x_axis.T, des_y_axis.T, des_z_axis.T))
#    # desired angular velocity	(constant)			            
#    omega_des = np.array([0,0,0])
    
    
    
    # EXERCISE 3.2 - Control of orientation with PD: singularity
    # rpy_des = np.array([0.8, math.pi/2 , 0.0])) # singularity
    # rpy_des = np.array([0.1, 0.2, 0.3])  # random orient with Euler Angles



    # EXERCISE 3.3: Control of orientation with PD - sinusoidal reference 
#    rpy_des = np.array([2.2*np.pi*np.sin(time), 0.0 , 0.0])
#    rpyd_des = np.array([2.2*np.pi*np.cos(time), 0.0 , 0.0])  
#    rpydd_des = np.array([-2.2*np.pi*np.sin(time), 0.0 , 0.0]) 
#    # compute rotation matrix representing the desired orientation from Euler Angles
#    w_R_des = math_utils.eul2Rot(rpy_des)        
#    # desired angular velocity		            
#    omega_des = math_utils.Tomega(rpy_des).dot(rpyd_des)
#    # desired angular acceleration			             
#    omega_d_des = math_utils.Tomega(rpy_des).dot(rpydd_des) +  math_utils.Tomega_dot(rpy_des, rpyd_des).dot(rpyd_des)
   
   
   
    # EXERCISE 3.1: Control of orientation with PD   
#    # compute rotation matrix from actual orientation of ee to the desired
#    e_R_des = w_R_e.T.dot(w_R_des)    
#    # compute the angle-axis representation of the associated orientation error				
#    # compoute the angle: method 1) with arc cos
#    #cos_theta = (e_R_des[0,0]+ e_R_des[1,1]+ e_R_des[2,2]-1)/2
#    #delta_theta = np.arccos( cos_theta) 
#    #compoute the angle: method 2) with atan2
#    delta_theta = math.atan2(np.sqrt(pow(e_R_des[2,1]-e_R_des[1,2], 2) +  pow(e_R_des[0,2]-e_R_des[2,0], 2) + pow(e_R_des[1,0]-e_R_des[0,1], 2)), e_R_des[0,0]+ e_R_des[1,1]+ e_R_des[2,2]-1 )  
#    # compute the axis    
#    r_hat = 1/(2*np.sin(delta_theta))*np.array([e_R_des[2,1]-e_R_des[1,2], e_R_des[0,2]-e_R_des[2,0], e_R_des[1,0]-e_R_des[0,1]])     
#    # compute the orientation error
#    e_error_o = delta_theta * r_hat 
#    # the error is expressed in the end-effector frame 
#    # we need to map it in the world frame to compute the moment because the jacobian is in the WF
#    w_error_o = w_R_e.dot(e_error_o) 				
#    # Compute the virtual force (linear part of the wrench) 
#    F_des = conf.Kx.dot(p_des-p) + conf.Dx.dot(pd_des-pd)
#    # compute the virtual moment (angular part of the wrench) to realize the orientation task
#    Gamma_des =  conf.Ko.dot(w_error_o) + conf.Do.dot(omega_des - omega)
#    # stack the previously computed linear part with the angular part
#    W_des = np.hstack([F_des, Gamma_des])
#    # map to torques
#    tau = J6.T.dot(W_des)            
#    # extract actual euler angles for logging   
#    rpy = math_utils.rot2eul(w_R_e)



    # EXERCISE 3.4: Control of orientation with PD - unwrapping  
    #unwrap euler angles   
#    UNWRAPPPING = True
#    for i in range(3):
#        rpy_unwrapped[i] = rpy[i];
#        while (rpy_unwrapped[i] < rpy_old[i]  - math.pi):
#            rpy_unwrapped[i] += 2*math.pi
#        while (rpy_unwrapped[i] > rpy_old[i]  + math.pi):
#            rpy_unwrapped[i] -= 2*math.pi
#        rpy_old[i] = rpy_unwrapped[i]
#    for i in range(3):
#        rpy_des_unwrapped[i] = rpy_des[i];
#        while (rpy_des_unwrapped[i] < rpy_des_old[i]  - math.pi):
#            rpy_des_unwrapped[i] += 2*math.pi
#        while (rpy_des_unwrapped[i] > rpy_des_old[i]  + math.pi):
#            rpy_des_unwrapped[i] -= 2*math.pi
#        rpy_des_old[i] = rpy_des_unwrapped[i]
    
    #EXERSISE 3.6 : full task space inverse dynamics (computed torque)
#    # compute lambda for both orientation and position
#    rho = 0.00001 # damping factor
#    #lambda6_= np.linalg.inv(J6.dot(M_inv).dot( J6.T))  #singular   
#    lambda6_ = np.linalg.inv(J6.dot(M_inv).dot( J6.T) + rho*eye(6)) # damped inertia matrix
#    #J6Tpinv = np.linalg.pinv(J6.T, rho) # damped pseudoinverse using native function     
#    J6Tpinv = np.linalg.inv(J6.dot(J6.T) + pow(rho,2)*eye(6)).dot(J6)  # damped pseudoinverse explicitely computed  
#    Jdqd6 = robot.frameClassicAcceleration(q, qd, None, frame_ee).vector   
#    mu6 = -lambda6_.dot(Jdqd6)  + J6Tpinv.dot(h)    
#    tau = J6.T.dot(lambda6_.dot(np.hstack((pdd_des + F_des, omega_d_des + Gamma_des))) + mu6) 
    			
#    EXERCISE 2.3: Add an external force
    if conf.EXTERNAL_FORCE  and time>1.0:
        tau += J.transpose().dot(conf.extForce)
        ros_pub.add_arrow(p, conf.extForce/100)                    
    
    #SIMULATION of the forward dynamics    
    qdd = M_inv.dot(tau - h)    
    
    # Forward Euler Integration    
    qd = qd + qdd*conf.dt
    q = q + qd*conf.dt + 0.5*conf.dt*conf.dt*qdd
				    
    # Log Data into a vector				
    time_log = np.append(time_log, time)	           
    p_log = np.vstack((p_log, p ))
    p_des_log= np.vstack((p_des_log, p_des))
    pd_log= np.vstack((pd_log, pd))
    pd_des_log= np.vstack((pd_des_log, pd_des))
    pdd_des_log= np.vstack((pdd_des_log, pdd_des))
    tau_log = np.vstack((tau_log, tau)) 
    try: 
        UNWRAPPPING
        rpy_log= np.vstack((rpy_log, rpy_unwrapped))
        rpy_des_log= np.vstack((rpy_des_log, rpy_des_unwrapped))
    except:    
        rpy_log= np.vstack((rpy_log, rpy))
        rpy_des_log= np.vstack((rpy_des_log, rpy_des))
    try: 
        ORIENTATION_CONTROL
        error_o_log= np.vstack((error_o_log, w_error_o))
    except: 
        pass                      
 
    # update time
    time = time + conf.dt                  
    
    # plot ball at the end-effector
    ros_pub.add_marker(p)                   
    #publish joint variables
    ros_pub.publish(robot, q, qd, tau)                   
    tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    # stops the while loop if  you prematurely hit CTRL+C                    
    if ros_pub.isShuttingDown():
        print ("Shutting Down")                    
        break;            
ros_pub.deregister_node()
      
#plot position
plotEndeff('position', 1,time_log, p_log, p_des_log)
#plotEndeff('velocity', 2, time_log, p_log, p_des_log, pd_log, pd_des_log, rpy_log, rpy_des_log)
#plotEndeff('orientation', 3,time_log, p_log, p_des_log, pd_log, pd_des_log, rpy_log, rpy_des_log)
#plotEndeff('orientation', 4,time_log, p_log, p_des_log, pd_log, pd_des_log, error_o_log)


    
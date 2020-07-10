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

from gazebo_controller.math_tools import Math
import ex_2_conf as conf

#instantiate graphic utils
ros_pub = RosPub()
robot = importDisplayModel(False, False)

math = Math()
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
euler_log = np.empty((3,0))*nan
euler_des_log = np.empty((3,0))*nan
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

# compute initial end effector position and velocity
x0 = robot.framePlacement(q, frame_ee, True).translation + np.matrix([0.0, 0.0, 0.0]).T
xd0 = np.matrix([ 0.0, 0.0, 0.0]).T
xdd0 = np.matrix([ 0.0, 0.0, 0.0]).T
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
    #...
        
    # EXERCISE 2: Step reference generation  for end effector 
    #...
        
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
    xd = J*qd  

    M_inv = np.linalg.inv(M)
    # Moore-penrose pseudoinverse  A^# = (A^TA)^-1 * A^T with A = J^T
    JTpinv = np.linalg.inv(J*J.T)*J
    lambda_= np.linalg.inv(J*M_inv* J.T)

    #Null space projector
    N = (eye(6)-J.T*JTpinv)  
    # null space torques (postural task)
    tau0 = 50*(conf.q0-q) - 10*qd
    tau_null = N*tau0
				         
    # EXERCISE 4: PD control (cartesian task)
    #Fdes = ...
    #tau = ...         
             
    # EXERCISE 5: PD control + Gravity Compensation:
    #Fdes = ...
    #tau = ... 
        
    # EXERCISE 6: PD control  + Gravity Compensation + Feed-Forward term
    #Fdes = ...
    #tau = ... 
     
    # EXERCISE 7: Operational space inverse dynamics
    #Fdes = ...
    #tau = ...  
    
     # EXERCISE 8: OSID with bias compensation in joint space (simpler to compute)
    #Fdes = ...
    #tau = ... 
    
     #EXERCISE 11: dyn consistent pseudon-inverse
    #Fdes = ...
    #tau = ...   
    
    # EXERCISE 10: Control of orientation
    # F_des				
    # Tau_des = ...
    #  W_des = np.vstack([F_des, Tau_des])			
    # tau = ...   
    #to log 
    #euler = math.rotTorpy(e_R_w)				
    #euler_des = math.rotTorpy(des_R_w)
				
#    EXERCISE 9: Add external force
    if conf.EXTERNAL_FORCE  and time>2.0:
        # tau += ...
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
    euler_log= np.hstack((euler_log, euler.reshape(3,-1)))
    euler_des_log= np.hstack((euler_des_log, euler_des.reshape(3,-1)))
    tau_log = np.hstack((tau_log, tau))                
 
    # update time
    time = time + conf.dt                  
    
    # plot ball at the end-effector
    ros_pub.add_marker(x.A1.tolist())                   
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
#plotEndeff('velocity', 2, x_log, x_des_log, xd_log, xd_des_log, euler_log, euler_des_log)
#plotEndeff('orientation', 2,time_log, x_log, x_des_log, xd_log, xd_des_log, euler_log, euler_des_log)
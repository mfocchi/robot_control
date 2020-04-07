from __future__ import print_function 
import pinocchio as pin
from pinocchio.utils import *
import numpy as np

import math

import time as tm

import eigenpy
eigenpy.switchToNumpyMatrix()
import sys
import os

from common_functions import *
from quadprog import solve_qp    
 
# Print options 
np.set_printoptions(precision = 3, linewidth = 200, suppress = True)
np.set_printoptions(threshold=np.inf)
sys.dont_write_bytecode = True





# Control loop interval
dt = 0.001
exp_duration_sin = 3.0 
exp_duration = 5.0
num_samples = (int)(exp_duration/dt)

## Matrix of KP gains
kp=eye(6)*200
# Matrix of KD gains (critical damping)
kd=eye(6)*20

#EXERCISE 6: high gains ...
#kp=eye(6)*500
#kd=eye(6)*30

# Parameters of Joint Reference Trajectories
amplitude = np.array([ 0.0, 0.4, 0.0, 0.0, 0.0, 0.0])
frequencies = np.array([ 0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
w_rad=2*math.pi*frequencies

# Value of linear external force
extForce = np.matrix([0.0, 0.0, 50.0]).T

# FLAGS
EXTERNAL_FORCE = True
DISPLAY = 1
DISPLAY_FLOOR = 1

# Init variables
zero = np.matrix([0.0, 0.0,0.0, 0.0, 0.0, 0.0]).T
time = 0.0
count = 0

robot = importDisplayModel(DISPLAY, DISPLAY_FLOOR)
    
    
# Init loggers
q_log = np.zeros((6,num_samples))
q_des_log = np.zeros((6,num_samples))
qd_log = np.zeros((6,num_samples))
qd_des_log = np.zeros((6,num_samples))
qdd_log = np.zeros((6,num_samples))
qdd_des_log = np.zeros((6,num_samples))
f_log = np.zeros((3,num_samples))
time_log = np.zeros(num_samples)

# Initial configuration / velocity / Acceleration
q0  = np.matrix([ 0.0, -0.1, -0.0, -1.57, -1.57, 0.0]).T
qd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
# EXERCISE 9: 
#qd0 = np.matrix([ 0, w_rad[1]*amplitude[1], 0.0, 0.0, w_rad[4]*amplitude[4], 0.0]).T
qdd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T

q = q0
qd = qd0
qdd = qdd0
q_des  = q0 
qd_des = zero
qdd_des = zero
f = np.matrix([0.0, 0.0,0.0]).T
#end effector ID
jid = robot.model.getJointId('wrist_3_joint')

#TODO
#robot.viewer.gui.addArrow('world/contact_arrow', 0.01, 1.0, EE_SPHERE_COLOR)

z_gnd = 0.0
ground_contact = False


# CONTROL LOOP
while True:   

    # Reference Generation
    
#    Sinusoidal reference
    q_des  = q0 + np.matrix([ 0.0, amplitude[1]*np.sin(w_rad[1]*time), 0.0, 0.0,  amplitude[4]*np.sin(w_rad[4]*time), 0.0]).T
    qd_des = np.matrix([ 0.0,  amplitude[1]*w_rad[1]*np.cos(w_rad[1]*time), 0.0, 0.0, amplitude[4]*w_rad[4]*np.cos(w_rad[4]*time), 0.0]).T
    qdd_des = np.matrix([ 0.0, -amplitude[1]*w_rad[1]*w_rad[1]*np.sin(w_rad[1]*time), 0.0, 0.0, -amplitude[4]*w_rad[4]*w_rad[4]*np.sin(w_rad[4]*time), 0.0]).T  
    # Set constant reference after a while
    if time>exp_duration_sin:
        q_des  = q0
        qd_des=zero
        qdd_des=zero         
 
    # Decimate print of time
    #if (divmod(count ,1000)[1]  == 0):
       #print('Time %.3f s'%(time))
    if time >= exp_duration:
        break

    #compute Jacobian and its derivative in the world frame  
    pin.computeJointJacobians(robot.model, robot.data, q)
    robot.computeJointJacobians(q)
    #get local jacobian
    J_i = robot.getJointJacobian(jid)[:3,:]
    R = robot.placement(q, jid).rotation                
    pin.computeJointJacobiansTimeVariation(robot.model, robot.data, q, qd)
    J_i_dot=pin.getJointJacobianTimeVariation(robot.model, robot.data, jid, pin.ReferenceFrame.LOCAL)[:3,:]
    
    #map jacobian to world frame
    J = R * J_i       
    Jdot = R * J_i_dot
    
    # compute end effector position and velocity    
    x = robot.placement(q, jid).translation
    xd = J*qd
				
				
    #compute matrix M of the model
    M = robot.mass(q)
    #compute bias terms
    h = robot.nle(q, qd)    
    #compute gravity terms
    g = robot.gravity(q)        
				
    M_inv = np.linalg.inv(M)
    #    #pseudoinverse of J^T = (A^TA)^-1 * A^T
    JTpinv = np.linalg.inv(J*J.T)*J
 
    #lambda_ contact inertia
    lambda_= np.linalg.inv(J*M_inv* J.T) 
    
    #dynamicaaly consistent psdinv
    JTpinv = lambda_*J*M_inv
    N_dyn = (eye(6)- J.T * JTpinv) 
    
    #tau = M*(qdd_des+kp*(q_des-q)+kd*(qd_des-qd)) + h    
    tau = kp*(q_des-q)+kd*(qd_des-qd) + g 
    
            
                
    #####simulator
    ################################            
    #detect contact
    if ((ground_contact) and x[2]>z_gnd+0.01):
       ground_contact = False;
       print ("contact OFF")
							
    if (not ground_contact) and (x[2]<=z_gnd):
        ground_contact = True;       
        print("contact ON")				
							
    # compute forward dynamics     
    qdd_uc = M_inv*(tau - h)              
    if (not ground_contact):
        qdd = qdd_uc
        f = np.matrix([0.0, 0.0,0.0]).T								
    else:
        qd = N_dyn*qd                  
        qdd = M_inv*( N_dyn*(tau -    h) - J.T*    lambda_*Jdot*qd)    
        # compute contact force    
        f = lambda_*( -Jdot*qd + J*M_inv*(h - tau) )
        print(f.T)								
	   #gauss principle of leas constraint verification
	   #Minimize     1/2 x^T G x - a^T x
	   #Subject to   C.T x >= b
	#								    G : array, shape=(n, n)
	#        matrix appearing in the quadratic function to be minimized
	#    a : array, shape=(n,)
	#        vector appearing in the quadratic function to be minimized
	#    C : array, shape=(n, m)
	#        matrix defining the constraints under which we want to minimize the
	#        quadratic function
	#    b : array, shape=(m), default=None
	#        vector defining the constraints
	#    meq : int, default=0
	#        the first meq constraints are treated as equality constraints,
	#        all further as inequality constraints (defaults to 0).

        qp_C = J.T
        qp_b = -Jdot*qd
        meq = J.shape[0]       				
        p = M*qdd_uc						

        qdd_check = solve_qp(M,p.A1,qp_C, qp_b.A1, meq)  [0]  
						
        #print("proj - gauss") 					
        #print( qdd.A1 - qdd_check)
  	
					
								
                        
                        
    # Forward Euler Integration        
    q = q + qd*dt + 0.5*dt*dt*qdd
    qd = qd + qdd*dt
    
    # Log Data into a vector
    q_log[:,count] = q.flatten()
    q_des_log[:,count] = q_des.flatten()
    qd_log[:,count] = qd.flatten()
    qd_des_log[:,count] = qd_des.flatten()
    qdd_log[:,count] = qdd.flatten()
    qdd_des_log[:,count] = qdd_des.flatten()
    time_log[count] = time 
    f_log[:,count] = f.flatten()
    time = time + dt 
    count +=1
    
    tm.sleep(dt)
    if DISPLAY:    
        robot.display(q)                    
        robot.viewer.gui.applyConfiguration('world/ee',     x.A1.tolist()+[0,0,0,1.])
    #==============================================================================TODO
	    #    if (ground_contact):							
	
	# 	        norm = np.linalg.norm(f.flatten())	 
	# 									
	# 	        force_end = x + f/norm						
	# 	        robot.viewer.gui.applyConfiguration('world/contact_arrow',     force_end.A1.tolist()+[0,0,0,1.])    						
	#==============================================================================
        
plot('pos', time_log,  q_des_log, q_log, qd_des_log, qd_log, qdd_des_log, qdd_log, num_samples)
#plot('vel', q_des_log, q_log, qd_des_log, qd_log, qdd_des_log, qdd_log)
#plot('acc', q_des_log, q_log, qd_des_log, qd_log, qdd_des_log, qdd_log)

plt.figure(2)
plt.subplot(3,1,1)
plt.title("fx")	
plt.plot(time_log, f_log[0,:],color = 'red')
plt.grid()

plt.subplot(3,1,2)
plt.title("fy")	
plt.plot(time_log, f_log[1,:],color = 'red')
plt.grid()

plt.subplot(3,1,3)
plt.title("fz")	
plt.plot(time_log, f_log[2,:],color = 'red')
plt.grid()





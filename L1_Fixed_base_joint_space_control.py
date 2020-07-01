from __future__ import print_function 
import pinocchio
from pinocchio.utils import *
import numpy as np
import matplotlib.pyplot as plt
import math
from ros_publish import RosPub #TODO 
from utils.common_functions import *

import time as tm
from pinocchio.robot_wrapper import RobotWrapper
import eigenpy
eigenpy.switchToNumpyMatrix()
import sys
import os


# Import the model
ERROR_MSG = 'You should set the environment variable UR5_MODEL_DIR to something like "$DEVEL_DIR/install/share"\n';
path      = os.environ.get('UR5_MODEL_DIR', ERROR_MSG)
urdf      = path + "/ur_description/urdf/ur5_modified.urdf";
srdf      = path + '/ur5_description/srdf/ur5_modified.srdf'
robot = RobotWrapper.BuildFromURDF(urdf, [path,srdf ])

# Control loop interval
dt = 0.001
exp_duration_sin = 3.0 
exp_duration = 5.0
num_samples = (int)(exp_duration/dt)

## Matrix of KP gains
kp=eye(6)*50
# Matrix of KD gains (critical damping)
kd=eye(6)*20

#EXERCISE 4: high gains ...
#kp=eye(6)*500
#kd=eye(6)*30

# Parameters of Joint Reference Trajectories
amplitude = np.array([ 0.0, 0.2, 0.0, 0.0, 0.4, 0.0])
frequencies = np.array([ 0.0, 1.0, 0.0, 0.0, 1.5, 0.0])
w_rad=2*math.pi*frequencies

# Value of linear external force
extForce = np.matrix([0.0, 0.0, 50.0]).T

# FLAGS
EXTERNAL_FORCE = False
DISPLAY = 1
DISPLAY_FLOOR = 0

# Init variables
zero = np.matrix([0.0, 0.0,0.0, 0.0, 0.0, 0.0]).T
time = 0.0
count = 0

if DISPLAY:
    import commands
    import gepetto
    from time import sleep
    robot.initViewer(loadModel=True)
    l = commands.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
    if int(l[1]) == 0:
        os.system('gepetto-gui &')
    sleep(1)
    gepetto.corbaserver.Client()
    robot.initViewer(loadModel=True)
    gui = robot.viewer.gui
    if(DISPLAY_FLOOR):
        robot.viewer.gui.createSceneWithFloor('world')
        gui.setLightingMode('world/floor', 'ON')
    robot.displayCollisions(False)
    robot.displayVisuals(True)
    
    
# Init loggers
q_log = np.zeros((6,num_samples))
q_des_log = np.zeros((6,num_samples))
qd_log = np.zeros((6,num_samples))
qd_des_log = np.zeros((6,num_samples))
qdd_log = np.zeros((6,num_samples))
qdd_des_log = np.zeros((6,num_samples))
tau_log = np.zeros((6,num_samples))
time_log = np.zeros(num_samples)

# Initial configuration / velocity / Acceleration
q0  = np.matrix([ 0.0, 0.0, -0.5, 0.5, -1.57, 0.5]).T
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

#end effector ID
jid = robot.model.getJointId('wrist_3_joint')

REF_SPHERE_RADIUS = 0.03
EE_SPHERE_COLOR  = (1, 0.5, 0, 0.5)
EE_REF_SPHERE_COLOR  = (1, 0, 0, 0.5)


# CONTROL LOOP
while True:

    
    if DISPLAY and not count%100:    
        robot.display(q)            
#        robot.viewer.gui.addSphere('world/ee_fixed_joint', 0.1, EE_SPHERE_COLOR)
    
    # Reference Generation
    # EXERCISE 1: Sinusoidal reference
    q_des  = q0 + np.matrix([ 0.0, amplitude[1]*np.sin(w_rad[1]*time), 0.0, 0.0,  amplitude[4]*np.sin(w_rad[4]*time), 0.0]).T
    qd_des = np.matrix([ 0.0,  amplitude[1]*w_rad[1]*np.cos(w_rad[1]*time), 0.0, 0.0, amplitude[4]*w_rad[4]*np.cos(w_rad[4]*time), 0.0]).T
    qdd_des = np.matrix([ 0.0, -amplitude[1]*w_rad[1]*w_rad[1]*np.sin(w_rad[1]*time), 0.0, 0.0, -amplitude[4]*w_rad[4]*w_rad[4]*np.sin(w_rad[4]*time), 0.0]).T  
    # Set constant reference after a while
    if time>exp_duration_sin:
        q_des  = q0
        qd_des=zero
        qdd_des=zero         
 
    
    # EXERCISE 2: Step reference
#    if time > 2.0:
#        q_des = q0 + np.matrix([ 0.0, -0.4, 0.0, 0.0,  0.5, 0.0]).T 
#        qd_des =  zero
#        qdd_des = zero 
#    else:
#        q_des = q0
#        qd_des =  zero
#        qdd_des = zero 
    
            # Decimate print of time
    if (divmod(count ,1000)[1]  == 0):
        print('Time %.3f s'%(time))
    if time >= exp_duration:
        break

    # compute matrix M of the model
    M= pinocchio.crba(robot.model, robot.data, q)

    # compute bias terms h
    h = pinocchio.rnea(robot.model, robot.data, q, qd, zero)
    
    # compute gravity terms
    g = pinocchio.rnea(robot.model, robot.data, q, zero, zero)
    
        
    # EXERCISE  5: PD control critical damping
    kd[0,0] = 2*np.sqrt(kp[0,0]*M[0,0])
    kd[1,1] = 2*np.sqrt(kp[1,1]*M[1,1])
    kd[2,2] = 2*np.sqrt(kp[2,2]*M[2,2])
    kd[3,3] = 2*np.sqrt(kp[3,3]*M[3,3])
    kd[4,4] = 2*np.sqrt(kp[4,4]*M[4,4])
    kd[5,5] = 2*np.sqrt(kp[5,5]*M[5,5])
#    print (2*np.sqrt(300*M[4,4])    )
        
    #Exercise 3:  PD control
#    tau=kp*(q_des-q)+kd*(qd_des-qd)
    
    # Exercise 6: PD control + Gravity Compensation
#    tau=kp*(q_des-q)+kd*(qd_des-qd) + g
    
    # Exercise 7: PD + gravity + Feed-Forward term
    tau= np.diag(M)*qdd_des + kp*(q_des-q)+kd*(qd_des-qd) + g

    
    # EXERCISE 8_ Innverse Dynamics
    #tau=M*(qdd_des+kp*(q_des-q)+kd*(qd_des-qd)) + h    
    
    
    #SIMULATION
    #compute fwd dynamics    
    M_inv = np.linalg.inv(M)  
    
    # Add external force if any (EXERCISE 11)
    if EXTERNAL_FORCE  and time>2.0:
     #compute Jacobian and its derivative in the world frame  
			
     pinocchio.computeJointJacobians(robot.model, robot.data, q)
     robot.computeJointJacobians(q)
     J_i = robot.getJointJacobian(jid)[:3,:]
     R = robot.placement(q, jid).rotation
     J = R * J_i       
     tau += J.transpose()*extForce
    qdd = M_inv*(tau-h)    
    
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
    tau_log[:,count] = tau.flatten()
				
    time_log[count] = time  
    time = time + dt 
    count +=1
    
    tm.sleep(dt)
    

        
                    
# plot joint variables                                                                              
plotJoint('position', 0, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('velocity', 1, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('acceleration', 2, time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)







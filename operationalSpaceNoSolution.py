# -*- coding: utf-8 -*-
"""
Created on Thu Dec 12 13:47:33 2019

@author: mfocchi
"""

from __future__ import print_function 
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
#from tsid_manipulator import TsidManipulator
import matplotlib.pyplot as plt
import math
import os
import time as tm
#from pinocchio.visualize import GepettoVisualizer
from pinocchio.robot_wrapper import RobotWrapper

import eigenpy
eigenpy.switchToNumpyMatrix()
import sys
import os

# Print options 
np.set_printoptions(precision = 3, linewidth = 200, suppress = True)
np.set_printoptions(threshold=np.inf)
import sys
sys.dont_write_bytecode = True


ERROR_MSG = 'You should set the environment variable UR5_MODEL_DIR to something like "$DEVEL_DIR/install/share"\n';
path      = os.environ.get('UR5_MODEL_DIR', ERROR_MSG)
urdf      = path + "/ur_description/urdf/ur5_gripper.urdf";
srdf      = path + '/ur_description/srdf/ur5_gripper.srdf'
robot = RobotWrapper.BuildFromURDF(urdf, [path, ])

# Control loop interval
dt=0.001
exp_duration_sin = 3.0 
exp_duration = 5.0
num_samples = (int)(exp_duration/dt)

## Matrix of KP gains
Kx=eye(3)
Kx[0,0] = 1000
Kx[1,1] = 1000
Kx[2,2] = 1000

Dx=eye(3)
Dx[0,0] = 300
Dx[1,1] = 300
Dx[2,2] = 300

extForce = np.matrix([0.0, 0.0, 100.0]).T

# Parameters of Joint Reference Trajectories
amplitude=np.array([ 0.0, 0.0, 0.2])
frequencies=np.array([ 0.0, 0.0, 0.8])
w_rad=2*math.pi*frequencies

# FLAGS
DISPLAY = True
DISPLAY_FLOOR = False
EXTERNAL_FORCE = True

#init variables
zero  = np.matrix([ 0.0, 0.0,0.0, 0.0, 0.0, 0.0]).T
zero_cart = np.matrix([ 0.0, 0.0,0.0]).T
count = 0
time = 0.0

#end effector ID
jid = robot.model.getJointId('wrist_3_joint')

if DISPLAY: q = q + qd*dt + 0.5*dt*dt*qdd
    qd = qd + qdd*dt
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
x_log = np.zeros((3,num_samples))
x_des_log = np.zeros((3,num_samples))
xd_log = np.zeros((3,num_samples))
xd_des_log = np.zeros((3,num_samples))
xdd_log = np.zeros((3,num_samples))
xdd_des_log = np.zeros((3,num_samples))
time_log = np.zeros(num_samples)

# Initial configuration 
q0  = np.matrix([ 0.0, 0.0, -0.5, 0.5, -1.57, 0.5]).T
q = q0
qd = zero #+ np.random.rand(robot.model.nv,1)
qdd = zero

# compute initial end effector position and velocity
pin.computeAllTerms(robot.model, robot.data, q, qd)
x0 = robot.data.oMi[jid].translation + np.matrix([0.0, 0.0, 0.0]).T
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
        
    if DISPLAY and count%100==0:    
        robot.display(q) 
    
    # EXERCISE 1: Sinusoidal referencefor end effector   
   
        
    # EXERCISE 2: Step reference

        
    if count%1000 == 0:
        print('Time %.3f s'%(time))
    if time >= exp_duration:
        break
    
       
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

    #compute matrix M of the model
    M = robot.mass(q)

    #compute bias terms
    h = robot.nle(q, qd)
    
    #compute gravity terms
    g = robot.gravity(q)
        
    M_inv = np.linalg.inv(M)
    
    #    #pseudoinverse of J^T = (A^TA)^-1 * A^T
    JTpinv = np.linalg.inv(J*J.T)*J
 
    #lambda_xdd+u=F
    lambda_= np.linalg.inv(J*M_inv* J.T)
    
#    # EXERCISE 4: 

    
#    # EXERCISE 5:     

#    # EXERCISE 6:

    #Null space projector
    N = (eye(6)-J.T*JTpinv)  
    # null space torques
    tau0 = 100*(q0-q) - 10*qd
    tau_null = N*tau0
    F_des = xdd_des+Kx*(x_des-x)+Dx*(xd_des-xd)
    u = -lambda_*(Jdot*qd)   + JTpinv*h 

    # EXERCISE 7:
    
    # EXERCISE 8:

    # EXERCISE 10:
   
    

    #SIMULATION
    
#    EXERCISE 9
    if EXTERNAL_FORCE  and time>2.0:
     tau += J.transpose()*extForce
    qdd = M_inv*(tau-h)

    #SIMULATION
    
    #compute fwd dynamics    
   
    qdd = M_inv*(tau-h)

    
    #forward euler integration 
    q = q+qd*dt
    qd = qd+qdd*dt    
    
    #forward kinematics   
    
    x_log[:,count] = x.flatten()
    x_des_log[:,count] = x_des.flatten()
    xd_log[:,count] = xd.flatten()
    xd_des_log[:,count] = xd_des.flatten()
    #xdd_log[:,count] = xdd.flatten()
    #xdd_des_log[:,count] = xdd_des.flatten()
    time_log[count] = time
    time = time + dt 
    count +=1
     

    tm.sleep(dt)


#plot position
lw_des=2
lw_act=4
plt.close()
plt.figure()
plt.subplot(3,1,1)
plt.title("x")
plt.plot(time_log, x_des_log[0,:],linestyle='-', lw=lw_des,color = 'red')
plt.plot(time_log, x_log[0,:],linestyle='-', lw=lw_act,color = 'blue')
plt.grid()

plt.subplot(3,1,2)
plt.title("y")     
plt.plot(time_log, x_des_log[1,:],linestyle='-', lw=lw_des,color = 'red', label="q_des")
plt.plot(time_log, x_log[1,:],linestyle='-',lw=lw_act, color = 'blue', label="q")
plt.grid()

plt.subplot(3,1,3)
plt.title("z")
plt.plot(time_log, x_des_log[2,:],linestyle='-',lw=lw_des,color = 'red')
plt.plot(time_log, x_log[2,:],linestyle='-',lw=lw_act,color = 'blue')
plt.grid()

# Plot velocity   
#plt.figure()
#plt.subplot(3,1,1)
#plt.title("x")
#plt.plot(time_log, xd_des_log[0,:],linestyle='-', lw=lw_des,color = 'red')
#plt.plot(time_log, xd_log[0,:],linestyle='-', lw=lw_act,color = 'blue')
#plt.grid()
 
#plt.subplot(3,1,2)
#plt.title("y")     
#plt.plot(time_log, xd_des_log[1,:],linestyle='-', lw=lw_des,color = 'red', label="q_des")
#plt.plot(time_log, xd_log[1,:],linestyle='-',lw=lw_act, color = 'blue', label="q")
#plt.grid()
##
##
#plt.subplot(3,1,3)
#plt.title("z")
#plt.plot(time_log, xd_des_log[2,:],linestyle='-',lw=lw_des,color = 'red')
#plt.plot(time_log, xd_log[2,:],linestyle='-',lw=lw_act,color = 'blue')
# plt.grid()
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os

dt = 0.001                   # controller time step
exp_duration_sin = 4.0 #sine reference duration
exp_duration = 5.0 #simulation duration

SLOW_FACTOR = 1#to slow down simulation

frame_name = 'ee_link'    # name of the frame to control (end-effector)

#PD controller
## Matrix of gains
kp = np.eye(6)*300  # proportional gains 
kd = np.eye(6)*20 # derivative gains (critical damping)

## PARAMETERS OF REFERENCE SINUSOIDAL TRAJECTORY (1, 2, 3, 4, 5, 6 joint)
amp = np.array([ 0.0, 0.2, 0.0, 0.0, 0.2, 0.0])    # amplitude
phi = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])      # phase
freq = np.array([ 0.0, 1.0, 0.0, 0.0, 1.5, 0.0])    # frequency


# Initial configuration / velocity / Acceleration
q0  = np.array([ 0.0, -0.3, 0.5, -1.57, -1.57, 0.5]) 
qd0 = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
qdd0 = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 


#EXERCISE 1.4: high gains
#kp = np.eye(6)*600
#kd = np.eye(6)*30

# EXERCISE 2.4: Add external force
# Value of linear external force
extForce = np.array([0.0, 0.0, 50.0])
# FLAGS
EXTERNAL_FORCE = True

# EXERCISE 2.7: Add  unilateral compliant contact
n = np.array([0.0,0.0,1.0]) # contact normal
p0 = np.array([0.0,0.0,0.0]) # contact position     
K_env = np.eye(3)*10000 # contact stiffness 
D_env = np.eye(3)*1000   # contact damping
mu = 0.8
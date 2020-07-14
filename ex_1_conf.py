# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os

dt = 0.001                   # controller time step
exp_duration_sin = 3.0 #sine reference duration
exp_duration = 5.0 #simulation duration

SLOW_FACTOR = 1 #to slow down simulation

frame_name = 'ee_link'    # name of the frame to control (end-effector)

#PD controller
## Matrix of gains
kp = np.eye(6)*600  # proportional gains 
kd = np.eye(6)*20 # derivative gains (critical damping)

## PARAMETERS OF REFERENCE SINUSOIDAL TRAJECTORY
amp = np.matrix([ 0.0, 0.2, 0.0, 0.0, 0.4, 0.0]).T    # amplitude
phi = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T    # phase
freq = np.matrix([ 0.0, 1.0, 0.0, 0.0, 1.5, 0.0]).T  # frequency


# Initial configuration / velocity / Acceleration
q0  = np.matrix([ 0.0, 0.0, -0.5, 0.5, -1.57, 0.5]).T
qd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
qdd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T


#EXERCISE 4: high gains
#kp=np.eye(6)*500
#kd=np.eye(6)*30


# Value of linear external force
extForce = np.matrix([0.0, 0.0, 50.0]).T
# FLAGS
EXTERNAL_FORCE = True


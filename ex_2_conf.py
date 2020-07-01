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

## Matrix of KP gains
Kx= np.eye(3)
Kx[0,0] = 1000
Kx[1,1] = 1000
Kx[2,2] = 1000

Dx = np.eye(3)
Dx[0,0] = 300
Dx[1,1] = 300
Dx[2,2] = 300

# P angular gains				
Ktheta= np.eye(3)
Ktheta[0,0] = 500
Ktheta[1,1] = 500
Ktheta[2,2] = 500
# D angular gains
Dtheta= np.eye(3)
Dtheta[0,0] = 30
Dtheta[1,1] = 30
Dtheta[2,2] = 30

## PARAMETERS OF REFERENCE CARTESIAN SINUSOIDAL TRAJECTORY
amp=np.matrix([ 0.1, 0.0, 0.0]).T   # amplitude
phi =np.matrix([ 0.0, 0.0, 0.0]).T  # phase
freq=np.matrix([ 1.5, 0.0, 0.0]).T  # frequency

# Initial configuration / velocity / Acceleration
q0  = np.matrix([ 0.0, -1, 1, 0.5, 0, 0.5]).T
qd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
qdd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T

# EXERCISE 9: Add external force
# Value of linear external force
extForce = np.matrix([0.0, 0.0, 100.0]).T
# FLAGS
EXTERNAL_FORCE = False


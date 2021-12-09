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
Ko= np.eye(3)
Ko[0,0] = 800
Ko[1,1] = 800
Ko[2,2] = 800
# D angular gains
Do= np.eye(3)
Do[0,0] = 30
Do[1,1] = 30
Do[2,2] = 30

## PARAMETERS OF REFERENCE CARTESIAN SINUSOIDAL TRAJECTORY
amp=np.array([ 0.1, 0.0, 0.0])   # amplitude
phi =np.array([ 0.0, 0.0, 0.0])  # phase
freq=np.array([ 1.5, 0.0, 0.0])  # frequency

# Initial configuration / velocity / Acceleration
q0  = np.array([ 0.0, -1, 1, 0.5, 0, 0.5])
qd0 = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
qdd0 = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# EXERCISE 9: Add external force
# Value of linear external force
extForce = np.array([0.0, 0.0, 200.0])
# FLAGS
EXTERNAL_FORCE = False


# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os

dt = 0.001                   # controller time step
<<<<<<< HEAD
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
=======
exp_duration_sin = 2.0 #sine reference duration
exp_duration = 3.0 #simulation duration

SLOW_FACTOR = 0.1 #to slow down simulation

frame_name = 'ee_link'    # name of the frame to control (end-effector)

# Joint-space stiffness				
Ktheta= np.eye(6)*500

# Joint-space damping
Dtheta= np.eye(6)*10

# Joint-space inertia
Mtheta = np.eye(6)*0.1

#Inverse of the joint-space inertia
Mtheta_inv = np.linalg.inv(Mtheta)

## PARAMETERS OF JOINT SINE WAVE
ampJS = np.matrix([ 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]).T   # amplitude
phiJS = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T   # phase
freqJS = np.matrix([ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]).T  # frequency

# Initial configuration / velocity / Acceleration
q0  = np.matrix([ 0.0, -1, 1, 0.5, 0, 0.5]).T
qd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
qdd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T

>>>>>>> 89d59ff6b9c2fec6953fda8b8c934b640df0221a
# FLAGS
EXTERNAL_FORCE = False


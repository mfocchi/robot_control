# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os

dt = 0.001                   # controller time step
exp_duration_sin = 2.0 #sine reference duration
exp_duration = 3.0 #simulation duration

SLOW_FACTOR = 0.1 #to slow down simulation

frame_name = 'ee_link'    # name of the frame to control (end-effector)

## Matrix of KP gains
Kx= np.eye(3)
Kx[0,0] = 500
Kx[1,1] = 500
Kx[2,2] = 500

Dx = np.eye(3)
Dx[0,0] = 300
Dx[1,1] = 300
Dx[2,2] = 0

# Joint-space stiffness				
Ktheta= np.eye(6)*500

# Joint-space damping
Dtheta= np.eye(6)*10

# Joint-space inertia
Mtheta = np.eye(6)*0.1

#Inverse of the joint-space inertia
Mtheta_inv = np.linalg.inv(Mtheta)

## PARAMETERS OF CARTESIAN SINE WAVE
amp=np.matrix([ 75.0, 50.0, 100.0]).T   # amplitude
phi =np.matrix([ 0.0, 0.0, 0.0]).T  # phase
freq=np.matrix([ 1.0, 1.0, 1.0]).T  # frequency

## PARAMETERS OF JOINT SINE WAVE
ampJS = np.matrix([ 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]).T   # amplitude
phiJS = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T   # phase
freqJS = np.matrix([ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]).T  # frequency

# Initial configuration / velocity / Acceleration
q0  = np.matrix([ 0.0, -1, 1, 0.5, 0, 0.5]).T
qd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
qdd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T

# EXERCISE 9: Add external force
# Value of linear external force
#extForce = np.matrix([0.0, 0.0, 200.0]).T
# FLAGS
EXTERNAL_FORCE = False


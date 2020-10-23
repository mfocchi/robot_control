# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os

dt = 0.001              # controller time step
exp_duration = 3.0 	#simulation duration

SLOW_FACTOR = 0.01 	#to slow down simulation

frame_name = 'ee_link'  # name of the frame to control (end-effector)

#PD joint-space controller
## Matrix of gains
kp = np.eye(6)*100  # proportional gains 
kd = np.eye(6)*20   # derivative gains


#PD task-space controller
## Matrix of gains
Kx = np.eye(3)*1000  # proportional gains 
Dx = np.eye(3)*300   # derivative gains


# Initial configuration / velocity / Acceleration
q0  = np.matrix([ 0.0, 0.0, -0.5, 0.5, -1.57, 0.5]).T
qd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
qdd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T

# FLAGS
EXTERNAL_FORCE = True

## PARAMETERS OF FORCE SINE WAVE
amp = np.matrix([ 0.0, 0.0, 50]).T   # amplitude
phi = np.matrix([ 0.0, 0.0, 0.0]).T   # phase
freq = np.matrix([ 0.0, 0.0, 1.0]).T  # frequency


# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os
import sys

# Print options 
np.set_printoptions(precision = 3, linewidth = 200, suppress = True)
np.set_printoptions(threshold=np.inf)
sys.dont_write_bytecode = True
LINE_WIDTH = 60


dt = 0.001                   # controller time step
exp_duration_sin = 3.0 #sine reference duration
exp_duration = 5.0 #simulation duration
num_samples = (int)(exp_duration/dt)


SLOW_FACTOR = 5 #to slow down simulation

frame_name = 'ee_link'    # name of the frame to control (end-effector)

#PD controller
## Matrix of gains
kp = np.eye(6)*200 # proportional gain 
kd = np.eye(6)*20 # derivative gai

## PARAMETERS OF REFERENCE SINUSOIDAL TRAJECTORY
#x0          = np.array([0.6, 0.2, 0.4]).T         # offset
#amp         = np.array([0.1, 0.1, 0.0]).T           # amplitude
#phi         = np.array([0.0, 0.5*np.pi, 0.0]).T     # phase
#freq        = np.array([0.5, 0.5, 0.3]).T           # frequency (time 2 PI)
#

#q0 = np.array([ 0. , -1.0,  0.7,  0. ,  0. ,  0. ])  # initial configuration


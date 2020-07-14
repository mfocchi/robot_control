# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os



LINE_WIDTH = 60


dt = 0.001                   # controller time step
exp_duration_sin = 3.0 #sine reference duration
exp_duration = 5.0 #simulation duration


SLOW_FACTOR = 1 #to slow down simulation

frame_name = 'ee_link'    # name of the frame to control (end-effector)
# EXERCISE 5: contact at the origin of wrist_3_link 
#frame_ee = robot.model.getFrameId('wrist_3_link')

#PD controller
## Matrix of gains
kp = np.eye(6)*200 # proportional gain 
kd = np.eye(6)*20 # derivative gai

## PARAMETERS OF REFERENCE SINUSOIDAL TRAJECTORY
amp = np.matrix([ 0.0, 0.6, 0.0, 0.0, 0.0, 0.0]).T          # amplitude
phi = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T    # phase
freq = np.matrix([ 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]).T  # frequency

# Initial configuration / velocity / Acceleration
q0  = np.matrix([ 0.0, -0.6, 0.6, -1.67, -1.57, 0.0]).T
qd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
qdd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T

# EXERCISE 9: 
#qd0 = np.matrix([ 0, w_rad[1]*amplitude[1], 0.0, 0.0, w_rad[4]*amplitude[4], 0.0]).T
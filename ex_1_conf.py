# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os
import math

dt = 0.001              # controller time step
exp_duration_sin = 3.0 	#sine reference duration
exp_duration = 5.0 	#simulation duration

SLOW_FACTOR = 0.1 	#to slow down simulation

frame_name = 'ee_link'  # name of the frame to control (end-effector)

<<<<<<< HEAD
=======
#PD controller
## Matrix of gains
kp = np.eye(6)*50  # proportional gains 
kd = np.eye(6)*20   # derivative gains

## PARAMETERS OF REFERENCE SINUSOIDAL TRAJECTORY
amp = np.matrix([ 0.0, 0.2, 0.0, 0.0, 0.4, 0.0]).T   # amplitude
phi = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T   # phase
freq = np.matrix([ 0.0, 1.0, 0.0, 0.0, 1.5, 0.0]).T  # frequency


>>>>>>> 89d59ff6b9c2fec6953fda8b8c934b640df0221a
# Initial configuration / velocity / Acceleration
q0 =   np.array([math.pi, -math.pi/8,  -math.pi/6, 0.0])
qd0 =  np.array([    0.0, 0.0, 0.0, 0.0])
qdd0 = np.array([    0.0, 0.0, 0.0, 0.0]) 


<<<<<<< HEAD


=======
# Value of linear external force
extForce = np.matrix([0.0, 0.0, 50.0]).T
# FLAGS
EXTERNAL_FORCE = True
>>>>>>> 89d59ff6b9c2fec6953fda8b8c934b640df0221a


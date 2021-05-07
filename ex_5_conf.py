# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os

LINE_WIDTH = 60

dt = 0.004  # controller time step
exp_duration = 5.0 #simulation duration
CONTINUOUS = False
verbose = False

# Initial configuration / velocity / Acceleration
q0  = np.matrix([ 0.0, -0.6, 0.6, -1.67, -1.57, 0.0]).T
qd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
qdd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T

# Parameters of Joint Reference Trajectories (X,Y, Z, Roll, Pitch, Yaw)
amp                  = np.array([ 0.0, 0.0, 0.03, 0.0, 0.1, 0.0]).T     # amplitude
freq                 = np.array([ 0.0, 0.0, 0.5, 0.0, 1.0, 0.0]).T           # frequency (time 2 PI)
phi                  = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T     # phase

# Gains for the virtual model
gravity = 9.81
Kp_lin_x = 2000
Kp_lin_y = 2000
Kp_lin_z = 2000

Kd_lin_x = 200
Kd_lin_y = 200
Kd_lin_z = 200

KpRoll =  1000
KpPitch = 1000
KpYaw =   1000

KdRoll =  100
KdPitch = 100
KdYaw =   100
    

# Inertia properties
robotMass = 85.446

class robotInertia :
    pass
robotInertia.Ixx = 4.0745
robotInertia.Iyy =  11.3576
robotInertia.Izz = 12.5675
robotInertia.Ixy = 0.1458
robotInertia.Ixz = -0.2245
robotInertia.Iyz =-0.0133
  
Bcom_x = 0.00942549 
Bcom_y = 0 
Bcom_z = -0.0433895

   
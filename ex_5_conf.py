# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os

LINE_WIDTH = 60

dt = 0.004  # controller time step
exp_duration = 4.0 #simulation duration
num_samples = (int)(exp_duration/dt)
CONTINUOUS = False
verbose = False


# Initial configuration / velocity / Acceleration
q0  = np.matrix([ 0.0, -0.6, 0.6, -1.67, -1.57, 0.0]).T
qd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
qdd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T

# Parameters of Joint Reference Trajectories (X,Y, Z, Roll, Pitch, Yaw)
amp                  = np.array([ 0.0, 0.0, 0.05, 0.0, 0.1, 0.0]).T           # amplitude
freq                 = np.array([ 0.0, 0.0, 0.5, 0.0, 1.0, 0.0]).T           # frequency (time 2 PI)
phi                  = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T     # phase


# Gains for the virtual model
gravity = -9.81
Kpcomx = 1500
Kpcomy = 1500
Kpcomz = 1500

Kdcomx = 100
Kdcomy = 100
Kdcomz = 100

KpRoll =  1500
KpPitch = 1500
KpYaw =   1500

KdRoll =  100
KdPitch = 100
KdYaw =   100
    
robotMass = 86.57
Bcom_x = 0 
Bcom_y = 0 
Bcom_z = -0.0433869


robotMass = 85.446
class robotInertia :
    pass
robotInertia.Ixx = 4.0745
robotInertia.Iyy =  11.3576
robotInertia.Izz = 12.5675
robotInertia.Ixy = 0.1458
robotInertia.Ixz = -0.2245
robotInertia.Iyz =-0.0133
  

   
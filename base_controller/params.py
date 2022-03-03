# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np

robot_params = {}
robot_params['hyq'] = {'dt': 0.004, 'kp': 400, 'kd': 6, 'q_0':  np.array([-0.2, 0.7, -1.4, -0.2, 0.7, -1.4, -0.2, -0.7, 1.4, -0.2, -0.7, 1.4]), 'buffer_size': 1501}
robot_params['solo'] ={'dt': 0.002, 'kp': 5., 'kd': 0.1, 'q_0':  np.array([0.,  np.pi/4, -np.pi/2, -0.,  np.pi/4, -np.pi/2, 0., -np.pi/4,  np.pi/2, 0., -np.pi/4,  np.pi/2]), 'buffer_size': 1501} # note the frames are all aligned with base for joints = 0


#note this controller is not able to run more than 350 Hz so do not set dt <0.003
ee_frames = ['lf_foot', 'rf_foot', 'lh_foot','rh_foot']


verbose = False




# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np

robot_params = {}
robot_params['hyq'] = {'dt': 0.004, 'kp': 400, 'kd': 6, 'q_0':  np.array([-0.2, 0.7, -1.4, -0.2, 0.7, -1.4, -0.2, -0.7, 1.4, -0.2, -0.7, 1.4])}
robot_params['solo'] ={'dt': 0.001, 'kp': 10, 'kd': 0.1, 'q_0':  np.array([0.2, 0.7, -1.4, -0.2, 0.7, -1.4, 0.2, -0.7, 1.4, -0.2, -0.7, 1.4])} # note the frames are all aligned with base for joints = 0






# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np

robot_params = {}

robot_params['myrobot'] ={'dt': 0.001,
                        'kp': np.array([10.,   10.,    10.,  10.]),
                        'kd':  np.array([1.,    1.,    1.,   1.  ]),
                        'q_0':  np.array([0, 0, 0, 0]),
                        'joint_names': ['lf_shoulder_pan', 'rf_shoulder_pan',  'lh_shoulder_pan', 'rh_shoulder_pan'],
                        'ee_frames': ['lf_foot', 'rf_foot', 'lh_foot','rh_foot'],
                        'spawn_x': 0.0,
                        'spawn_y': 0.0,
                        'spawn_z': 1.0,
                        'buffer_size': 30001}

verbose = False
plotting = False



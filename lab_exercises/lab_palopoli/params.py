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

robot_params['ur5'] ={'dt': 0.001,
                       'kp': np.array([300, 300, 300,30,30,1]),
                       'kd':  np.array([20,20,20,5, 5,0.5]),
                       #'q_0':  np.array([ 0.3, -1.3, 1.0, -0.7, 0.7, 0.5]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
                       'q_0':  np.array([ -0.3223527113543909,-0.7805794638446351, -2.5675506591796875,-1.6347843609251917, -1.5715253988849085, -1.0017417112933558]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
                       'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
                       'ee_frame': 'tool0',
                       'control_mode': 'point', # 'trajectory','point'
                       'real_robot': False,
                       'control_type': 'position', # 'position', 'torque'
                       'gripper_sim': True,
                       'spawn_x' : 0.5,
                       'spawn_y' : 0.35,
                       'spawn_z' : 1.75,
                       'buffer_size': 6000} # note the frames are all aligned with base for joints = 0

verbose = False
plotting = False



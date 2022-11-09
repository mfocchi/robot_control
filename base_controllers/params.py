# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np

robot_params = {}
robot_params['hyq'] = {'dt': 0.004,
                        'kp': np.array([400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400]),
                        'kd': np.array([6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6]),
                        'q_0':  np.array([-0.2, 0.7, -1.4, -0.2, 0.7, -1.4, -0.2, -0.7, 1.4, -0.2, -0.7, 1.4]),
                        'joint_names': ['lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint', 'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint','lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint', 'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint'],
                        'ee_frames': ['lf_foot', 'rf_foot', 'lh_foot','rh_foot'],
                        'force_th': 50,
                        'spawn_x': 0.0,
                        'spawn_y': 0.0,
                        'spawn_z': 0.8,
                       'buffer_size': 30001}
robot_params['solo'] ={'dt': 0.002,
                       'kp': [5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 5.],
                       'kd': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                       'q_0':  np.array([0,  np.pi/4, -np.pi/2,   -0,  np.pi/4, -np.pi/2, 0, -np.pi/4,  np.pi/2, -0, -np.pi/4,  np.pi/2]),
                       'q_fold': np.array([0,  1.57, -3.13, 0,  -1.57, 3.13, 0,  1.57, -3.13, 0, -1.57, 3.13]),
                       'joint_names': ['lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint', 'rf_haa_joint', 'rf_hfe_joint',
                                       'rf_kfe_joint', 'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint', 'rh_haa_joint',
                                       'rh_hfe_joint', 'rh_kfe_joint'],
                       'ee_frames': ['lf_foot', 'rf_foot', 'lh_foot','rh_foot'],
                       'real_robot': False,
                       'force_th': 2,
                       'spawn_x': 0.25,
                       'spawn_y': 0.0,
                       'spawn_z': 0.3,
                        'buffer_size': 1501} # note the frames are all aligned with base for joints = 0
robot_params['solo_fw'] ={'dt': 0.002,
                        'kp': [5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 0, 0],
                        'kd': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0, 0],
                        'joint_names': ['lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint', 'rf_haa_joint', 'rf_hfe_joint',
                                       'rf_kfe_joint', 'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint', 'rh_haa_joint',
                                       'rh_hfe_joint', 'rh_kfe_joint', 'z_left_wheel_joint', 'z_right_wheel_joint'],
                        'q_0':  np.array([0.,  np.pi/4, -np.pi/2, -0.,  np.pi/4, -np.pi/2, 0., -np.pi/4,  np.pi/2, 0., -np.pi/4,  np.pi/2, 0., 0.]),
                        'ee_frames': ['lf_foot', 'rf_foot', 'lh_foot','rh_foot'],
                        'real_robot': False,
                        'force_th': 4,
                        'spawn_x': 0.0,
                        'spawn_y': 0.0,
                        'spawn_z': 0.3,
                        'buffer_size': 1501} # note the frames are all aligned with base for joints = 0

robot_params['aliengo'] ={'dt': 0.002,
                        'kp': 60.*np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.]),
                        'kd': 10.*np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.]),
                        'kp_real': 15.*np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.]),
                        'kd_real': 0.5*np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.]),
                        #'q_0':  np.array([0.2, 0.7, -1.4, -0.2, 0.7, -1.4, 0.2, 0.7, -1.4, -0.2, 0.7, -1.4]),
                        'q_0':  np.array([0.2, 0.78, -1.7,  0.20, 0.78, -1.7, -0.20, 0.78, -1.7, -0.20, 0.78, -1.7]),
                        #'q_fold': np.array([0.6, 1.7, -2.77, 0.6, 1.7, -2.77, -0.6, 1.52, -2.77, -0.6, 1.52, -2.77]),
                        'q_fold': np.array([0.2, 1.7, -2.7, 0.2, 1.7, -2.7, -0.2, 1.7, -2.7, -0.2, 1.7, -2.7]),
                        'joint_names': ['lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint', 'rf_haa_joint',  'rf_hfe_joint',
                                          'rf_kfe_joint', 'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',  'rh_haa_joint',
                                          'rh_hfe_joint', 'rh_kfe_joint'],
                        'ee_frames': ['lf_foot', 'rf_foot', 'lh_foot','rh_foot'],
                        'real_robot': False,
                        'force_th': 10,
                          'spawn_x': 0.0,
                          'spawn_y': 0.0,
                          'spawn_z': 0.5,
                          'buffer_size': 30001} # note the frames are all aligned with base for joints = 0
robot_params['go1'] ={'dt': 0.002,
                        'kp': 5.*np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1]),
                        'kd': 0.8*np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1]),
                        'kp_real': 20.*np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1]),
                        'kd_real': 0.3*np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1]),
                        'q_0':  np.array([0.2, 0.78, -1.7, -0.20, 0.78, -1.7, 0.20, 0.78, -1.7, -0.20, 0.78, -1.7]),
                        #'q_fold': np.array([0.6, 1.7, -2.77, 0.6, 1.7, -2.77, -0.6, 1.52, -2.77, -0.6, 1.52, -2.77]),
                        'q_fold': np.array([0.2, 1.7, -2.7, 0.2, 1.7, -2.7, -0.2, 1.7, -2.7, -0.2, 1.7, -2.7]),
                        'joint_names': ['lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint', 'rf_haa_joint',  'rf_hfe_joint',
                                          'rf_kfe_joint', 'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',  'rh_haa_joint',
                                          'rh_hfe_joint', 'rh_kfe_joint'],
                        'ee_frames': ['lf_foot', 'rf_foot', 'lh_foot','rh_foot'],
                        'real_robot': False,
                        'force_th': 7.,
                          'spawn_x': 0.0,#0.3,
                          'spawn_y': 0.0,
                          'spawn_z': 1.2,
                          'buffer_size': 30001} # note the frames are all aligned with base for joints = 0
robot_params['ur5'] ={'dt': 0.001, 
                       'kp': np.array([300, 300, 300,30,30,1]),
                       'kd':  np.array([20,20,20,5, 5,0.5]),
                       #'q_0':  np.array([ 0.3, -1.3, 1.0, -0.7, 0.7, 0.5]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
                       'q_0':  np.array([ -0.3223527113543909,-0.7805794638446351, -2.5675506591796875,-1.6347843609251917, -1.5715253988849085, -1.0017417112933558]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
                       'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
                       'ee_frame': 'tool0',
                       'control_mode': 'point', # 'trajectory','point'
                       'real_robot': False,
                       'control_type': 'torque', # 'position', 'torque'
                       'gripper_sim': False,
                       'spawn_x' : 0.5,
                       'spawn_y' : 0.35,
                       'spawn_z' : 1.75,
                       'buffer_size': 6000} # note the frames are all aligned with base for joints = 0

robot_params['jumpleg'] ={'dt': 0.001,
                       'kp': np.array([100, 100, 100, 10, 10, 10 ]),
                       'kd':  np.array([10,10,10, 0.2,0.2,0.2]),
                       'q_0':  np.array([ 0.0, 0.0, 0.25, -0.24010055475883635,0.7092776153747403,-1.4185292429491714]),
                       'joint_names': ['base_x_joint', 'base_y_joint', 'base_z_joint', 'lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint'],
                       'ee_frame': 'lf_foot',
                       'spawn_x' : 0.0,
                       'spawn_y' : 0.0,
                       'spawn_z' : 0.0,
                       'buffer_size': 10000} # note the frames are all aligned with base for joints = 0

robot_params['climbingrobot'] ={'dt': 0.001,
                       'kp': np.array([0 ,    0,    400,  100,    50,   50,    50, 30, 30]),
                       'kd':  np.array([0,    0,    10,   10,     10,   10,     4,   4, 4  ]),

                       'q_0':  np.array([ 0, 0, 8.0, 0,0,0, -1.57, 0.0, 0.0 ]),
                       'joint_names': ['mountain_wire_pitch', 'mountain_wire_roll',  'wire_base_prismatic',
                                       'wire_base_pitch', 'wire_base_roll','wire_base_yaw',
                                       'hip_pitch', 'hip_roll', 'knee'],
                       'ee_frame': 'foot',
                       'spawn_x' : 0.0,
                       'spawn_y' : 0.0,
                       'spawn_z' : 20.0, 
                       'buffer_size': 10000} # note the frames are all aligned with base for joints = 0
                         
verbose = False
plotting = True



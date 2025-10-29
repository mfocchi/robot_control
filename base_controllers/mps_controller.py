# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

from __future__ import print_function
import rospy as ros
from base_controllers.utils.math_tools import *

np.set_printoptions(threshold=np.inf, precision=5, linewidth=1000, suppress=True)
from base_controllers.base_controller import BaseController
from base_controllers.quadruped_controller import QuadrupedController
from base_controllers.utils.common_functions import plotFrame, plotJoint
import params as conf

robotName = "aliengo"  # needs to inherit BaseController

# jessica
import os
from AlienGo_SDK.example_py.MPS_robot_sensors.mps_code import *

# Neural network and configuration imports
from AlienGo_SDK.example_py.MPS_robot_sensors.config_loader import load_config
from AlienGo_SDK.example_py.MPS_robot_sensors.nominal_policy import NominalPolicy
from AlienGo_SDK.example_py.MPS_robot_sensors.backup_trot import load_backup_network, compute_actions
from AlienGo_SDK.example_py.MPS_robot_sensors.utils import scale_axis, quat_rotate_inverse, swap_legs
import sys
import time
import math
import numpy as np
import torch
import rospy
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
import pygame
import threading
import copy


# Remove if controller is not used
# Initialize pygame and the joystick module
# pygame.init()
# pygame.joystick.init()

# Remove if controller is not used
# Check if there is at least one joystick (gamepad) connected
# if pygame.joystick.get_count() == 0:
#    print("No joystick connected")
# else:
#    joystick = pygame.joystick.Joystick(0)  # Get the first joystick
#    joystick.init()
#    print(f"Detected joystick: {joystick.get_name()}")
class Data:
    pass

class MpsSimulator(QuadrupedController):

    def __init__(self, robot_name="myrobot"):
        super().__init__(robot_name=robot_name)
        self.freezeBaseFlag = False
        print("Initialized murobot controller---------------------------------------------------------------")

    def initVars(self):
        super().initVars()
        ## add your variables to initialize here
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']

        # jessica stuff
        ### Configuration and neural network setup
        # Nominal policy
        config_path = os.environ["LOCOSIM_DIR"] + '/robot_control/AlienGo_SDK/example_py/MPS_robot_sensors/config.yaml'
        config = load_config(config_path)

        self.nominal_policy = NominalPolicy(config)
        self.prev_actions = np.zeros(12)  # Store the previous actions

        # Backup policy
        self.backup_trot = load_backup_network(config, config['settings']['tests']['trot_backup'])
        self.scaling_factors = config['scaling_backup']
        self.device = config['networks']['device']
        self.q_def_backup = config['robot']['backup']['default_joint_angles']
        self.backup_trot_policy = config['settings']['tests']['trot_backup']

        self.running_mean = copy.copy(self.backup_trot.running_mean_std.running_mean)
        self.running_var = copy.copy(self.backup_trot.running_mean_std.running_var)
        self.count = copy.copy(self.backup_trot.running_mean_std.count)

        self.scale_backup = config['scaling_backup']['q_des']
        p.current_actions = np.zeros(12)

        self.data = Data()

    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            ## add your logs here
            pass
        super().logData()


def talker(p):
    p.use_gui = False
    additional_args = ['gui:=' + str(p.use_gui), 'go0_conf:=standDown' ]
    p.startController(additional_args=additional_args)
    p.startupProcedure()
    p.q_des = np.copy(p.q_des_q0)
    #switch off wbc
    p.grForcesW_des = np.zeros((12))
    p.tau_ffwd = np.zeros(12)
    is_rec = True
    #reset to 0.01 cause policies need to be eval at that rate
    p.dt = 0.01
    p.rate = ros.Rate(1 / p.dt)
    while not ros.is_shutdown():
        p.updateKinematics()
        #fill in data struct
        p.data.imu_acc = p.baseLinAccB
        p.data.imu_quat = p.quaternion
        p.data.imu_gyro = p.b_R_w @ p.baseTwistW[3:]
        p.data.joint_pos = p.q
        p.data.joint_vel = p.qd
        if p.time < 10: #nominal policy sensor based from Giulio
            p.pid.setPDjoints(conf.robot_params[p.robot_name]['kp_nominal'], conf.robot_params[p.robot_name]['kd_nominal'], np.zeros(p.robot.na))
            qDes_computed, prev_action_computed = p.nominal_policy.compute_qdes(p.data, p.prev_actions)
            p.q_des = qDes_computed
            p.prev_actions = prev_action_computed
        else:
            p.pid.setPDjoints(conf.robot_params[p.robot_name]['kp_backup'], conf.robot_params[p.robot_name]['kd_backup'],  np.zeros(p.robot.na))
            new_actions1 = compute_actions(p.device, p.data, p.scaling_factors, p.current_actions, p.backup_trot, p.backup_trot_policy)
            p.current_actions = swap_legs(new_actions1)
            qDes = p.scale_backup * p.current_actions + np.array(p.q_def_backup)
            p.q_des = qDes
        # publishes /command
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        # log variables
        p.logData()
        # wait for synchronization of the control loop
        p.rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),  3)  # to avoid issues of dt 0.0009999
        p.visualizeContacts()

if __name__ == '__main__':
    p = MpsSimulator(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, joint_names=p.joint_names)



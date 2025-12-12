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
from base_controllers.components.rl_velocity_controller.rl_controller import RlVelocityController
import params as conf

robotName = "aliengo"  # needs to inherit BaseController

# jessica
import os
from AlienGo_SDK.example_py.MPS_robot_sensors.mps_code import *

# Neural network and configuration imports
from AlienGo_SDK.example_py.MPS_robot_sensors.config_loader import load_config
from AlienGo_SDK.example_py.MPS_robot_sensors.nominal_policy import NominalPolicy
from AlienGo_SDK.example_py.MPS_robot_sensors.mps import MPS
from AlienGo_SDK.example_py.MPS_robot_sensors.trot import TrotPolicy
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
        print("Initialized murobot controller---------------------------------------------------------------")

    def initVars(self):
        super().initVars()
        ## add your variables to initialize here
    #    torch.set_num_threads(1)
     #   torch.set_num_interop_threads(1)
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']

        # jessica stuff
        ### Configuration and neural network setup
        # Nominal policy
        config_path = os.environ["LOCOSIM_DIR"] + '/robot_control/AlienGo_SDK/example_py/MPS_robot_sensors/config.yaml'
        config = load_config(config_path)
        self.dt_mps = 0.01
        self.nominal_policy = RlVelocityController(self.robot_name, self.dt_mps)#, 100)#NominalPolicy(config)
        self.nominal_policy.velocity_cmd = np.array([0.1, 0.0, 0.0])
        self.state_estimation = 'ground_truth'  # 'odometry','imu', 'pronto', 'ground_truth' (only sim)
        self.prev_actions = np.zeros(12)  # Store the previous actions

        # Backup policy
        self.device = config['networks']['device']

        backup_trot_policy = config['settings']['tests']['backup_policy']
        self.backup_trot = TrotPolicy(config, backup_trot_policy)
        self.running_mean = copy.copy(self.backup_trot.actor_network.running_mean_std.running_mean)
        self.running_var = copy.copy(self.backup_trot.actor_network.running_mean_std.running_var)
        self.count = copy.copy(self.backup_trot.actor_network.running_mean_std.count)
        self.backup_trot.commands = np.array(config['robot'][backup_trot_policy]['cmd'])

        self.kp_backup = np.array(config['robot'][backup_trot_policy]['kp'])
        self.kd_backup = np.array(config['robot'][backup_trot_policy]['kd'])

        self.current_actions = np.zeros(12)



        self.data = Data()

        self.data.imu_acc = np.zeros(3)
        self.data.imu_quat = np.zeros(4)
        self.data.imu_gyro = np.zeros(3)
        self.data.joint_pos = np.zeros(12)
        self.data.joint_vel = np.zeros(12)

        # MPS loop
        #self.mps = MPS(config, self.data)

    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            ## add your logs here
            pass
        super().logData()


def talker(p):
    p.use_gui = False
    additional_args = ['gui:=' + str(p.use_gui), 'go0_conf:=standDown' ]
    #p.startController(additional_args=additional_args)
    world_name = 'fast.world'

    '''use_joy = True
    if use_joy:
        joy = JoyManager()'''

    #if p.real_robot and p.state_estimation != 'pronto':
    #    print(colored("RL is state_est based need to start state estimation!","red"))
    #    sys.exit()
    #nominal_policy = RlVelocityController(p.robot_name, p.dt)

    p.startController(world_name=world_name,
                      use_ground_truth_contacts=True,
                      additional_args=['gui:=' + str(p.use_gui),
                                       'go0_conf:=standDown'])
    p.startupProcedure()
    #p.q_des = np.copy(p.q_des_q0)
    #switch off wbc
    p.grForcesW_des = np.zeros((12))
    p.tau_ffwd = np.zeros(12)
    #reset to 0.01 because policies need to be eval at that rate
    p.dt = p.dt_mps
    p.rate = ros.Rate(1 / p.dt)
    is_rec = True
    p.pid.setPDjoints(conf.robot_params[p.robot_name]['kp_nominal'], conf.robot_params[p.robot_name]['kd_nominal'],
                      np.zeros(p.robot.na))

    '''p.pid.setPDjoints(p.kp_backup,
                      p.kd_backup,
                      np.zeros(p.robot.na))'''
    start_time = p.time
    first_switch = True
    while not ros.is_shutdown():
        '''if use_joy:
                    axes, buttons = joy.get_commands()
                    #use a scaling to make the joy input less reactive
                    lx = 0.2*axes[0]
                    ly = 0.2*axes[1]
                    ry = 0.2*axes[3]
                    p.backup_trot.commands = np.array([lx, ly, ry])'''
        p.updateKinematics()

        #fill in data struct
        p.data.imu_acc = p.baseLinAccB
        p.data.imu_quat = p.quaternion
        p.data.imu_gyro = p.b_R_w @ p.baseTwistW[3:]
        p.data.joint_pos = p.q
        p.data.joint_vel = p.qd
        p.data.euler = p.euler

        lin_vel_b = p.b_R_w.dot(p.baseTwistW[:3])
        ang_vel_b = p.b_R_w.dot(p.baseTwistW[3:6])
        proj_gravity = p.b_R_w.dot(np.array([0, 0, -1]))

        '''if is_rec:
            is_rec = True#p.mps.isRecSingle(p.data)
            #if first_switch and p.time > start_time + 4:

            #if not is_rec:
            #    p.pid.setPDjoints(conf.robot_params[p.robot_name]['kp_backup'],
            #                      conf.robot_params[p.robot_name]['kd_backup'], np.zeros(p.robot.na))
                # reset data arrays for nominal policy
            #    p.nominal_policy.history_obs = np.zeros((5, 52), dtype=np.float32)
            #    p.nominal_policy.history_est = np.zeros((5, 52), dtype=np.float32)
            #    p.prev_actions = np.zeros(12)
                #p.nominal_policy.prev_action = np.zeros(12)
                #start_time = p.time
        elif not is_rec and p.time > start_time + 4:
            # Switch back to nominal policy
            is_rec = True
            p.pid.setPDjoints(p.nominal_policy.kp,
                              p.nominal_policy.kd,
                              np.zeros(p.robot.na))

            # reset data arrays for backup policy
            p.current_actions = np.zeros(12)
            p.backup_trot.running_mean_std.running_mean = copy.copy(p.running_mean)
            p.backup_trot.running_mean_std.running_var = copy.copy(p.running_var)
            p.backup_trot.running_mean_std.count = copy.copy(p.count)'''

        if p.time < 10:# and is_rec: #nominal policy sensor based from Giulio
            #p.pid.setPDjoints(conf.robot_params[p.robot_name]['kp_nominal'], conf.robot_params[p.robot_name]['kd_nominal'], np.zeros(p.robot.na))
            #p.q_des, p.prev_actions = p.nominal_policy.compute_qdes(p.data, p.prev_actions, p.math_utils)
            #lin_vel_b = p.b_R_w.dot(p.baseTwistW[:3])
            #ang_vel_b = p.b_R_w.dot(p.baseTwistW[3:6])
            #proj_gravity = p.b_R_w.dot(np.array([0, 0, -1]))
            p.q_des = p.nominal_policy.action(lin_vel_b, ang_vel_b, proj_gravity, p.q, p.qd)
        else:
            p.q_des = p.backup_trot.compute_actions(p.quaternion, ang_vel_b, p.q, p.qd)
            p.pid.setPDjoints(p.kp_backup,
                              p.kd_backup,
                              np.zeros(p.robot.na))
            for i in range(4):
                p.q_des[i * 3] = np.clip(p.q_des[i * 3], -1.22, 1.22)  # Hip joint
                p.q_des[i * 3 + 1] = np.clip(p.q_des[i * 3 + 1], 0.0, 1.8)  # Thigh joint
                p.q_des[i * 3 + 2] = np.clip(p.q_des[i * 3 + 2], -2.78, -0.65)  # Calf joint
        # publishes /command
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        # log variables
        p.logData()
        # wait for synchronization of the control loop
        p.rate.sleep()
        p.time = np.round(p.time + np.array([p.dt]),  3)  # to avoid issues of dt 0.0009999
        p.visualizeContacts()

if __name__ == '__main__':
    p = MpsSimulator(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        #plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, joint_names=p.joint_names)
        plotFrame('position', time_log=p.time_log, des_Pose_log=p.basePoseW_des_log, Pose_log=p.basePoseW_log,
                  title='Base', frame='W', sharex=True, sharey=False, start=0, end=-1)
        plotFrame('velocity', time_log=p.time_log, des_Twist_log=p.baseTwistW_des_log, Twist_log=p.baseTwistW_log,
                  title='Base', frame='W', sharex=True, sharey=False, start=0, end=-1)


# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

from __future__ import print_function
import rospy as ros
from base_controllers.utils.math_tools import *
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
from base_controllers.base_controller import BaseController
from base_controllers.quadruped_controller import QuadrupedController
from base_controllers.utils.common_functions import plotFrame, plotJoint
import params as conf
robotName = "aliengo" # needs to inherit BaseController

#jessica
import os
from AlienGo_SDK.example_py.MPS_robot_sensors.mps_code import *
from   AlienGo_SDK.example_py.MPS_robot_sensors import publish_subscribe
# Neural network and configuration imports
from   AlienGo_SDK.example_py.MPS_robot_sensors.config_loader import   load_config, load_actor_network
from   AlienGo_SDK.example_py.MPS_robot_sensors.utils import scale_axis, quat_rotate_inverse, swap_legs
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
#pygame.init()
#pygame.joystick.init()

# Remove if controller is not used
# Check if there is at least one joystick (gamepad) connected
#if pygame.joystick.get_count() == 0:
#    print("No joystick connected")
#else:
#    joystick = pygame.joystick.Joystick(0)  # Get the first joystick
#    joystick.init()
#    print(f"Detected joystick: {joystick.get_name()}")




class GenericSimulator(QuadrupedController):
    
    def __init__(self, robot_name="myrobot"):
        super().__init__(robot_name=robot_name)
        self.freezeBaseFlag = False
        print("Initialized murobot controller---------------------------------------------------------------")

        # Shared variables
        self.lock = threading.Lock()
        self.latest_actions = np.zeros(12)  # Store the latest actions safely across threads
        self.previous_actions = np.zeros(12)  # Store the previous actions
        self.inference_ready = threading.Event()  # Event to signal new inference results
        self.stop_threads = False  # Flag to stop threads gracefully

    def initVars(self):
        super().initVars()
        ## add your variables to initialize here
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']

        #jessica stuff
        ### Configuration and neural network setup
        # Nominal policy
        config_path = os.environ["LOCOSIM_DIR"] + '/robot_control/AlienGo_SDK/example_py/MPS_robot_sensors/config.yaml'
        config = load_config(config_path)
        network_path = os.environ["LOCOSIM_DIR"] + '/robot_control/AlienGo_SDK/example_py/nn/' + \
                       config['nominal']['paths']['checkpoint_path']
        self.actor_network = load_actor_network(config['nominal'], network_path)
        self.scaling_factors = config['nominal']['scaling']
        self.default_joint_angles = config['nominal']['robot']['default_joint_angles']
        self.Kp_n = config['nominal']['robot']['Kp_n']
        self.Kd_n = config['nominal']['robot']['Kd_n']
        self.max_pos = config['nominal']['robot']['max_pos']
        self.min_pos = config['nominal']['robot']['min_pos']
        self.torque_values = config['nominal']['robot']['torque_values']
        self.scaling_qdes = self.scaling_factors['factor']
        # Backup policy
        self.torque_values_b = config['backup']['robot']['torque_values']

        self.d = {'FR_0': 0, 'FR_1': 1, 'FR_2': 2,
             'FL_0': 3, 'FL_1': 4, 'FL_2': 5,
             'RR_0': 6, 'RR_1': 7, 'RR_2': 8,
             'RL_0': 9, 'RL_1': 10, 'RL_2': 11}

        self.legs = ['FR', 'FL', 'RR', 'RL']
        self.joints = ['_0', '_1', '_2']
        self.sin_mid_q = 4 * [0.0, 0.7, -1.5]  # Creates a 12-element list with the default joint angles for the standup


        self.qInit = [0, 0, 0,
                 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0]

        self.qDes = [0, 0, 0,
                0, 0, 0,
                0, 0, 0,
                0, 0, 0]

        self.rate_count = 0

        # PD tuning parameters
        self.Kp = [self.Kp_n, self.Kp_n, self.Kp_n]
        self.Kd = [self.Kd_n, self.Kd_n, self.Kd_n]

        self.actions = torch.zeros(12, dtype=torch.float32)

        # state = sdk.LowState()
        self.motiontime = 0
        self.disable_torques = False  # Flag to disable torques if inclination exceeds threshold or safety button is pressed

        # # Start the inference thread
        # self.threading.Thread(target=self.compute_actions,
        #                  args=(self.pubSub.imu_gyro, self.pubSub.imu_quat, pubSub.joint_pos, pubSub.joint_vel, scaling_factors),
        #                  daemon=True).start()
        self.pubSub = publish_subscribe.PubSub()
        self.pubSub.init_subscribers(config['controller']['topics'])
        self.tau_offset = np.array(config['nominal']['robot']['torque_values'] * 4)

        # Decimation factor to reduce the policy update frequency - Number of control action updates @ sim DT per policy DT
        # Decimation changed to 5 to have a 100 Hz main loop, like in the simulations
        self.decimation = 5
        self.mps = MPS(self.decimation, self.max_pos, self.min_pos, self.tau_offset, self.Kp_n, self.Kd_n,
                       self.scaling_qdes, self.default_joint_angles, self.scaling_factors)
        self.Kp_b = config['backup']['robot']['Kp_b']
        self.Kd_b = config['backup']['robot']['Kd_b']

        print('topics to subscribe to from PubSub', config['controller']['topics'])

    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                ## add your logs here
                pass
            super().logData()

    def compute_observation(self,imu_gyro, imu_quat, joint_pos, joint_vel, scaling_factors, walk):
        """
        Compute the observation vector from the robot's state.
        Legs are swapped to match the order of the neural network input.
        SDK order = [FR, FL, RR, RL]
        nn order = [FL, FR, RL, RR]
        """
        # Remove if the controller is not used
        # commands = get_commands() # The stopping condition here is not evaluated

        # Add if the controller is not used
        if walk:
            commands = np.array([0.5, 0., 0.])  # The stopping condition here is not evaluated
        else:
            commands = np.array([-0.35, -0.1, 0.])
        # imu = state.imu
        # body_quat = np.array([imu.quaternion[1], imu.quaternion[2], imu.quaternion[3], imu.quaternion[0]])
        body_quat = imu_quat
        # body_vel = np.array([imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]])
        body_vel = imu_gyro
        joint_angles = joint_pos
        joint_velocities = joint_vel

        # Gravity vector in body frame
        gravity_body = quat_rotate_inverse(
            torch.tensor(body_quat, dtype=torch.float32).unsqueeze(0),
            torch.tensor([[0.0, 0.0, -1.0]], dtype=torch.float32)
        ).squeeze().numpy()

        prev_actions1 = np.copy(self.previous_actions)
        prev_actions = swap_legs(prev_actions1)

        # Scale observations
        scaled_body_vel = body_vel * scaling_factors['body_ang_vel']
        scaled_commands = commands[:2] * scaling_factors['commands']
        scaled_commands = np.append(scaled_commands, commands[2] * scaling_factors['body_ang_vel'])
        scaled_gravity_body = gravity_body * scaling_factors['gravity_body']
        scaled_joint_angles = np.array(joint_angles) * scaling_factors['joint_angles']
        scaled_joint_velocities = np.array(joint_velocities) * scaling_factors['joint_velocities']
        scaled_actions = prev_actions * scaling_factors['actions']

        # Concatenate into a single observation vector
        return np.concatenate((scaled_body_vel, scaled_commands, scaled_gravity_body, scaled_joint_angles,
                               scaled_joint_velocities, scaled_actions))

    def compute_actions(self,imu_gyro, imu_quat, joint_pos, joint_vel, scaling_factors, is_rec):
        """
        Inference on the NN to retrive actions from observations.
        Legs are swapped to match the order of the neural network input.
        SDK order = [FR, FL, RR, RL]
        nn order = [FL, FR, RL, RR]
        """
        #print('Computing actions')
        # start_time = time.time()
        obs = self.compute_observation(imu_gyro, imu_quat, joint_pos, joint_vel, scaling_factors, is_rec)
        obs_tensor = torch.tensor(obs, dtype=torch.float32)
        obs_normalized = self.actor_network.norm_obs(obs_tensor)

        with torch.no_grad():
            new_actions1 = self.actor_network(obs_normalized).numpy()

        # Swap the actions to the correct order for SDK/Pinocchio
        new_actions = swap_legs(new_actions1)
        self.previous_actions = self.latest_actions
        self.latest_actions = new_actions

        """ print(f"Inference completed in: {time.time() - start_time:.5f} seconds") """

    def jointLinearInterpolation(self,initPos, targetPos, rate):
        """
        Performs a linear interpolation between initial and target joint positions.
        """
        rate = np.fmin(np.fmax(rate, 0.0), 1.0)
        p = initPos * (1 - rate) + targetPos * rate
        return p

    def check_safety_stops(self,imu_quat):
        """
        Check if the inclination of the robot base exceeds the threshold (pi/8) and checks the safety button as well.
        """
        # imu = state.imu
        body_quat = imu_quat  # imu.quaternion  # Quaternion from qpos
        # Calculate inclination using arcsin formula
        inclination = 2 * np.arcsin(np.sqrt(body_quat[1] ** 2 + body_quat[2] ** 2))

        stop_button = get_safety_button()  # Check if the safety button is pressed

        # if pygame.joystick.get_count() != 1:
        #     return True

        if stop_button:  # inclination > np.pi/8:# or stop_button:
            return True
        else:
            return False

def talker(p):
    p.use_gui = False
    additional_args = ['gui:=' + str(p.use_gui),
                       'go0_conf:=standDown']
    p.startController(additional_args=additional_args)
    #p.start()
    # p.startSimulator(additional_args = additional_args)
    # p.loadModelAndPublishers()
    # p.initSubscribers()
    p.initVars()
    p.startupProcedure()

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])
    p.q_des = np.copy(p.q_des_q0)
    p.tau_ffwd = p.tau_offset

    is_rec = True

    while not ros.is_shutdown():
      #  print(p.pubSub.effort)
      #  p.motiontime += 1
      #  if p.motiontime % p.decimation == 0:
         # Trigger inference every `decimation` steps
        latest_actions_ant = np.copy(p.latest_actions)
        previous_actions_ant = np.copy(p.previous_actions)
        p.compute_actions(p.pubSub.imu_gyro, p.pubSub.imu_quat, p.pubSub.joint_pos, p.pubSub.joint_vel,
                          p.scaling_factors, is_rec)

        # Get the latest available actions
        qDes = p.scaling_qdes * np.copy(p.latest_actions) + np.array(p.default_joint_angles)

    # Clip the joint angles to the joint limits
        qDes = np.clip(qDes, p.min_pos, p.max_pos)

        if is_rec:
            is_rec, iter_mps = p.mps.is_rec_single(qDes, p.pubSub.pose, p.pubSub.twist, p.pubSub.joint_pos, p.pubSub.joint_vel,
                                                   copy.deepcopy(p.actor_network), np.copy(p.previous_actions),
                                               np.copy(p.latest_actions))
            #is_rec=True
            if not is_rec:
                p.latest_actions = np.copy(latest_actions_ant)
                p.previous_actions = np.copy(previous_actions_ant)
                p.compute_actions(p.pubSub.imu_gyro, p.pubSub.imu_quat, p.pubSub.joint_pos, p.pubSub.joint_vel,
                                  p.scaling_factors, is_rec)

                qDes = p.scaling_qdes * np.copy(p.latest_actions) + np.array(p.default_joint_angles)
                qDes = np.clip(qDes, p.min_pos, p.max_pos)

        p.q_des = qDes
        #publishes /command
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

        # log variables
        p.logData()

        # wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999

if __name__ == '__main__':
    p = GenericSimulator(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, joint_names = p.joint_names)



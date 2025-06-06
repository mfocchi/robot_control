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
from base_controllers.utils.common_functions import plotFrame, plotJoint
import params as conf
robotName = "go1" # needs to inherit BaseController

#jessica
import os
from AlienGo_SDK.example_py.MPS_robot_sensors.mps_code import *
import  AlienGo_SDK.example_py.MPS_robot_sensors.publish_subscribe
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

### Configuration and neural network setup
# Nominal policy
config_path = os.environ["LOCOSIM_DIR"]+'/robot_control/AlienGo_SDK/example_py/MPS_robot_sensors/config.yaml'
config = load_config(config_path)
network_path = os.environ["LOCOSIM_DIR"]+'/robot_control/AlienGo_SDK/example_py/nn/' + config['nominal']['paths']['checkpoint_path']
actor_network = load_actor_network(config['nominal'], network_path)
scaling_factors = config['nominal']['scaling']
default_joint_angles = config['nominal']['robot']['default_joint_angles']
Kp_n = config['nominal']['robot']['Kp_n']
Kd_n = config['nominal']['robot']['Kd_n']
max_pos = config['nominal']['robot']['max_pos']
min_pos = config['nominal']['robot']['min_pos']
torque_values = config['nominal']['robot']['torque_values']
scaling_qdes = scaling_factors['factor']

# Backup policy
torque_values_b = config['backup']['robot']['torque_values']


class GenericSimulator(BaseController):
    
    def __init__(self, robot_name="myrobot"):
        super().__init__(robot_name=robot_name, external_conf = conf)
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
        self.Kp = [Kp_n, Kp_n, Kp_n]
        self.Kd = [Kd_n, Kd_n, Kd_n]

        self.actions = torch.zeros(12, dtype=torch.float32)

        # Decimation factor to reduce the policy update frequency - Number of control action updates @ sim DT per policy DT
        # Decimation changed to 5 to have a 100 Hz main loop, like in the simulations
        self.decimation = 5
        self.mps = mps_code.MPS(decimation, max_pos, min_pos, torque_values, Kp_n, Kd_n, config['backup'])
        self.Kp_b = mps.Kp_b
        self.Kd_b = mps.Kd_b

        # state = sdk.LowState()
        self.motiontime = 0
        self.disable_torques = False  # Flag to disable torques if inclination exceeds threshold or safety button is pressed

        # # Start the inference thread
        # self.threading.Thread(target=self.compute_actions,
        #                  args=(self.pubSub.imu_gyro, self.pubSub.imu_quat, pubSub.joint_pos, pubSub.joint_vel, scaling_factors),
        #                  daemon=True).start()

    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                ## add your logs here
                pass
            super().logData()

    def compute_observation(self,imu_gyro, imu_quat, joint_pos, joint_vel, scaling_factors):
        """
        Compute the observation vector from the robot's state.
        Legs are swapped to match the order of the neural network input.
        SDK order = [FR, FL, RR, RL]
        nn order = [FL, FR, RL, RR]
        """
        # Remove if the controller is not used
        # commands = get_commands() # The stopping condition here is not evaluated

        # Add if the controller is not used
        commands = np.array([0, 0, 0])  # The stopping condition here is not evaluated

        # imu = state.imu
        # body_quat = np.array([imu.quaternion[1], imu.quaternion[2], imu.quaternion[3], imu.quaternion[0]])
        body_quat = np.array([imu_quat[1], imu_quat[2], imu_quat[3], imu_quat[0]])
        # body_vel = np.array([imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]])
        body_vel = imu_gyro
        joint_angles1 = joint_pos  # [state.motorState[i].q for i in range(12)]
        joint_angles = swap_legs(joint_angles1)
        joint_velocities1 = joint_vel  # [state.motorState[i].dq for i in range(12)]
        joint_velocities = swap_legs(joint_velocities1)

        # Gravity vector in body frame
        gravity_body = quat_rotate_inverse(
            torch.tensor(body_quat, dtype=torch.float32).unsqueeze(0),
            torch.tensor([[0.0, 0.0, -1.0]], dtype=torch.float32)
        ).squeeze().numpy()

        prev_actions1 = np.copy(previous_actions)
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

    def compute_actions(self,imu_gyro, imu_quat, joint_pos, joint_vel, scaling_factors):
        """
        Inference on the NN to retrive actions from observations.
        Legs are swapped to match the order of the neural network input.
        SDK order = [FR, FL, RR, RL]
        nn order = [FL, FR, RL, RR]
        """
        global latest_actions, previous_actions, stop_threads
        while not stop_threads:
            # start_time = time.time()

            inference_ready.wait()  # Wait for signal from the main thread
            inference_ready.clear()

            obs = compute_observation(imu_gyro, imu_quat, joint_pos, joint_vel, scaling_factors)
            obs_tensor = torch.tensor(obs, dtype=torch.float32)
            obs_normalized = actor_network.norm_obs(obs_tensor)

            with torch.no_grad():
                new_actions1 = actor_network(obs_normalized).numpy()

            # Swap the actions to the correct order for SDK
            new_actions = swap_legs(new_actions1)

            with lock:
                previous_actions[:] = latest_actions  # Store current actions as previous
                latest_actions[:] = new_actions  # Update latest actions

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
    p.start()
    additional_args = None
    p.startSimulator(additional_args = additional_args)
    p.loadModelAndPublishers()
    p.initSubscribers()

    p.initVars()
    p.startupProcedure()

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])
    p.q_des = np.copy(p.q_des_q0)

    while not ros.is_shutdown():
        p.tau_ffwd = np.zeros(p.robot.na)

        # Trigger inference every `decimation` steps
        self.compute_actions(self.pubSub.imu_gyro, self.pubSub.imu_quat, self.pubSub.joint_pos, self.pubSub.joint_vel, scaling_factors)

        # Get the latest available actions
        with lock:
            current_actions = np.copy(latest_actions)
        qDes = scaling_qdes * current_actions + np.array(default_joint_angles)
        # Clip the joint angles to the joint limits
        qDes = np.clip(qDes, min_pos, max_pos)

        #p.q_des = p.q_des_q0  + 0.3 * np.sin(2*np.pi*0.5*p.time)
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



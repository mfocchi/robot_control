# Description: Wrapper of the locomotion policy

# Authors:
# Giulio Turrisi

import time
import copy
import numpy as np

np.set_printoptions(precision=3, suppress=True)

from tqdm import tqdm
import mujoco
import onnxruntime as ort
import torch

import base_controllers.components.rl_velocity_controller.config as config
from base_controllers.components.rl_velocity_controller.supervised_learning_networks import  load_network
from base_controllers.components.rl_velocity_controller.legs_attr import LegsAttr
import sys
import os

dir_path = os.path.dirname(os.path.realpath(__file__))


class LocomotionPolicyWrapper:
    def __init__(self, q_home):

        self.policy = ort.InferenceSession(config.policy_folder_path + "/exported/policy.onnx")
        self.Kp_walking = config.Kp_walking
        self.Kd_walking = config.Kd_walking
        self.Kp_stand_up_and_down = config.Kp_stand_up_and_down
        self.Kd_stand_up_and_down = config.Kd_stand_up_and_down

        self.RL_FREQ = 1. / (config.training_env["sim"]["dt"] * config.training_env[
            "decimation"])  # Hz, frequency of the RL controller

        # RL controller initialization -------------------------------------------------------------
        self.action_scale = config.training_env["action_scale"]
        self.rl_actions = LegsAttr(*[np.zeros((1, int(12 / 4))) for _ in range(4)])
        self.past_rl_actions = np.zeros(12)

        q_home_arr = np.array(q_home).flatten()
        # Split into 4 legs of 3 joints each
        FLhome, RLhome, FRhome,  RRhome = np.split(q_home_arr, 4)
        self.default_joint_pos = LegsAttr(FLhome, FRhome, RLhome, RRhome)


        # Observation space initialization -------------------------------------------------------
        self.observation_space = config.training_env["single_observation_space"]

        self.use_clock_signal = config.training_env["use_clock_signal"]

        self.step_freq = 1.4
        self.duty_factor = 0.65
        self.phase_offset = np.array([0.0, 0.5, 0.5, 0.0])
        self.phase_signal = self.phase_offset

        self.desired_clip_actions = config.training_env["desired_clip_actions"]

        self.use_filter_actions = config.training_env["use_filter_actions"]

        self.use_observation_history = config.training_env["use_observation_history"]
        self.history_length = config.training_env["history_length"]
        if (self.use_observation_history):
            self.observation_space = self.observation_space * self.history_length
        single_observation_space = int(self.observation_space / self.history_length)
        self._observation_history = np.zeros((self.history_length, single_observation_space), dtype=np.float32)

        self.use_vision = config.use_vision


        # Learned State Estimator
        if (config.training_env["use_cuncurrent_state_est"] == True):
            self._cuncurrent_state_est_network = load_network(config.cuncurrent_state_est_network, device='cpu')
            self._observation_history_cuncurrent_state_est = np.zeros((self.history_length, single_observation_space),
                                                                      dtype=np.float32)

        # Desired joint vector
        self.desired_joint_pos = LegsAttr(*[np.zeros((1, int(12 / 4))) for _ in range(4)])

    def _get_projected_gravity(self, quat_wxyz):
        # Normalize gravity vector
        GRAVITY_VEC_W = np.array([0.0, 0.0, -9.81])
        GRAVITY_VEC_W /= np.linalg.norm(GRAVITY_VEC_W)

        # Ensure quaternion is numpy array

        q_w = quat_wxyz[0]
        q_vec = quat_wxyz[1:]

        v = GRAVITY_VEC_W

        # Compute components
        a = v * (2.0 * q_w ** 2 - 1.0)
        b = np.cross(q_vec, v) * q_w * 2.0
        c = q_vec * (np.dot(q_vec, v)) * 2.0

        projected_gravity = a - b + c
        return projected_gravity.flatten()

    def compute_action(self,
                        base_quat_wxyz = np.array([1, 0,0,0]),
                        base_linear_velocity= np.zeros(3),
                        imu_angular_velocity= np.zeros(3),
                        imu_linear_acceleration= np.zeros(3),
                        h_R_b= np.eye(3),
                        joints_pos = np.zeros(12),
                        joints_vel= np.zeros(12),
                        ref_base_lin_vel= np.zeros(3),
                        ref_base_ang_vel= np.zeros(3)):

        # Update Observation ----------------------

        base_projected_gravity = self._get_projected_gravity(base_quat_wxyz)

        base_vel = imu_linear_acceleration
        base_ang_vel = imu_angular_velocity

        # Get the reference base velocity in the world frame
        ref_base_lin_vel_h = h_R_b @ ref_base_lin_vel

        joints_pos_arr = np.array(joints_pos).flatten()
        # Split into 4 legs of 3 joints each
        FL, RL, FR, RR = np.split(joints_pos_arr, 4)
        joints_pos = LegsAttr(FL, FR, RL, RR)



        joints_vel_arr =  np.array(joints_vel).flatten()
        FLvel, RLvel, FRvel, RRvel = np.split(joints_vel_arr, 4)
        joints_vel = LegsAttr(FLvel, FRvel, RLvel, RRvel)


        # Fill the observation vector
        joints_pos_delta = joints_pos - self.default_joint_pos
        obs = np.concatenate([
            base_vel,  # this could be imu linear acc if use_imu or linear vel from state est
            base_ang_vel,
            base_projected_gravity,
            ref_base_lin_vel_h[0:2],
            [ref_base_ang_vel[2]],
            [joints_pos_delta.FL[0]], [joints_pos_delta.FR[0]], [joints_pos_delta.RL[0]], [joints_pos_delta.RR[0]],
            [joints_pos_delta.FL[1]], [joints_pos_delta.FR[1]], [joints_pos_delta.RL[1]], [joints_pos_delta.RR[1]],
            [joints_pos_delta.FL[2]], [joints_pos_delta.FR[2]], [joints_pos_delta.RL[2]], [joints_pos_delta.RR[2]],
            [joints_vel.FL[0]], [joints_vel.FR[0]], [joints_vel.RL[0]], [joints_vel.RR[0]],
            [joints_vel.FL[1]], [joints_vel.FR[1]], [joints_vel.RL[1]], [joints_vel.RR[1]],
            [joints_vel.FL[2]], [joints_vel.FR[2]], [joints_vel.RL[2]], [joints_vel.RR[2]],
            self.past_rl_actions.copy(),

        ])

        # Phase Signal
        if (self.use_clock_signal):
            self.phase_signal += self.step_freq * (1 / (self.RL_FREQ))
            self.phase_signal = self.phase_signal % 1.0
            obs = np.concatenate((obs, self.phase_signal), axis=0)

            commands = np.array([ref_base_lin_vel_h[0], ref_base_lin_vel_h[1], ref_base_ang_vel[2]], dtype=np.float32)
            if (np.linalg.norm(commands) < 0.01):
                obs[48:52] = -1.0


        if (config.training_env["use_cuncurrent_state_est"] == True):
            # the bottom element is the newest observation!!
            past_cuncurrent_state_est = self._observation_history_cuncurrent_state_est[1:, :]
            self._observation_history_cuncurrent_state_est = np.vstack((past_cuncurrent_state_est, copy.deepcopy(obs)))
            obs_cuncurrent_state_est = self._observation_history_cuncurrent_state_est.flatten()
            # QUERY THE NETOWRK
            base_lin_vel_predicted = self._cuncurrent_state_est_network(
                torch.tensor(obs_cuncurrent_state_est, dtype=torch.float32).unsqueeze(0)).detach().numpy().squeeze()
            obs[0:3] = base_linear_velocity

        if (self.use_observation_history):
            # the bottom element is the newest observation!!
            past = self._observation_history[1:, :]
            self._observation_history = np.vstack((past, copy.deepcopy(obs)))
            obs = self._observation_history.flatten()



        # RL Prediction
        obs = obs.reshape(1, -1)
        obs = obs.astype(np.float32)
        rl_action_temp = self.policy.run(None, {'obs': obs})[0][0]
        rl_action_temp = np.clip(rl_action_temp, -self.desired_clip_actions, self.desired_clip_actions)

        # Action Filtering
        if (self.use_filter_actions):
            alpha = 0.8
            past_rl_actions_temp = self.past_rl_actions.copy()
            self.past_rl_actions = rl_action_temp.copy()
            rl_action_temp = alpha * rl_action_temp + (1 - alpha) * past_rl_actions_temp
        else:
            self.past_rl_actions = rl_action_temp.copy()

        self.rl_actions.FL = np.array([rl_action_temp[0], rl_action_temp[4], rl_action_temp[8]])
        self.rl_actions.FR = np.array([rl_action_temp[1], rl_action_temp[5], rl_action_temp[9]])
        self.rl_actions.RL = np.array([rl_action_temp[2], rl_action_temp[6], rl_action_temp[10]])
        self.rl_actions.RR = np.array([rl_action_temp[3], rl_action_temp[7], rl_action_temp[11]])

        # Impedence Loop
        self.desired_joint_pos.FL = self.default_joint_pos.FL + self.rl_actions.FL * self.action_scale
        self.desired_joint_pos.FR = self.default_joint_pos.FR + self.rl_actions.FR * self.action_scale
        self.desired_joint_pos.RL = self.default_joint_pos.RL + self.rl_actions.RL * self.action_scale
        self.desired_joint_pos.RR = self.default_joint_pos.RR + self.rl_actions.RR * self.action_scale

        return np.concatenate([self.desired_joint_pos.FL, self.desired_joint_pos.RL, self.desired_joint_pos.FR, self.desired_joint_pos.RR])

if __name__ == '__main__':
    q_0 =  np.array([0.0951, 0.8303, -1.5419,
                     0.0980, 0.9864, -1.4778,
                     -0.0948, 0.8305, -1.5420,
                     -0.0979, 0.9864, -1.4779]),
    agent = LocomotionPolicyWrapper(q_home=q_0)
    des_joints = agent.compute_action(base_quat_wxyz=np.array([1.,0,0,0]),
                                  imu_angular_velocity=np.zeros(3),
                                  imu_linear_acceleration=np.zeros(3),
                                h_R_b=np.eye(3),
                                joints_pos=q_0,
                                joints_vel=np.zeros(12),
                                ref_base_lin_vel=np.array([0., 0.0, 0.0]),
                                ref_base_ang_vel=np.zeros(3))
    print(des_joints)

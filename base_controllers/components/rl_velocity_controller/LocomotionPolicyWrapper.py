# Description: Wrapper of the locomotion policy

# Authors:
# Giulio Turrisi

import time
import copy
import numpy as np
np.set_printoptions(precision=3, suppress=True)
import onnxruntime as ort
import torch
import base_controllers.components.rl_velocity_controller.config as config
from base_controllers.components.rl_velocity_controller.supervised_learning_networks import  load_network
from base_controllers.components.rl_velocity_controller.legs_attr import LegsAttr
import sys
import os
dir_path = os.path.dirname(os.path.realpath(__file__))

class LocomotionPolicyWrapper:
    def __init__(self, use_state_est = True, dt = 0.02):
        self.dt = dt
        self.counter = 0
        self.use_state_est = use_state_est
        self.policy = ort.InferenceSession(config.policy_folder_path + "/exported/policy.onnx")

        self.kp = np.full(12, config.Kp_walking)
        self.kd = np.full(12, config.Kd_walking)
        #50Hz
        self.RL_FREQ = 1./(config.training_env["sim"]["dt"]*config.training_env["decimation"])  # Hz, frequency of the RL controller

        # RL controller initialization -------------------------------------------------------------
        self.action_scale = config.training_env["action_scale"]
        self.rl_actions = LegsAttr(*[np.zeros((1, int(12/4))) for _ in range(4)])
        self.past_rl_actions = np.zeros(12)

        self.default_joint_pos = LegsAttr(*[np.zeros((1, int(12/4))) for _ in range(4)])
        self.default_joint_pos.FL = np.array([ 0.,   0.9, -1.8])
        self.default_joint_pos.FR = np.array([ 0.,   0.9, -1.8])
        self.default_joint_pos.RL = np.array([ 0.,   0.9, -1.8])
        self.default_joint_pos.RR = np.array([ 0.,   0.9, -1.8])

        self.joints_pos = LegsAttr(*[np.zeros((1, int(12/4))) for _ in range(4)])
        self.joints_vel = LegsAttr(*[np.zeros((1, int(12/4))) for _ in range(4)])

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
        if(self.use_observation_history):
            self.observation_space = self.observation_space * self.history_length
        single_observation_space = int(self.observation_space/self.history_length)
        self._observation_history = np.zeros((self.history_length, single_observation_space), dtype=np.float32)

        # RMA
        if(config.training_env["use_rma"] == True):
            self._rma_network = load_network(config.rma_network_path, device='cpu')
            self._observation_history_rma = np.zeros((self.history_length, single_observation_space), dtype=np.float32)

        # Learned State Estimator
        if(config.training_env["use_cuncurrent_state_est"] == True):
            self._cuncurrent_state_est_network = load_network(config.cuncurrent_state_est_network, device='cpu')
            self._observation_history_cuncurrent_state_est = np.zeros((self.history_length, single_observation_space), dtype=np.float32)


        # Desired joint vector
        self.desired_joint_pos = LegsAttr(*[np.zeros((1, int(12/4))) for _ in range(4)])
        self.action = np.zeros(12)

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


    def compute_control(self,
            h_R_b,
            joints_pos,
            joints_vel,
            ref_base_lin_vel,
            ref_base_ang_vel,
            imu_linear_acceleration,
            imu_angular_velocity,
            imu_orientation):

        if np.mod(self.counter, int((1/self.dt)/self.RL_FREQ) ) == 0:
            # print("heading_orientation_SO3",heading_orientation_SO3)
            # print("imu_linear_acceleration",imu_linear_acceleration)
            # print("imu_angular_velocity",imu_angular_velocity)
            # print("ref_base_lin_vel",ref_base_lin_vel)
            # print("ref_base_ang_vel",ref_base_ang_vel)
            # print("imu_orientation", imu_orientation)

            # heading_orientation_SO3 [[ 1.    -0.006  0.   ]
            #  [ 0.006  1.     0.   ]
            #  [ 0.     0.     1.   ]]
            # imu_linear_acceleration [-0.553  0.005  9.791]
            # imu_angular_velocity [0.    0.001 0.001]
            # ref_base_lin_vel [0. 0. 0.]
            # ref_base_ang_vel [0. 0. 0.]
            # imu_orientation [1.    0.    0.028 0.003]

            #map from pinocchio convention
            self.joints_pos.FL = joints_pos[:3]
            self.joints_pos.FR = joints_pos[6:9]
            self.joints_pos.RL = joints_pos[3:6]
            self.joints_pos.RR = joints_pos[9:12]

            self.joints_vel.FL = joints_vel[:3]
            self.joints_vel.FR = joints_vel[6:9]
            self.joints_vel.RL = joints_vel[3:6]
            self.joints_vel.RR = joints_vel[9:12]

            #joint pos
            # print(joints_pos)
            #FL = [0.076  0.919 - 1.821], FR = [-0.071  0.939 - 1.822], RL = [0.049  1.003 - 1.653], RR = [-0.033 1.011 - 1.66]

            #print(joints_vel)
            #FL = [0.032  0.013 - 0.002], FR = [-0.037  0.019 - 0.], RL = [0.02  0.045 0.048], RR = [-0.021  0.047  0.047]


            # Update Observation ----------------------
            base_projected_gravity = self._get_projected_gravity(imu_orientation)

            # Get the reference base velocity from base frame in the horizontal frame
            ref_base_lin_vel_h = h_R_b @ref_base_lin_vel

            # Fill the observation vector
            joints_pos_delta = self.joints_pos - self.default_joint_pos


            obs = np.concatenate([
                imu_linear_acceleration, # this could be imu linear acc if use_imu or linear vel from state est
                imu_angular_velocity,
                base_projected_gravity,
                ref_base_lin_vel_h[0:2],
                [ref_base_ang_vel[2]],
                [joints_pos_delta.FL[0]], [joints_pos_delta.FR[0]], [joints_pos_delta.RL[0]], [joints_pos_delta.RR[0]],
                [joints_pos_delta.FL[1]], [joints_pos_delta.FR[1]], [joints_pos_delta.RL[1]], [joints_pos_delta.RR[1]],
                [joints_pos_delta.FL[2]], [joints_pos_delta.FR[2]], [joints_pos_delta.RL[2]], [joints_pos_delta.RR[2]],
                [self.joints_vel.FL[0]], [self.joints_vel.FR[0]], [self.joints_vel.RL[0]], [self.joints_vel.RR[0]],
                [self.joints_vel.FL[1]], [self.joints_vel.FR[1]], [self.joints_vel.RL[1]], [self.joints_vel.RR[1]],
                [self.joints_vel.FL[2]], [self.joints_vel.FR[2]], [self.joints_vel.RL[2]], [self.joints_vel.RR[2]],
                self.past_rl_actions.copy(),
            ])


            # Phase Signal
            if(self.use_clock_signal):
                self.phase_signal += self.step_freq * (1 / (self.RL_FREQ))
                self.phase_signal = self.phase_signal % 1.0
                obs = np.concatenate((obs, self.phase_signal), axis=0)
                commands = np.array([ref_base_lin_vel_h[0], ref_base_lin_vel_h[1], ref_base_ang_vel[2]], dtype=np.float32)
                if(np.linalg.norm(commands) < 0.01):
                    obs[48:52] = -1.0

            if(self.use_state_est == True):
                #the bottom element is the newest observation!!
                past_cuncurrent_state_est = self._observation_history_cuncurrent_state_est[1:,:]
                self._observation_history_cuncurrent_state_est = np.vstack((past_cuncurrent_state_est, copy.deepcopy(obs)))
                obs_cuncurrent_state_est = self._observation_history_cuncurrent_state_est.flatten()
                # QUERY THE NETOWRK
                base_lin_vel_predicted = self._cuncurrent_state_est_network(torch.tensor(obs_cuncurrent_state_est, dtype=torch.float32).unsqueeze(0)).detach().numpy().squeeze()
                obs[0:3] = base_lin_vel_predicted
            else:
                obs[0:3] = np.zeros(3)

            if(self.use_observation_history):
                #the bottom element is the newest observation!!
                past = self._observation_history[1:,:]
                self._observation_history = np.vstack((past, copy.deepcopy(obs)))
                obs = self._observation_history.flatten()


            # RL Prediction
            obs = obs.reshape(1, -1)
            obs = obs.astype(np.float32)
            rl_action_temp = self.policy.run(None, {'obs': obs})[0][0]
            rl_action_temp = np.clip(rl_action_temp, -self.desired_clip_actions, self.desired_clip_actions)


            # Action Filtering
            if(self.use_filter_actions):
                alpha = 0.8
                past_rl_actions_temp = self.past_rl_actions.copy()
                self.past_rl_actions = rl_action_temp.copy()
                rl_action_temp = alpha * rl_action_temp + (1-alpha) * past_rl_actions_temp
            else:
                self.past_rl_actions = rl_action_temp.copy()


            self.rl_actions.FL = np.array([rl_action_temp[0], rl_action_temp[4], rl_action_temp[8]])
            self.rl_actions.FR = np.array([rl_action_temp[1], rl_action_temp[5], rl_action_temp[9]])
            self.rl_actions.RL = np.array([rl_action_temp[2], rl_action_temp[6], rl_action_temp[10]])
            self.rl_actions.RR = np.array([rl_action_temp[3], rl_action_temp[7], rl_action_temp[11]])


            # Impedence Loop
            self.desired_joint_pos.FL = self.default_joint_pos.FL + self.rl_actions.FL*self.action_scale
            self.desired_joint_pos.FR = self.default_joint_pos.FR + self.rl_actions.FR*self.action_scale
            self.desired_joint_pos.RL = self.default_joint_pos.RL + self.rl_actions.RL*self.action_scale
            self.desired_joint_pos.RR = self.default_joint_pos.RR + self.rl_actions.RR*self.action_scale

            #print("desired_joint_pos", self.desired_joint_pos)
            #FL = [-0.102  0.778 - 1.323], FR = [0.094  0.795 - 1.341], RL = [-0.023  1.034 - 1.252], RR = [0.03
            #                                                                                               1.033 - 1.25]

            #map back to pinocchio
            self.action = np.concatenate((self.desired_joint_pos.FL, self.desired_joint_pos.RL,self.desired_joint_pos.FR,self.desired_joint_pos.RR))
        self.counter+=1
        return self.action

if __name__ == '__main__':
    agent = LocomotionPolicyWrapper(use_state_est = False)
    joints_pos = np.array([0.076,  0.919, -1.821,  #LF
                           0.049,  1.003, -1.653,   #LH
                           -0.071,  0.939, -1.822, #RF
                           -0.033, 1.011, -1.66]) #RH

    joints_vel = np.zeros(12)

    desired_joint_pos = agent.compute_control(h_R_b=np.eye(3),
            joints_pos=joints_pos,
            joints_vel=joints_vel,
            ref_base_lin_vel=np.zeros(3),
            ref_base_ang_vel=np.zeros(3),
            imu_linear_acceleration=np.array([0, 0,9.81]),
            imu_angular_velocity=np.zeros(3),
            imu_orientation=np.array([1.,0,0,0]))
    print(desired_joint_pos)
    #desired_joint_pos FL=[-0.03956  0.81262 -1.67402], FR=[ 0.04544  0.81681 -1.67197], RL=[-0.11964  0.93739 -1.5598 ], RR=[ 0.11965  0.94345 -1.57245]

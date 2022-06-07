from __future__ import print_function

import os

import rospy as ros
import sys
import time
import threading

import numpy as np
import pinocchio as pin
# utility functions
from base_controllers.utils.utils import Utils
from base_controllers.utils.math_tools import *
from base_controllers.utils.common_functions import getRobotModel

import base_controllers.params as conf

import rosbag
import datetime




""" This class is parent of ExperimentController and of SimulationController. It implements methods that are necessary
 to both the children"""

class Controller(threading.Thread):

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.robot = getRobotModel(robot_name, generate_urdf = True)

        threading.Thread.__init__(self)

        # instantiating objects
        self.joint_names = ""
        self.u = Utils()
        self.mathJet = Math()

        self.q = np.empty(self.robot.na) * np.nan
        self.qd = np.empty(self.robot.na) * np.nan
        self.tau = np.empty(self.robot.na) * np.nan

        self.q_des = conf.robot_params[robot_name]['q_0']
        self.qd_des = np.empty(self.robot.na) * np.nan
        self.tau_des = np.empty(self.robot.na) * np.nan         # tau_des = tau_ffwd + tau_fb
        self.tau_fb = np.empty(self.robot.na) * np.nan
        self.tau_ffwd = np.empty(self.robot.na) * np.nan
        self.gravity_comp = np.empty(self.robot.na) * np.nan

        self.basePoseW = np.empty(6) * np.nan
        self.baseTwistW = np.empty(6) * np.nan

        self.quaternion = np.empty(4) * np.nan

        self.gen_config_neutral = pin.neutral(self.robot.model)
        self.qPin_base_oriented = pin.neutral(self.robot.model)
        self.vPin_base_oriented = np.zeros(self.robot.na)

        self.b_R_w = np.eye(3)

        self.comPosW = np.empty(3) * np.nan
        self.comVelW = np.empty(3) * np.nan

        self.comPosW_des = np.empty(3) * np.nan
        self.comVelW_des = np.empty(3) * np.nan

        self.comPosB = np.empty(3) * np.nan
        self.comVelB = np.empty(3) * np.nan

        self.w_p_b_legOdom = np.empty(3) * np.nan
        self.w_v_b_legOdom = np.empty(3) * np.nan


        self.centroidalInertiaB = np.identity(3)
        self.compositeRobotInertiaB = np.identity(3)

        self.contacts_state = np.array([False] * 4)

        self.contact_matrix = np.hstack( [np.vstack( [np.eye(3),np.empty((3,3))] )]*self.robot.nee )
        self.gravityW = -self.robot.robot_mass*self.robot.model.gravity.vector
        self.gravityB = np.zeros_like(self.gravityW)

        self.grForcesW = np.empty(3*self.robot.nee) * np.nan
        self.grForcesB = np.empty(3 * self.robot.nee) * np.nan
        self.grForcesLocal_gt = np.empty(3 * self.robot.nee) * np.nan
        self.grForcesW_gt  = np.empty(3 * self.robot.nee) * np.nan
        self.wJ = [np.zeros((6, self.robot.nv))] * 4
        self.bJ = [np.eye(3)] * 4

        self.W_contacts = [np.zeros((3))] * 4
        self.B_contacts = [np.zeros((3))] * 4

        self.grForcesLocal_gt_tmp = np.zeros(3)

        # Attribute to be set by ALL children
        self.dt = NotImplemented
        self.ros_pub = NotImplemented
        self.rate = NotImplemented

        now = datetime.datetime.now()
        date_string = now.strftime("%Y%m%d%H%M")

        self.sensors_bag = rosbag.Bag(os.environ['PYSOLO_FROSCIA'] + '/bags/' + date_string + '.bag', 'w')

        # These are for solo!
        self.ee_frames = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
        self.lower_leg_frames = ['lf_lower_leg', 'rf_lower_leg', 'lh_lower_leg', 'rh_lower_leg']
        self.ros_joints_name = ['universe',
                                'floating_base_joint',
                                'lf_haa_joint',
                                'lf_hfe_joint',
                                'lf_kfe_joint',
                                'lh_haa_joint',
                                'lh_hfe_joint',
                                'lh_kfe_joint',
                                'rf_haa_joint',
                                'rf_hfe_joint',
                                'rf_kfe_joint',
                                'rh_haa_joint',
                                'rh_hfe_joint',
                                'rh_kfe_joint']  # ros convention
        self.force_th = conf.robot_params[robot_name]['force_th']

        # send data to param server
        self.verbose = conf.verbose

        self.u.putIntoGlobalParamServer("verbose", self.verbose)



    def _receive_jstate(self, msg):
        self.joint_names = msg.name
        q_ros = np.zeros(self.robot.na)
        qd_ros = np.zeros(self.robot.na)
        tau_ros = np.zeros(self.robot.na)
        for i in range(len(self.joint_names)):
            q_ros[i] = msg.position[i]
            qd_ros[i] = msg.velocity[i]
            tau_ros[i] = msg.effort[i]
        # map from ROS (alphabetical) to our  LF RF LH RH convention
        self.q = self.u.mapFromRos(q_ros)
        self.qd = self.u.mapFromRos(qd_ros)
        self.tau = self.u.mapFromRos(tau_ros)



    def mapBaseToWorld(self, B_var):
        W_var = self.b_R_w.transpose().dot(B_var) + self.u.linPart(self.basePoseW)
        return W_var

    def updateKinematics(self):
        # q is continuously updated
        # to compute in the base frame  you should put neutral base
        q_ros = self.u.mapToRos(self.q)
        qd_ros = self.u.mapToRos(self.qd)
        tau_ros = self.u.mapToRos(self.tau)

        neutral_fb_jointstate = np.hstack((self.gen_config_neutral[0:7], q_ros))
        gen_velocities = np.hstack((self.baseTwistW, qd_ros))

        pin.forwardKinematics(self.robot.model, self.robot.data, neutral_fb_jointstate, gen_velocities)
        pin.computeJointJacobians(self.robot.model, self.robot.data)
        pin.updateFramePlacements(self.robot.model, self.robot.data)

        for leg in range(4):
            self.B_contacts[leg] = self.robot.framePlacement(neutral_fb_jointstate, self.robot.model.getFrameId(
                conf.ee_frames[leg])).translation
            self.W_contacts[leg] = self.mapBaseToWorld(self.B_contacts[leg].transpose())

            leg_joints = range(6 + self.u.mapIndexToRos(leg) * 3, 6 + self.u.mapIndexToRos(leg) * 3 + 3)
            self.bJ[leg] = self.robot.frameJacobian(neutral_fb_jointstate,
                                                   self.robot.model.getFrameId(conf.ee_frames[leg]),
                                                   pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, leg_joints]
            self.wJ[leg] = self.b_R_w.transpose().dot(self.bJ[leg])

    def initVars(self):
        self.comPosB_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.comVelB_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan

        self.comPosW_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.comVelW_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.comPosW_des_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.comVelW_des_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan

        self.comVelW_leg_odom = np.empty((3))*np.nan
        self.comVelW_leg_odom_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan

        self.basePoseW_log = np.empty((6, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.baseTwistW_log = np.empty((6, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan

        self.w_p_b_legOdom_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.w_v_b_legOdom_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.q_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.q_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan

        self.qd_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.qd_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan

        self.tau_fb_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.tau_ffwd_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.tau_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.tau_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan

        self.grForcesW_log = np.empty((3*self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.grForcesB_log = np.empty((3*self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.grForcesW_gt_log = np.empty((3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.grForcesW_des_log = np.empty((3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.W_contacts_log = np.empty((3*self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.B_contacts_log = np.empty((3*self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan

        self.contacts_state_log = np.empty((self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan

        self.constr_viol_log = np.empty((4, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan

        self.time_log = np.empty((1, (conf.robot_params[self.robot_name]['buffer_size']))) *np.nan
        
        self.tau_minus_h_log = np.empty((self.robot.nv, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.tau_minus_h = np.zeros(self.robot.nv)
        
        self.qdd_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.qdd = np.zeros(self.robot.na)

        self.base_acc_W_log = np.empty((6, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.base_acc_W = np.zeros(6)

        self.C_qd_log = np.empty((self.robot.nv, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.C_qd = np.zeros(self.robot.nv)


        self.T_p_com_ref_lc_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.T_p_base_leg_odom_lc_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.T_p_com_ref_lc = np.zeros(3)
        self.T_p_base_leg_odom_lc = np.zeros(3)

        self.T_v_com_ref_lc_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.T_v_base_leg_odom_lc_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.T_v_com_ref_lc = np.zeros(3)
        self.T_v_base_leg_odom_lc = np.zeros(3)

        self.gravity_prop_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) *np.nan
        self.gravity_prop = np.zeros(3)

        self.time = 0.
        self.log_counter = 0

    def logData(self):
        # if log_counter exceed N*buffer_size, reshape all the log variables
        if self.log_counter != 0 and (self.log_counter % conf.robot_params[self.robot_name]['buffer_size']) == 0:
            self.comPosB_log       = self.log_policy(self.comPosB_log)
            self.comVelB_log       = self.log_policy(self.comVelB_log)
            self.comPosW_log       = self.log_policy(self.comPosW_log)
            self.comVelW_log       = self.log_policy(self.comVelW_log)
            self.comPosW_des_log   = self.log_policy(self.comPosW_des_log)
            self.comVelW_des_log   = self.log_policy(self.comVelW_des_log)
            self.basePoseW_log     = self.log_policy(self.basePoseW_log)
            self.baseTwistW_log    = self.log_policy(self.baseTwistW_log)
            self.w_p_b_legOdom_log = self.log_policy(self.w_p_b_legOdom_log)
            self.w_v_b_legOdom_log = self.log_policy(self.w_v_b_legOdom_log)
            self.q_des_log         = self.log_policy(self.q_des_log)
            self.q_log             = self.log_policy(self.q_log)
            self.qd_des_log        = self.log_policy(self.qd_des_log)
            self.qd_log            = self.log_policy(self.qd_log)
            self.tau_fb_log        = self.log_policy(self.tau_fb_log)
            self.tau_ffwd_log      = self.log_policy(self.tau_ffwd_log)
            self.tau_des_log       = self.log_policy(self.tau_des_log)
            self.tau_log           = self.log_policy(self.tau_log)
            self.time_log          = self.log_policy(self.time_log)
            self.constr_viol_log   = self.log_policy(self.constr_viol_log)
            self.grForcesW_log     = self.log_policy(self.grForcesW_log)
            self.grForcesW_des_log = self.log_policy(self.grForcesW_des_log)
            self.grForcesB_log     = self.log_policy(self.grForcesB_log)
            self.grForcesW_gt_log   = self.log_policy(self.grForcesW_gt_log)
            self.W_contacts_log    = self.log_policy(self.W_contacts_log)
            self.B_contacts_log    = self.log_policy(self.B_contacts_log)
            self.contacts_state_log= self.log_policy(self.contacts_state_log)
            self.tau_minus_h_log = self.log_policy(self.tau_minus_h_log)
            self.qdd_log = self.log_policy(self.qdd_log)
            self.base_acc_W_log = self.log_policy(self.base_acc_W_log)
            self.C_qd_log = self.log_policy(self.C_qd_log)
            self.T_p_com_ref_lc_log      = self.log_policy(self.T_p_com_ref_lc_log)
            self.T_p_base_leg_odom_lc_log = self.log_policy(self.T_p_base_leg_odom_lc_log)
            self.T_v_com_ref_lc_log      = self.log_policy(self.T_v_com_ref_lc_log)
            self.T_v_base_leg_odom_lc_log = self.log_policy(self.T_v_base_leg_odom_lc_log)
            self.comVelW_leg_odom_log = self.log_policy(self.comVelW_leg_odom_log)
            self.gravity_prop_log = self.log_policy(self.gravity_prop_log)


        # Fill with new values
        self.comPosB_log[:, self.log_counter] = self.comPosB
        self.comVelB_log[:, self.log_counter] = self.comVelB
        self.comPosW_log[:, self.log_counter] = self.comPosW
        self.comVelW_log[:, self.log_counter] = self.comVelW
        self.comPosW_des_log[:, self.log_counter] = self.comPosW_des
        self.comVelW_des_log[:, self.log_counter] = self.comVelW_des
        self.basePoseW_log[:, self.log_counter] = self.basePoseW
        self.baseTwistW_log[:, self.log_counter] = self.baseTwistW
        self.w_p_b_legOdom_log[:, self.log_counter] = self.w_p_b_legOdom
        self.w_v_b_legOdom_log[:, self.log_counter] = self.w_v_b_legOdom
        self.q_des_log[:, self.log_counter] = self.q_des
        self.q_log[:, self.log_counter] = self.q
        self.qd_des_log[:, self.log_counter] = self.qd_des
        self.qd_log[:, self.log_counter] = self.qd
        self.tau_fb_log[:, self.log_counter] = self.tau_fb
        self.tau_ffwd_log[:, self.log_counter] = self.tau_ffwd
        self.tau_des_log[:, self.log_counter] = self.tau_des
        self.tau_log[:, self.log_counter] = self.tau
        self.grForcesW_log[:, self.log_counter] = self.grForcesW
        self.grForcesW_gt_log[:, self.log_counter] = self.grForcesW_gt
        #self.grForcesW_des_log[:, self.log_counter] = self.grForcesW
        self.grForcesB_log[:, self.log_counter] = self.grForcesB
        self.contacts_state_log[:, self.log_counter] = self.contacts_state
        self.tau_minus_h_log[:, self.log_counter] = self.tau_minus_h
        self.qdd_log[:, self.log_counter] = self.qdd
        self.base_acc_W_log[:, self.log_counter] = self.base_acc_W
        self.C_qd_log[:, self.log_counter] = self.C_qd
        self.T_p_com_ref_lc_log[:, self.log_counter] = self.T_p_com_ref_lc
        self.T_p_base_leg_odom_lc_log[:, self.log_counter] = self.T_p_base_leg_odom_lc

        self.T_v_com_ref_lc_log[:, self.log_counter] = self.T_v_com_ref_lc
        self.T_v_base_leg_odom_lc_log[:, self.log_counter] = self.T_v_base_leg_odom_lc

        self.comVelW_leg_odom_log[:, self.log_counter] = self.comVelW_leg_odom
        self.gravity_prop_log[:, self.log_counter] = self.gravity_prop


        self.time_log[:, self.log_counter] = self.time

        self.log_counter += 1
        self.time += self.dt  # TODO: modify with a more sofisticate update of time


    def log_policy(self, var):
        tmp = np.empty((var.shape[0], var.shape[1]+conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        tmp[:var.shape[0], :var.shape[1]] = var
        return tmp

    # feedforward controllers

    def self_weightCompensation(self):
        self.qPin_base_oriented[3:7] = self.quaternion
        self.qPin_base_oriented[7:] = self.u.mapFromRos(self.q)
        gravity_torques = self.u.mapToRos(self.robot.gravity(self.qPin_base_oriented))[6:]
        return gravity_torques

    def gravityCompensation(self):
        # to simplyfy, all the feet are assumed to be in contact
        fb_config = np.hstack((pin.neutral(self.robot.model)[:7] ,  self.u.mapFromRos(self.q)))

        com_B = self.robot.com(fb_config)

        self.robot.forwardKinematics(fb_config)
        pin.updateFramePlacements(self.robot.model, self.robot.data)

        for i, index in enumerate(self.robot.getEndEffectorsFrameId):
            foot_pos_B = self.robot.data.oMf[index].translation
            S = pin.skew(foot_pos_B-com_B)
            self.contact_matrix[3:, 3 * i:3 * (i + 1)] = S

        C_pinv = np.linalg.pinv(self.contact_matrix)
        self.gravityB[0:3] = pin.Quaternion(self.quaternion).toRotationMatrix().T @ self.gravityW[0:3]
        feet_forces_gravity = C_pinv @ self.gravityB
        J = self.robot.getEEStackJacobians(fb_config, 'linear')[:, 6:]
        contact_torques = -self.u.mapFromRos(J.T @ feet_forces_gravity)
        return contact_torques

    def comFeedforward(self, a_com):
        self.qPin_base_oriented[3:7] = self.quaternion
        self.qPin_base_oriented[7:] = self.u.mapFromRos(self.q)

        pin.jacobianCenterOfMass(self.robot.model, self.robot.data, self.qPin_base_oriented)
        J_com = self.robot.data.Jcom

        wrench = self.robot.robot_mass * a_com
        com_torques = self.u.mapToRos(J_com[:, 6:].T @ wrench)

        return -com_torques


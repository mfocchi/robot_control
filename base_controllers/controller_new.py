from __future__ import print_function

import os

import rospy as ros
import sys
import time
import threading

import numpy as np
import pinocchio as pin
# utility functions
from base_controllers.utils.pidManager import PidManager
from base_controllers.base_controller import BaseController
from base_controllers.utils.math_tools import *

from termcolor import colored

import base_controllers.params as conf

from scipy.io import savemat

import rosbag
import datetime

class Controller(BaseController):
    def __init__(self, robot_name="hyq", launch_file=None):
        super(Controller, self).__init__(robot_name, launch_file)
        self.qj_0 = conf.robot_params[self.robot_name]['q_0']
        self.dt = conf.robot_params[self.robot_name]['dt']


        now = datetime.datetime.now()
        date_string = now.strftime("%Y%m%d%H%M")

        self.sensors_bag = rosbag.Bag(os.environ['PYSOLO_FROSCIA'] + '/bags/' + date_string + '.bag', 'w')

        self.use_ground_truth_contacts = True


    #####################
    # OVERRIDEN METHODS #
    #####################
    # initVars
    # logData
    # startupProcedure

    def initVars(self):
        super().initVars()
        # some extra variables
        self.gen_config_neutral = pin.neutral(self.robot.model)
        self.qPin_base_oriented = pin.neutral(self.robot.model)
        self.vPin_base_oriented = np.zeros(self.robot.na)

        self.tau_ddp = np.zeros(self.robot.na)
        self.tau_fb = np.zeros(self.robot.na)
        self.tau_des = np.zeros(self.robot.na)

        self.basePoseW_des = np.zeros(6) * np.nan
        self.baseTwistW_des = np.zeros(6) * np.nan

        self.comPosW = np.zeros(3) * np.nan
        self.comVelW = np.zeros(3) * np.nan

        self.comPosW_des = np.zeros(3) * np.nan
        self.comVelW_des = np.zeros(3) * np.nan

        self.comPosB = np.zeros(3) * np.nan
        self.comVelB = np.zeros(3) * np.nan

        self.w_p_b_legOdom = np.zeros(3) * np.nan
        self.w_v_b_legOdom = np.zeros(3) * np.nan

        self.contact_state = np.array([False] * 4)

        self.contact_matrix = np.hstack([np.vstack([np.eye(3), np.zeros((3, 3))])] * self.robot.nee)
        self.gravityW = -self.robot.robot_mass * self.robot.model.gravity.vector
        self.gravityB = np.zeros_like(self.gravityW)

        self.g_mag = np.linalg.norm(self.robot.model.gravity.vector)


        self.grForcesB = np.empty(3 * self.robot.nee) * np.nan
        self.grForcesB_ffwd = np.empty(3 * self.robot.nee) * np.nan
        self.grForcesB_ddp = np.empty(3 * self.robot.nee) * np.nan

        self.bJ = [np.eye(3)] * 4

        self.grForcesLocal_gt_tmp = np.zeros(3)

        try:
            self.force_th = conf.robot_params[self.robot_name]['force_th']
        except KeyError:
            self.force_th = 0.


        self.comPosB_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.comVelB_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.comPosW_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.comVelW_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.comPosW_des_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.comVelW_des_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.comVelW_leg_odom = np.empty((3)) * np.nan
        self.comVelW_leg_odom_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan


        self.basePoseW_des_log = np.empty((6, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.baseTwistW_des_log = np.empty((6, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.w_p_b_legOdom_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.w_v_b_legOdom_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.tau_fb_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.tau_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan


        self.grForcesB_log = np.empty((3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.grForcesW_gt_log = np.empty(
            (3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.grForcesW_des_log = np.empty(
            (3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.W_contacts_log = np.empty((3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.B_contacts_log = np.empty((3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.contact_state_log = np.empty((self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.tau_minus_h_log = np.empty((self.robot.nv, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.tau_minus_h = np.zeros(self.robot.nv)

        self.qdd_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.qdd = np.zeros(self.robot.na)

        self.base_acc_W_log = np.empty((6, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.base_acc_W = np.zeros(6)

        self.C_qd_log = np.empty((self.robot.nv, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.C_qd = np.zeros(self.robot.nv)

        self.T_p_com_ref_lc_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.T_p_base_leg_odom_lc_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.T_p_com_ref_lc = np.zeros(3)
        self.T_p_base_leg_odom_lc = np.zeros(3)

        self.T_v_com_ref_lc_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.T_v_base_leg_odom_lc_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.T_v_com_ref_lc = np.zeros(3)
        self.T_v_base_leg_odom_lc = np.zeros(3)

        self.time_log =  np.empty((1, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan


    def logData(self):
        self.log_counter += 1

        # if log_counter exceed N*buffer_size, reshape all the log variables
        if self.log_counter != 0 and (self.log_counter % conf.robot_params[self.robot_name]['buffer_size']) == 0:
            self.comPosB_log = self.log_policy(self.comPosB_log)
            self.comVelB_log = self.log_policy(self.comVelB_log)
            self.comPosW_log = self.log_policy(self.comPosW_log)
            self.comVelW_log = self.log_policy(self.comVelW_log)
            self.comPosW_des_log = self.log_policy(self.comPosW_des_log)
            self.comVelW_des_log = self.log_policy(self.comVelW_des_log)
            self.basePoseW_log = self.log_policy(self.basePoseW_log)
            self.baseTwistW_log = self.log_policy(self.baseTwistW_log)
            self.basePoseW_des_log = self.log_policy(self.basePoseW_des_log)
            self.baseTwistW_des_log = self.log_policy(self.baseTwistW_des_log)
            self.w_p_b_legOdom_log = self.log_policy(self.w_p_b_legOdom_log)
            self.w_v_b_legOdom_log = self.log_policy(self.w_v_b_legOdom_log)
            self.q_des_log = self.log_policy(self.q_des_log)
            self.q_log = self.log_policy(self.q_log)
            self.qd_des_log = self.log_policy(self.qd_des_log)
            self.qd_log = self.log_policy(self.qd_log)
            self.tau_fb_log = self.log_policy(self.tau_fb_log)
            self.tau_ffwd_log = self.log_policy(self.tau_ffwd_log)
            self.tau_des_log = self.log_policy(self.tau_des_log)
            self.tau_log = self.log_policy(self.tau_log)
            self.time_log = self.log_policy(self.time_log)
            self.constr_viol_log = self.log_policy(self.constr_viol_log)
            self.grForcesW_log = self.log_policy(self.grForcesW_log)
            self.grForcesW_des_log = self.log_policy(self.grForcesW_des_log)
            self.grForcesB_log = self.log_policy(self.grForcesB_log)
            self.grForcesW_gt_log = self.log_policy(self.grForcesW_gt_log)
            self.W_contacts_log = self.log_policy(self.W_contacts_log)
            self.B_contacts_log = self.log_policy(self.B_contacts_log)
            self.contact_state_log = self.log_policy(self.contact_state_log)
            self.tau_minus_h_log = self.log_policy(self.tau_minus_h_log)
            self.qdd_log = self.log_policy(self.qdd_log)
            self.base_acc_W_log = self.log_policy(self.base_acc_W_log)
            self.C_qd_log = self.log_policy(self.C_qd_log)
            self.T_p_com_ref_lc_log = self.log_policy(self.T_p_com_ref_lc_log)
            self.T_p_base_leg_odom_lc_log = self.log_policy(self.T_p_base_leg_odom_lc_log)
            self.T_v_com_ref_lc_log = self.log_policy(self.T_v_com_ref_lc_log)
            self.T_v_base_leg_odom_lc_log = self.log_policy(self.T_v_base_leg_odom_lc_log)
            self.comVelW_leg_odom_log = self.log_policy(self.comVelW_leg_odom_log)

        # Fill with new values
        self.comPosB_log[:, self.log_counter] = self.comPosB
        self.comVelB_log[:, self.log_counter] = self.comVelB
        self.comPosW_log[:, self.log_counter] = self.comPosW
        self.comVelW_log[:, self.log_counter] = self.comVelW
        self.comPosW_des_log[:, self.log_counter] = self.comPosW_des
        self.comVelW_des_log[:, self.log_counter] = self.comVelW_des
        self.basePoseW_log[:, self.log_counter] = self.basePoseW
        self.baseTwistW_log[:, self.log_counter] = self.baseTwistW
        self.basePoseW_des_log[:, self.log_counter] = self.basePoseW_des
        self.baseTwistW_des_log[:, self.log_counter] = self.baseTwistW_des
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
        # self.grForcesW_des_log[:, self.log_counter] = self.grForcesW
        self.grForcesB_log[:, self.log_counter] = self.grForcesB
        self.contact_state_log[:, self.log_counter] = self.contact_state
        self.tau_minus_h_log[:, self.log_counter] = self.tau_minus_h
        self.qdd_log[:, self.log_counter] = self.qdd
        self.base_acc_W_log[:, self.log_counter] = self.base_acc_W
        self.C_qd_log[:, self.log_counter] = self.C_qd
        self.T_p_com_ref_lc_log[:, self.log_counter] = self.T_p_com_ref_lc
        self.T_p_base_leg_odom_lc_log[:, self.log_counter] = self.T_p_base_leg_odom_lc

        self.T_v_com_ref_lc_log[:, self.log_counter] = self.T_v_com_ref_lc
        self.T_v_base_leg_odom_lc_log[:, self.log_counter] = self.T_v_base_leg_odom_lc

        self.comVelW_leg_odom_log[:, self.log_counter] = self.comVelW_leg_odom
        self.time_log[:, self.log_counter] = self.time


    def startController(self, xacro_path=None, world_name=None, use_real_robot=False):
        self.start()                               # as a thread
        if not use_real_robot:
            self.startSimulator(world_name)        # run gazebo
        self.loadModelAndPublishers(xacro_path)    # load robot and all the publishers
        #self.resetGravity(True)
        self.initVars()                            # overloaded method
        self.rate = ros.Rate(1 / self.dt)
        self.startupProcedure()                    # overloaded method
        print(colored("Started controller", "blue"))


    def log_policy(self, var):
        tmp = np.empty((var.shape[0], var.shape[1] + conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        tmp[:var.shape[0], :var.shape[1]] = var
        return tmp

    # feedforward controllers

    def self_weightCompensation(self):
        # require the call to updateKinematics
        gravity_torques = self.h_joints
        return gravity_torques

    def gravityCompensation(self):
        # require the call to updateKinematics
        # to simplyfy, all the feet are assumed to be in contact
        contact_torques = np.zeros(self.robot.na)
        for leg in range(4):
            S = pin.skew(self.B_contacts[leg] - self.comB )
            self.contact_matrix[3:, 3 * leg:3 * (leg + 1)] = S
        C_pinv = np.linalg.pinv(self.contact_matrix)
        self.gravityB[0:3] = self.b_R_w @ self.gravityW[0:3]
        feet_forces_gravity = C_pinv @ self.gravityB
        for leg in range(4):
            tau_leg  = -self.J[leg].T  @ self.u.getLegJointState(leg, feet_forces_gravity)
            self.u.setLegJointState(leg, tau_leg, contact_torques)
        return contact_torques

    # TODO: To be tested
    # def gravityCompensation(self, legsInContact):
    #     legsInContact.sort()
    #     nc = len(legsInContact)
    #     fb_config = np.hstack((0.,0.,0.,0., 0., 0., 1., self.u.mapFromRos(self.q)))
    #
    #     com_B = self.robot.com(fb_config)
    #
    #     self.robot.forwardKinematics(fb_config)
    #     pin.updateFramePlacements(self.robot.model, self.robot.data)
    #     contact_matrix = np.zeros([6, nc])
    #
    #     for i in range(0, nc):
    #         index = self.robot.getEndEffectorsFrameId[i]
    #         foot_pos_B = self.robot.data.oMf[index].translation
    #         S = pin.skew(foot_pos_B - com_B)
    #         contact_matrix[:3, 3 * i:3 * (i + 1)] = np.eye(3)
    #         contact_matrix[3:, 3 * i:3 * (i + 1)] = S
    #     C_pinv = np.linalg.pinv(contact_matrix)
    #     self.gravityB[0:3] = pin.Quaternion(self.quaternion).toRotationMatrix().T @ self.gravityW[0:3]
    #     # these are the forces applied to the feet in contact
    #     feet_forces_gravity_tmp = C_pinv @ self.gravityB
    #     # let add zeros for the forces on the feet not in contact
    #     feet_forces_gravity= np.zeros(3*self.robot.nee)
    #     for leg in range(0, self.robot.nee):
    #         if leg in legsInContact:
    #             f = feet_forces_gravity_tmp[3*legsInContact.index(leg):3*(legsInContact.index(leg)+1)]
    #             feet_forces_gravity = self.u.getLegJointState(leg, f)
    #
    #     J = self.robot.getEEStackJacobians(fb_config, 'linear')[:, 6:]
    #     contact_torques = -self.u.mapToRos(J.T @ feet_forces_gravity)
    #     return contact_torques

    def comFeedforward(self, a_com):
        self.qPin_base_oriented[3:7] = self.quaternion
        self.qPin_base_oriented[7:] = self.u.mapFromRos(self.q)

        pin.jacobianCenterOfMass(self.robot.model, self.robot.data, self.qPin_base_oriented)
        J_com = self.robot.data.Jcom

        wrench = self.robot.robot_mass * a_com
        com_torques = self.u.mapToRos(J_com[:, 6:].T @ wrench)

        return -com_torques


    def send_command(self, q_des=None, qd_des=None, tau_ffwd=None, tau_fb = None):
        # q_des, qd_des, and tau_ffwd have dimension 12
        # and are ordered as on the robot

        if (q_des is None) and (qd_des is None) and (tau_ffwd is None):
            raise RuntimeError('Cannot have both states (q_des and qd_des) and controls (tau_ffwd) as None')

        if q_des is not None:
            self.q_des = q_des
        else:
            self.q_des = np.zeros(self.robot.na)

        if qd_des is not None:
            self.qd_des = qd_des
        else:
            self.qd_des = np.zeros(self.robot.na)

        if tau_ffwd is not None:
            self.tau_ffwd = tau_ffwd
        else:
            self.tau_ffwd = np.zeros(self.robot.na)

        if tau_fb is not None:
            self.tau_fb = tau_fb
        else:
            self.tau_fb = np.zeros(self.robot.na)

        self.tau_des = self.tau_ffwd + self.tau_fb

        self.send_des_jstate(self.q_des, self.qd_des, self.tau_des)

        # if (self.APPLY_EXTERNAL_WRENCH and self.time > self.TIME_EXTERNAL_WRENCH):
        #     print("START APPLYING EXTERNAL WRENCH")
        #     self.applyForce(0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.05)
        #     self.APPLY_EXTERNAL_WRENCH = False

        # log variables
        self.rate.sleep()
        self.logData()
        self.time = np.round(self.time + np.array([conf.robot_params[self.robot_name]['dt']]), 3)



    # def estimateContacts_KKT(self): # TODO
    #     q_ros = self.u.mapToRos(self.q)
    #     v_ros = self.u.mapToRos(self.qd)
    #     tau_ros = self.u.mapToRos(self.tau)
    #
    #     B_R_W = pin.Quaternion(self.quaternion).toRotationMatrix().T
    #
    #     full_conf = np.hstack((self.basePoseW[0:3], self.quaternion, q_ros))
    #     full_vel  = np.hstack((self.baseTwistW[0:6], v_ros))
    #
    #     full_tau = np.hstack(([0.]*6, tau_ros))
    #     full_h = self.robot.nle(full_conf, full_vel)
    #
    #     C_qd = full_h - self.robot.gravity(full_conf)
    #
    #     self.tau_minus_h = full_tau - full_h
    #     self.C_qd = C_qd
    #     bias = np.hstack((self.tau_minus_h, [0., 0., 0.] * 4))
    #
    #     dJdq = np.zeros(12)
    #     for leg in range(0, 4):
    #         dJdq[3*leg:3*(leg+1)] = self.robot.dJdq(full_conf, full_vel, self.robot.model.getFrameId(self.ee_frame_names[leg]), 'linear')
    #     bias[self.robot.nv:] = self.u.mapToRos(dJdq)
    #
    #     KKT_inv = self.robot.KKTMatrixAtEndEffectorsInv(full_conf, 'linear')
    #     tmp = KKT_inv @ bias
    #     full_a = tmp[0:self.robot.nv]
    #     full_grfW = -tmp[self.robot.nv:]
    #
    #     self.grForcesW = self.u.mapToRos(full_grfW)
    #     self.base_acc_W = full_a[:6]
    #     self.qdd = self.u.mapToRos(full_a[6:])
    #
    #     for leg in range(4):
    #
    #         self.grForcesB[3*leg:3*(leg+1)] = B_R_W @ self.grForcesW[3*leg:3*(leg+1)]
    #
    #         self.contact_state[leg] = self.grForcesW[3*(leg+1)-1] > self.force_th




    def visualizeContacts(self):
        for legid in self.u.leg_map.keys():
            leg = self.u.leg_map[legid]
            # self.ros_pub.add_arrow(self.W_contacts[leg],
            #                        self.u.getLegJointState(leg, self.grForcesB/ (6*self.robot.robot_mass )),
            #                        "green", alpha = 0.5)
            self.ros_pub.add_arrow(self.W_contacts[leg],
                                   self.u.getLegJointState(leg, self.grForcesB_ffwd / (6 * self.robot.robot_mass)),
                                   "blue", alpha=0.5)
            self.ros_pub.add_arrow(self.W_contacts[leg],
                                   self.u.getLegJointState(leg, self.grForcesB_ddp / (6 * self.robot.robot_mass)),
                                   "black", alpha=0.5)
            # if (self.use_ground_truth_contacts):
            #     self.ros_pub.add_arrow(self.W_contacts[leg],
            #                         self.u.getLegJointState(leg, self.grForcesW_gt / (6*self.robot.robot_mass)),
            #                         "red", alpha = 0.5)
            if self.contacts_state[leg]:
                self.ros_pub.add_marker(self.W_contacts[leg], radius=0.1)
            else:
                self.ros_pub.add_marker(self.W_contacts[leg], radius=0.001)
        self.ros_pub.publishVisual()


    def startupProcedure(self):
        if self.robot_name == 'hyq':
            super(Controller, self).startupProcedure()
            return

        self.pid = PidManager(self.joint_names)
        self.pid.setPDjoints(conf.robot_params[self.robot_name]['kp'],
                             conf.robot_params[self.robot_name]['kd'],
                             np.zeros(self.robot.na))

        start_t = ros.get_time()
        q_start = self.q.copy()
        q_end = conf.robot_params[self.robot_name]['q_0']
        self.qd_des = np.zeros(self.robot.na)
        alpha = 0

        # run startup procedure if cumulative error is > 0.2 or if joint velocity is > 0.01
        while np.linalg.norm(self.q-q_end)>0.2 or np.linalg.norm(self.qd-self.qd_des)>0.01:
            self.q_des = alpha * q_end + (1-alpha) * q_start
            self.tau_ffwd = self.gravityCompensation()+self.self_weightCompensation()
            self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
            if alpha < 1:
                alpha = np.round(alpha+0.01, 2)
            ros.sleep(0.01)

        print("finished startup")



    def save_reference(self, filename):
        DATA = {}
        DATA['last_sample'] = self.log_counter
        DATA['q_des'] = self.q_des_log[:, :self.log_counter]
        DATA['qd_des'] = self.qd_des_log[:, :self.log_counter]
        DATA['tau_ffwd'] = self.tau_ffwd_log[:, :self.log_counter]

        savemat(filename, DATA)

        print('Reference saved in', filename)




if __name__ == '__main__':
    p = Controller('solo')

    p.startupProcedure()
    from pysolo.controllers.simple_controllers import PushUpReference
    reference_push_up = PushUpReference(q0=p.q, amplitude=np.pi / 18, frequency=1)
    start_time = p.time
    try:

        while not ros.is_shutdown():
            task_time = p.time - start_time
            p.estimateContacts()
            p.visualizeContacts()
            q_des, qd_des = reference_push_up.compute(task_time)
            tau_ffwd = p.gravityCompensation()+p.self_weightCompensation()
            tau_fb = p.computePID(q_des, qd_des)
            p.send_command(q_des, qd_des, tau_ffwd, tau_fb)



    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        # stats = pstats.Stats(profiler).sort_stats('cumtime')
        # stats.print_stats()


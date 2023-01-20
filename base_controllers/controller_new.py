from __future__ import print_function

import os

import rospy as ros
import sys
import time
import threading

import numpy as np
import pinocchio as pin
# utility functions
from  scipy.linalg import block_diag
from base_controllers.utils.pidManager import PidManager
from base_controllers.base_controller import BaseController
from base_controllers.utils.math_tools import *
from base_controllers.utils.optimTools import quadprog_solve_qp

from base_controllers.components.inverse_kinematics.inv_kinematics_quadruped import InverseKinematics
from base_controllers.components.leg_odometry.leg_odometry import LegOdometry
from termcolor import colored

import base_controllers.params as conf

from scipy.io import savemat

#gazebo messages
from gazebo_ros import gazebo_interface

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from base_controllers.components.imu_utils import IMU_utils


import datetime

class Controller(BaseController):
    def __init__(self, robot_name="hyq", launch_file=None):
        super(Controller, self).__init__(robot_name, launch_file)
        self.qj_0 = conf.robot_params[self.robot_name]['q_0']
        self.dt = conf.robot_params[self.robot_name]['dt']

        self.ee_frames = conf.robot_params[self.robot_name]['ee_frames']

        self.imu_utils = IMU_utils(dt=conf.robot_params[self.robot_name]['dt'])

    #####################
    # OVERRIDEN METHODS #
    #####################
    # initVars
    # logData
    # startupProcedure

    def initSubscribers(self):
        super().initSubscribers()
        if self.real_robot:
            self.sub_imu_lin_acc = ros.Subscriber("/" + self.robot_name + "/trunk_imu", Vector3,
                                                 callback=self._receive_imu_acc_real, queue_size=1, tcp_nodelay=True)
            self.sub_imu_euler = ros.Subscriber("/" + self.robot_name + "/euler_imu", Vector3,
                                                 callback=self._receive_euler, queue_size=1, tcp_nodelay=True)
        else:
            self.sub_imu_lin_acc = ros.Subscriber("/" + self.robot_name + "/trunk_imu", Imu,
                                                  callback=self._receive_imu_acc, queue_size=1, tcp_nodelay=True)

    def _receive_imu_acc_real(self, msg):
        self.B_imu_lin_acc[0] = msg.x
        self.B_imu_lin_acc[1] = msg.y
        self.B_imu_lin_acc[2] = msg.z

        self.W_base_lin_acc = self.b_R_w.T @ (self.B_imu_lin_acc - self.imu_utils.IMU_accelerometer_bias) - self.imu_utils.g0

    def _receive_imu_acc(self, msg):
        self.B_imu_lin_acc[0] = msg.linear_acceleration.x
        self.B_imu_lin_acc[1] = msg.linear_acceleration.y
        self.B_imu_lin_acc[2] = msg.linear_acceleration.z

        self.W_base_lin_acc = self.b_R_w.T @ (self.B_imu_lin_acc - self.imu_utils.IMU_accelerometer_bias) - self.imu_utils.g0

    def _receive_euler(self, msg):
        self.euler[0] = msg.x
        self.euler[1] = msg.y
        self.euler[2] = msg.z



    def _receive_pose(self, msg):
        self.quaternion[0] = msg.pose.pose.orientation.x
        self.quaternion[1] = msg.pose.pose.orientation.y
        self.quaternion[2] = msg.pose.pose.orientation.z
        self.quaternion[3] = msg.pose.pose.orientation.w

        if self.real_robot:
            self.basePoseW[self.u.sp_crd["LX"]] = self.w_p_b_legOdom[0]
            self.basePoseW[self.u.sp_crd["LY"]] = self.w_p_b_legOdom[1]
            self.basePoseW[self.u.sp_crd["LZ"]] = self.w_p_b_legOdom[2]
        else:
            self.basePoseW[self.u.sp_crd["LX"]] = msg.pose.pose.position.x
            self.basePoseW[self.u.sp_crd["LY"]] = msg.pose.pose.position.y
            self.basePoseW[self.u.sp_crd["LZ"]] = msg.pose.pose.position.z

            self.euler = np.array(euler_from_quaternion(self.quaternion))

        self.basePoseW[self.u.sp_crd["AX"]] = self.euler[0]
        self.basePoseW[self.u.sp_crd["AY"]] = self.euler[1]
        self.basePoseW[self.u.sp_crd["AZ"]] = self.euler[2]

        if self.real_robot:
            if all(self.contact_state) == True:
                self.baseTwistW[self.u.sp_crd["LX"]] = self.w_v_b_legOdom[0]
                self.baseTwistW[self.u.sp_crd["LY"]] = self.w_v_b_legOdom[1]
                self.baseTwistW[self.u.sp_crd["LZ"]] = self.w_v_b_legOdom[2]
            else:
                self.baseTwistW[self.u.sp_crd["LX"]] = self.imu_utils.W_lin_vel[0]
                self.baseTwistW[self.u.sp_crd["LY"]] = self.imu_utils.W_lin_vel[1]
                self.baseTwistW[self.u.sp_crd["LZ"]] = self.imu_utils.W_lin_vel[2]
        else:
            self.baseTwistW[self.u.sp_crd["LX"]] = msg.twist.twist.linear.x
            self.baseTwistW[self.u.sp_crd["LY"]] = msg.twist.twist.linear.y
            self.baseTwistW[self.u.sp_crd["LZ"]] = msg.twist.twist.linear.z
        self.baseTwistW[self.u.sp_crd["AX"]] = msg.twist.twist.angular.x
        self.baseTwistW[self.u.sp_crd["AY"]] = msg.twist.twist.angular.y
        self.baseTwistW[self.u.sp_crd["AZ"]] = msg.twist.twist.angular.z

        # compute orientation matrix
        self.b_R_w = self.math_utils.rpyToRot(self.euler)
        self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
                                       self.quaternion,
                                       ros.Time.now(), '/base_link', '/world')



    def initVars(self):
        super().initVars()
        self.q_des = self.u.mapToRos(conf.robot_params[self.robot_name]['q_fold'])
        self.IK = InverseKinematics(self.robot)
        self.leg_odom = LegOdometry(self.robot)
        self.legConfig = {}
        if 'solo' in self.robot_name:  # either solo or solo_fw
            self.legConfig['lf'] = ['HipDown', 'KneeInward']
            self.legConfig['lh'] = ['HipDown', 'KneeInward']
            self.legConfig['rf'] = ['HipDown', 'KneeInward']
            self.legConfig['rh'] = ['HipDown', 'KneeInward']

        elif self.robot_name == 'aliengo' or self.robot_name == 'go1':
            self.legConfig['lf'] = ['HipDown', 'KneeInward']
            self.legConfig['lh'] = ['HipDown', 'KneeOutward']
            self.legConfig['rf'] = ['HipDown', 'KneeInward']
            self.legConfig['rh'] = ['HipDown', 'KneeOutward']

        else:
            assert False, 'Robot name is not valid'

        self.euler = np.zeros(3)
        # some extra variables

        self.tau_fb = np.zeros(self.robot.na)
        self.tau_des = np.zeros(self.robot.na)

        self.basePoseW_des = np.zeros(6) * np.nan
        self.baseTwistW_des = np.zeros(6) * np.nan


        self.comPoseW_des = np.zeros(6) * np.nan
        self.comTwistW_des = np.zeros(6) * np.nan

        self.comPosB = np.zeros(3) * np.nan
        self.comVelB = np.zeros(3) * np.nan

        self.w_p_b_legOdom = np.zeros(3) #* np.nan
        self.w_v_b_legOdom = np.zeros(3) #* np.nan

        self.contact_state = np.array([False] * 4)

        self.g_mag = np.linalg.norm(self.robot.model.gravity.vector)

        self.grForcesW_des = np.empty(3 * self.robot.nee) * np.nan
        self.grForcesB = np.empty(3 * self.robot.nee) * np.nan
        self.grForcesB_ffwd = np.empty(3 * self.robot.nee) * np.nan

        # virtual impedance wrench control
        self.Kp_lin = np.diag(conf.robot_params[self.robot_name].get('Kp_lin', np.zeros(3)))
        self.Kd_lin = np.diag(conf.robot_params[self.robot_name].get('Kd_lin', np.zeros(3)))

        self.Kp_ang = np.diag(conf.robot_params[self.robot_name].get('Kp_ang', np.zeros(3)))
        self.Kd_ang = np.diag(conf.robot_params[self.robot_name].get('Kd_ang', np.zeros(3)))

        self.wrench_fbW  = np.zeros(6)
        self.wrench_ffW  = np.zeros(6)
        self.wrench_gW   = np.zeros(6)
        self.wrench_gW[self.u.sp_crd["LZ"]] = self.robot.robotMass * self.g_mag
        self.wrench_desW = np.zeros(6)

        self.wrench_fbW_log = np.full( (6, conf.robot_params[self.robot_name]['buffer_size'] ), np.nan)
        self.wrench_ffW_log = np.full( (6, conf.robot_params[self.robot_name]['buffer_size'] ), np.nan)
        self.wrench_gW_log = np.full( (6, conf.robot_params[self.robot_name]['buffer_size'] ), np.nan)
        self.wrench_desW_log = np.full( (6, conf.robot_params[self.robot_name]['buffer_size'] ), np.nan)

        self.NEMatrix = np.zeros([6, 3*self.robot.nee]) # Newton-Euler matrix

        try:
            self.force_th = conf.robot_params[self.robot_name]['force_th']
        except KeyError:
            self.force_th = 0.

        # imu
        self.B_imu_lin_acc = np.full(3, np.nan)
        self.W_base_lin_acc = np.full(3, np.nan)


        self.comPosB_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.comVelB_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)

        self.comPoseW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.comTwistW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.comPoseW_des_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.comTwistW_des_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)

        self.comVelW_leg_odom = np.full((3), np.nan)
        self.comVelW_leg_odom_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)


        self.basePoseW_des_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.baseTwistW_des_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.w_p_b_legOdom_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.w_v_b_legOdom_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)

        self.tau_fb_log = np.full((self.robot.na, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.tau_des_log = np.full((self.robot.na, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)


        self.grForcesB_log = np.full((3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.grForcesW_gt_log = np.full((3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)

        self.grForcesW_des_log = np.full((3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)

        self.W_contacts_log = np.full((3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.W_contacts_des_log = np.full((3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)

        self.B_contacts_log = np.full((3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.B_contacts_des_log = np.full((3 * self.robot.nee, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)

        self.contact_state_log = np.empty((self.robot.nee, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan

        self.W_base_lin_acc_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.W_base_lin_acc = np.zeros(3)
        
        self.W_lin_vel_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)

        self.zmp = np.zeros(3)
        self.zmp_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)


    def logData(self):
        self.log_counter += 1
        self.log_counter %= conf.robot_params[self.robot_name]['buffer_size']

        # full with new values
        self.comPosB_log[:, self.log_counter] = self.comB
        self.comVelB_log[:, self.log_counter] = self.comVelB
        self.comPoseW_log[:, self.log_counter] = self.comPoseW
        self.comTwistW_log[:, self.log_counter] = self.comTwistW
        self.comPoseW_des_log[:, self.log_counter] = self.comPoseW_des
        self.comTwistW_des_log[:, self.log_counter] = self.comTwistW_des
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

        self.tau_des = self.tau_ffwd + self.tau_fb
        self.tau_des_log[:, self.log_counter] = self.tau_des
        self.tau_log[:, self.log_counter] = self.tau
        self.grForcesW_log[:, self.log_counter] = self.grForcesW
        self.grForcesW_des_log[:, self.log_counter] = self.grForcesW_des
        self.grForcesW_gt_log[:, self.log_counter] = self.grForcesW_gt
        self.grForcesB_log[:, self.log_counter] = self.grForcesB
        self.contact_state_log[:, self.log_counter] = self.contact_state
        
        self.W_base_lin_acc_log[:, self.log_counter] = self.W_base_lin_acc
        

        self.comVelW_leg_odom_log[:, self.log_counter] = self.comVelW_leg_odom

        self.B_contacts_log[:, self.log_counter] = np.array(self.B_contacts).flatten()
        self.B_contacts_des_log[:, self.log_counter] = np.array(self.B_contacts_des).flatten()

        self.W_contacts_log[:, self.log_counter] = np.array(self.W_contacts).flatten()
        self.W_contacts_des_log[:, self.log_counter] = np.array(self.W_contacts_des).flatten()

        self.W_lin_vel_log[:, self.log_counter] = self.imu_utils.W_lin_vel
        self.zmp_log[:, self.log_counter] = self.zmp

        self.wrench_fbW_log[:, self.log_counter] = self.wrench_fbW
        self.wrench_ffW_log[:, self.log_counter] = self.wrench_ffW
        self.wrench_gW_log[:, self.log_counter] = self.wrench_gW
        self.wrench_desW_log[:, self.log_counter] = self.wrench_desW


        self.time_log[self.log_counter] = self.time


    def startController(self, world_name=None, xacro_path=None, use_ground_truth_contacts=True, additional_args=None):

        if self.real_robot == False:
            self.use_ground_truth_contacts = use_ground_truth_contacts
        else:
            self.use_ground_truth_contacts = False

        self.start()                               # as a thread

        self.go0_conf = 'home'
        if additional_args is not None:
            for arg in additional_args:
                if 'go0_conf:=' in arg:
                    self.go0_conf = arg.replace('go0_conf:=', '')



        self.startSimulator(world_name, additional_args)            # run gazebo
        if world_name is None:
            self.world_name_str = ''
        else:
            self.world_name_str = world_name
        if 'camera' in self.world_name_str:
            # check if some old jpg are still in /tmp
            print('number of files /tmp/camera_save/default_camera_link_my_camera*: ')
            n_files = os.system('ls /tmp/camera_save/ | grep "default_camera_link_my_camera*" | wc -l')
            if n_files != 0:
                remove_jpg_cmd = "rm /tmp/camera_save/default_camera_link_my_camera*.jpg"
                os.system(remove_jpg_cmd)
                print(colored('Jpg files removed', 'blue'), flush=True)

        self.loadModelAndPublishers(xacro_path)    # load robot and all the publishers
        #self.resetGravity(True)
        self.initVars()                            # overloaded method
        self.initSubscribers()
        self.rate = ros.Rate(1 / self.dt)
        print(colored("Started controller", "blue"))




    def reset(self, basePoseW=None, baseTwistW=None, resetPid=False):
        if basePoseW is None:
            basePoseW = np.hstack([self.base_offset, np.zeros(3)])
        self.freezeBase(flag=True, basePoseW=basePoseW)
        q_des = conf.robot_params[self.robot_name]['q_0']
        qd_des = np.zeros(self.robot.na)
        tau_ffwd = np.zeros(self.robot.na)
        if resetPid:
            self.pid.setPDjoints(conf.robot_params[self.robot_name]['kp'],
                                 conf.robot_params[self.robot_name]['kd'],
                                 np.zeros(self.robot.na))
        gazebo_interface.set_model_configuration_client(self.robot_name, '', self.joint_names, self.qj_0, '/gazebo')
        for k in range(100): # for 30 iteration keep the position, needed for restore joints
            self.send_command(q_des, qd_des, tau_ffwd)
        if baseTwistW is None:
            baseTwistW = np.zeros(6)

        self.freezeBase(flag=True, basePoseW=basePoseW, baseTwistW=baseTwistW)

        self.initVars() # reset logged values


        self.imu_utils.W_lin_vel = self.u.linPart(self.baseTwistW)



    def self_weightCompensation(self):
        # it seems that self weight compensation degrates the control performances: do not use it!
        # require the call to updateKinematics
        gravity_torques = self.u.mapFromRos(self.g_joints)
        # gravity_torques = np.zeros(12)
        return gravity_torques

    def gravityCompensation(self):
        # require the call to updateKinematics
        return self.WBC(des_pose = None, des_twist = None, des_acc = None, comControlled = True, type = 'projection')


    def virtualImpedanceWrench(self, des_pose, des_twist, des_acc = None, comControlled = True):
        if not(des_pose is None or des_twist is None):
            if comControlled:
                act_pose = self.comPoseW
                act_twist = self.comTwistW
            else:
                act_pose = self.basePoseW
                act_twist = self.baseTwistW
            # FEEDBACK WRENCH
            # ---> linear part
            self.wrench_fbW[self.u.sp_crd["LX"]:self.u.sp_crd["LX"] + 3] = self.Kp_lin @ (self.u.linPart(des_pose)  - self.u.linPart(act_pose)) + \
                                                                       self.Kd_lin @ (self.u.linPart(des_twist) - self.u.linPart(act_twist))

            # ---> angular part
            # actual orientation: self.b_R_w
            # Desired Orientation
            b_Rdes_w = self.math_utils.rpyToRot(self.u.angPart(des_pose))

            # compute orientation error
            b_Re_w = b_Rdes_w @ self.b_R_w.T
            # express orientation error in angle-axis form
            b_err = rotMatToRotVec(b_Re_w)

            # the orientation error is expressed in the base_frame so it should be rotated to have the wrench in the
            # world frame
            w_err = self.b_R_w.T @ b_err
            # map des euler rates into des omega
            Jomega = self.math_utils.Tomega(self.u.angPart(self.comPoseW))

            # Note we defined the angular part of the des twist as euler rates not as omega so we need to map them to an
            # Euclidean space with Jomega
            self.wrench_fbW[self.u.sp_crd["AX"]:self.u.sp_crd["AX"] + 3] = self.Kp_ang @ w_err + \
                                                                       self.Kd_ang @ ( Jomega @ (self.u.angPart(des_twist) - self.u.angPart(act_twist)))

            # FEED-FORWARD WRENCH
            if not (des_acc is None):
                # ---> linear part
                self.wrench_ffW[self.u.sp_crd["LX"]:self.u.sp_crd["LX"] + 3] = self.robot.robotMass * self.u.linPart(
                    des_acc)
                # ---> angular part
                # compute inertia in the world frame:  w_I = R' * B_I * R
                w_I = self.b_R_w.T @ self.centroidalInertiaB @ self.b_R_w
                # compute w_des_omega_dot =  Jomega*des euler_rates_dot + Jomega_dot*des euler_rates (Jomega already computed, see above)
                Jomega_dot = self.math_utils.Tomega_dot(self.u.angPart(self.comPoseW), self.u.angPart(self.comTwistW))
                w_des_omega_dot = Jomega @ self.u.angPart(des_acc) + Jomega_dot @ self.u.angPart(des_twist)
                self.wrench_ffW[self.u.sp_crd["AX"]:self.u.sp_crd["AX"] + 3] = w_I @ w_des_omega_dot

        # GRAVITY WRENCH
        # ---> linear part
        # self.wrench_gW[self.u.sp_crd["LZ"]+1] = self.robot.robotMass * self.g_mag (to avoid unuseful repetition, this is in the definiton of wrench_gW)
        # ---> angular part
        if not comControlled:  # act_state  = base position in this case
            W_base_to_com = self.u.linPart(self.comPoseW) - self.u.linPart(self.basePoseW)
            self.wrench_gW[self.u.sp_crd["AX"]:self.u.sp_crd["AX"]+3] = np.cross(W_base_to_com, self.wrench_gW[self.u.sp_crd["LX"]:self.u.sp_crd["LX"]+3])
        # else the angular wrench is zero



    # Whole body controller that includes ffwd wrench + fb wrench (Virtual PD) + gravity compensation
    # all vector is in the wf
    def WBC(self, des_pose, des_twist, des_acc = None, comControlled = True, type = 'projection'):
        # self.pr.enable()
        self.virtualImpedanceWrench(des_pose, des_twist, des_acc, comControlled)
        self.wrench_desW = self.wrench_fbW + self.wrench_gW + self.wrench_ffW

        # wrench = NEMatrix @ grfs
        for leg in range(self.robot.nee):
            start_col = 3 * leg
            end_col = 3 * (leg + 1)
            if True:#self.contact_state[leg]:
                # ---> linear part
                # identity matrix (I avoid to rewrite zeros)
                self.NEMatrix[self.u.sp_crd["LX"], start_col] = 1.
                self.NEMatrix[self.u.sp_crd["LY"], start_col + 1] = 1.
                self.NEMatrix[self.u.sp_crd["LZ"], start_col + 2] = 1.
                # ---> angular part
                # all in a function
                self.NEMatrix[self.u.sp_crd["AX"]:self.u.sp_crd["AZ"] + 1, start_col:end_col] = \
                    pin.skew(self.W_contacts[leg] - self.u.linPart(self.comPoseW))
            else:
                # clean the matrix
                self.NEMatrix[:, start_col:end_col] = 0.

        # Map the desired wrench to grf
        if type == 'projection':
            self.grForcesW_des = self.projectionWBC()
        elif type == 'qp':
            self.grForcesW_des = self.qpWBC()

        WBC_torques = np.empty(12)
        h_jointFromRos = self.u.mapFromRos(self.h_joints)
        for leg in range(4):
            tau_leg = self.u.getLegJointState(leg, h_jointFromRos) - \
                      self.wJ[leg].T @ self.u.getLegJointState(leg, self.grForcesW_des)
            self.u.setLegJointState(leg, tau_leg, WBC_torques)
        return WBC_torques


    def projectionWBC(self, tol=1e-6):
        return np.linalg.pinv(self.NEMatrix, tol) @ self.wrench_desW

    def setWBCConstraints(self, normals = [np.array([0, 0, 1])*4], friction_coeffs= [0.8]*4):
        # this must be called at least once
        C_leg = [None] * 4

        for leg in range(4):
            ty = np.cross(normals[leg], np.array([1, 0, 0]))
            tx = np.cross(ty, normals[leg])
            coeff_per_normal = friction_coeffs[leg] * normals[leg]
            C_leg[leg] = np.array([
                tx - coeff_per_normal,
                -tx - coeff_per_normal,
                ty - coeff_per_normal,
                -ty - coeff_per_normal])

        self.C_qp = block_diag(block_diag(C_leg[0], C_leg[1], C_leg[2], C_leg[3]))
        self.d_qp = np.zeros(self.C_qp.shape[0])


    def qpWBC(self):
        G = self.NEMatrix.T @ self.NEMatrix + np.eye(12) * 1e-4  # regularize and make it definite positive
        g = -self.NEMatrix.T @ self.wrench_desW

        w_des_grf = quadprog_solve_qp(G, g, self.C_qp, self.d_qp, None , None)
        return w_des_grf



    def support_poly(self, contacts):
        # Wcontacts: lf, rf, lh, rh
        sp = {}
        # CCW order
        sides_order = {'F': [1, 0], 'L': [0, 2], 'H': [2, 3], 'R': [3, 1]}
        for side in sides_order: # side is the key of sides_order
            p0 = contacts[sides_order[side][0]][0:2]
            p1 = contacts[sides_order[side][1]][0:2]
            m, q = self.line2points2D(p0, p1)
            sp['line'+side] = {'m': m, 'q': q, 'p0':p0, 'p1':p1}
        return sp

    @staticmethod
    def line2points2D(p0, p1):
        # line defined as y= mx+q
        m = (p1[1] - p0[1]) / (p1[0] - p0[0])
        q = p0[1] - m * p0[0]
        return m, q

    def send_command(self, q_des=None, qd_des=None, tau_ffwd=None):
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

        self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)

        # if (self.APPLY_EXTERNAL_WRENCH and self.time > self.TIME_EXTERNAL_WRENCH):
        #     print("START APPLYING EXTERNAL WRENCH")
        #     self.applyForce(0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.05)
        #     self.APPLY_EXTERNAL_WRENCH = False

        # log variables
        self.rate.sleep()
        self.logData()
        self.sync_check()
        self.time = np.round(self.time + np.array([self.loop_time]), 3)




    def visualizeContacts(self):
        for legid in self.u.leg_map.keys():

            leg = self.u.leg_map[legid]
            if self.contact_state[leg]:
                self.ros_pub.add_arrow(self.W_contacts[leg],
                                       self.u.getLegJointState(leg, self.grForcesW/ (6*self.robot.robot_mass)),
                                       "green")
                #self.ros_pub.add_marker(self.W_contacts[leg], radius=0.1)
            else:
                self.ros_pub.add_arrow(self.W_contacts[leg],
                                       np.zeros(3),
                                       "green", scale=0.0001)
                #self.ros_pub.add_marker(self.W_contacts[leg], radius=0.001)

            # if (self.use_ground_truth_contacts):
            #     self.ros_pub.add_arrow(self.W_contacts[leg],
            #                            self.u.getLegJointState(leg, self.grForcesW_gt / (6 * self.robot.robot_mass)),
            #                            "red")
            # else:
            #     self.ros_pub.add_arrow(self.W_contacts[leg],
            #                            self.u.getLegJointState(leg,self.grForcesW_des / (6 * self.robot.robot_mass)),
            #                            "red")
            self.ros_pub.add_arrow(self.W_contacts[leg],
                                   self.u.getLegJointState(leg, self.grForcesW_des / (6 * self.robot.robot_mass)),
                                   "blue")

        # self.ros_pub.add_polygon([self.B_contacts[0],
        #                           self.B_contacts[1],
        #                           self.B_contacts[3],
        #                           self.B_contacts[2],
        #                           self.B_contacts[0] ], "red", visual_frame="base_link")
        #
        # self.ros_pub.add_polygon([self.B_contacts_des[0],
        #                           self.B_contacts_des[1],
        #                           self.B_contacts_des[3],
        #                           self.B_contacts_des[2],
        #                           self.B_contacts_des[0]], "green", visual_frame="base_link")
        #
        # self.ros_pub.add_marker_fixed(self.zmp)
        self.ros_pub.publishVisual()





    def startupProcedure(self):
        ros.sleep(.5)
        print(colored("Starting up", "blue"))
        if self.robot_name == 'hyq':
            super(Controller, self).startupProcedure()
            return

        # wait for user confirmation to start
        # if self.real_robot:
        #     if input('press ENTER to continue/any key to STOP') != '':
        #         exit(0)
        self.q_des = self.q.copy()
        # if self.real_robot:
        #     self.q_des = self.q.copy()
        # else:
        #     self.q_des = self.u.mapToRos(conf.robot_params[self.robot_name]['q_fold'])
        self.pid = PidManager(self.joint_names)
        if self.real_robot:
            self.pid.setPDjoints(conf.robot_params[self.robot_name]['kp_real'],
                                 conf.robot_params[self.robot_name]['kd_real'],
                                 np.zeros(self.robot.na))
        else:
            self.pid.setPDjoints(conf.robot_params[self.robot_name]['kp'],
                                 conf.robot_params[self.robot_name]['kd'],
                                 np.zeros(self.robot.na))

        for i in range(10):
            self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
            ros.sleep(0.01)



        if self.go0_conf == 'standUp':
            self._startup_from_stand_up()
        elif self.go0_conf == 'standDown':
            self._startup_from_stand_down()

        # reset time to zero (I don't want to log startup)
        self.time = np.zeros(1)
        self.log_counter = 0



    def _startup_from_stand_up(self):
        self.q_des = conf.robot_params[self.robot_name]['q_0']
        alpha = 0.
        try:
            print(colored("[startupProcedure] applying gravity compensation", "blue"))
            GCStartTime = self.time
            while True:
                q_norm = np.linalg.norm(self.q - self.q_des)
                qd_norm = np.linalg.norm(self.qd - self.qd_des)
                if q_norm < 0.1 and qd_norm < 0.1 or self.time > 5:
                    break
                self.updateKinematics()
                self.visualizeContacts()
                GCTime = self.time - GCStartTime
                if GCTime <= 0.6:
                    if alpha < 1:
                        alpha = GCTime / 0.5

                self.send_command(self.q_des, self.qd_des, alpha*self.gravityCompensation())

            # IMU BIAS ESTIMATION
            print(colored("[startupProcedure] Imu bias estimation", "blue"))
            if self.real_robot and self.robot_name == 'go1':
                # print('counter: ' + self.imu_utils.counter + ', timeout: ' + self.imu_utils.timeout)
                while self.imu_utils.counter < self.imu_utils.timeout:
                    self.updateKinematics()
                    self.imu_utils.IMU_bias_estimation(self.b_R_w, self.B_imu_lin_acc)
                    self.tau_ffwd[:] = 0.
                    self.send_command(self.q_des, self.qd_des, self.tau_ffwd)


        except (ros.ROSInterruptException, ros.service.ServiceException):
            ros.signal_shutdown("killed")
            self.deregister_node()



    def _startup_from_stand_down(self):
        for i in range(12):
            # modify HAAs
            if (i%3) != 0:
                self.q_des[i] =  conf.robot_params[self.robot_name]['q_fold'][i]
        # IMU BIAS ESTIMATION
        print(colored("[startupProcedure] Imu bias estimation", "blue"))
        #if self.real_robot and self.robot_name == 'go1':
            # print('counter: ' + self.imu_utils.counter + ', timeout: ' + self.imu_utils.timeout)
        while self.imu_utils.counter < self.imu_utils.timeout:
            self.updateKinematics()
            self.imu_utils.IMU_bias_estimation(self.b_R_w, self.B_imu_lin_acc)
            self.tau_ffwd[:] = 0.
            self.send_command(self.q_des, self.qd_des, self.tau_ffwd)


        # initial feet position
        B_feet_pose = [np.zeros(3)] * 4
        neutral_fb_jointstate = np.hstack((pin.neutral(self.robot.model)[0:7], self.u.mapToRos(self.q)))#conf.robot_params[self.robot_name]['q_fold']))
        pin.forwardKinematics(self.robot.model, self.robot.data, neutral_fb_jointstate)
        pin.updateFramePlacements(self.robot.model, self.robot.data)

        for leg in range(4):
            foot = conf.robot_params[self.robot_name]['ee_frames'][leg]
            foot_id = self.robot.model.getFrameId(foot)
            B_feet_pose[leg] = self.robot.data.oMf[foot_id].translation.copy()
        # desired final height
        neutral_fb_jointstate[7:] = self.u.mapToRos(conf.robot_params[self.robot_name]['q_0'])

        self.robot.forwardKinematics(neutral_fb_jointstate)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        robot_height = 0.
        for id in self.robot.getEndEffectorsFrameId:
            robot_height += self.robot.data.oMf[id].translation[2]
        robot_height /= -4.

        # increase of the motion
        delta_z = 0.0005
        ########################
        # FINITE STATE MACHINE #
        ########################
        # state = -1: initialize pid
        # state = 0: not all the contacts are active, move the feet in order to activate all the contacts (use IK+PD)
        # state = 1: apply gravity compensation for 1 second
        # state = 2: apply PD + gravity compensation
        # state = 3: exit
        state = 0
        print(colored("[startupProcedure] searching contacts", "blue"))
        try:
            while not ros.is_shutdown() and state != 4:
                self.updateKinematics()
                self.visualizeContacts()
                # if state == -1:
                #     print(self.q-self.q_des)
                #     if np.linalg.norm(self.q-self.q_des) > 0.05:
                #         self.q_des = self.u.mapToRos(conf.robot_params[self.robot_name]['q_fold'])
                #         self.qd_des[:] = 0.
                #         self.tau_ffwd[:] = 0.
                #     else:
                #         state = 0

                if state == 0:
                    if not self.contact_state.all(): # if at least one is not in contact
                        for leg in range(4):
                            if not self.contact_state[leg]:
                                # update feet task
                                # if a leg is in contact, it must keep the same reference (waiting for the others)
                                foot = conf.robot_params[self.robot_name]['ee_frames'][leg]
                                foot_id = self.robot.model.getFrameId(foot)
                                leg_name = foot[:2]
                                B_feet_pose[leg][2] -= delta_z
                                q_des_leg = self.IK.ik_leg(self.b_R_w.T@B_feet_pose[leg], foot_id,self. legConfig[leg_name][0],self. legConfig[leg_name][1])[0].flatten()
                                self.u.setLegJointState(leg, q_des_leg, self.q_des)
                        # if self.log_counter != 0:
                        #     self.qd_des = (self.q_des - self.q_des_log[:, self.log_counter]) / self.dt
                        self.tau_ffwd[:] = 0.

                    else:
                        print(colored("[startupProcedure] appling gravity compensation", "blue"))
                        state = 1
                        GCStartTime = self.time
                        alpha = 0.

                if state == 1:
                    GCTime = self.time-GCStartTime
                    self.qd_des[:] = 0
                    if GCTime <= 0.6:
                        if alpha < 1:
                            alpha = GCTime / 0.5
                        self.tau_ffwd = alpha * (self.gravityCompensation() + self.self_weightCompensation())
                    else:
                        print(colored("[startupProcedure] moving to desired height (" + str(np.around(robot_height, 3)) +" m)", "blue"))
                        state = 2

                if state == 2:
                    neutral_fb_jointstate[7:] = self.u.mapToRos(self.q)
                    self.robot.forwardKinematics(neutral_fb_jointstate)
                    pin.updateFramePlacements(self.robot.model, self.robot.data)
                    current_robot_height = 0.
                    for id in self.robot.getEndEffectorsFrameId:
                        current_robot_height += self.robot.data.oMf[id].translation[2]
                    current_robot_height /= -4.

                    if np.abs(current_robot_height - robot_height) > 0.005:
                        # set reference
                        for leg in range(4):
                            foot = conf.robot_params[self.robot_name]['ee_frames'][leg]
                            foot_id = self.robot.model.getFrameId(foot)
                            leg_name = foot[:2]
                            B_feet_pose[leg][2] -= delta_z
                            if current_robot_height <= robot_height:
                                B_foot = B_feet_pose[leg] # self.b_R_w.T @ b_R_w_init @ B_feet_pose[leg]
                                q_des_leg = self.IK.ik_leg(self.b_R_w.T@B_foot, foot_id,self. legConfig[leg_name][0],self. legConfig[leg_name][1])[0].flatten()
                                self.u.setLegJointState(leg, q_des_leg, self.q_des)

                        self.tau_ffwd = self.gravityCompensation()

                    else:
                        print(colored("[startupProcedure] desired height reached", "blue"))
                        state = 3
                        WTime = self.time

                if state == 3:
                    #if (self.time - WTime) <= 0.5:

                    if all(np.abs(self.qd) < 0.025) and (self.time - WTime) >0.5:
                        print(colored("[startupProcedure] completed", "green"))
                        state = 4
                    else:
                        # enter
                        # if any of the joint position errors is larger than 0.02 or
                        # if any of the joint velocities is larger than 0.02 or
                        # if the watchdog timer is not expired (1 sec)
                        self.tau_ffwd = self.gravityCompensation()


                self.send_command(self.q_des, self.qd_des, self.tau_ffwd)

        except (ros.ROSInterruptException, ros.service.ServiceException):
            ros.signal_shutdown("killed")
            self.deregister_node()

        #self.time = 0.



    def save_reference(self, filename):
        DATA = {}
        DATA['last_sample'] = self.log_counter
        DATA['q_des'] = self.q_des_log[:, :self.log_counter]
        DATA['qd_des'] = self.qd_des_log[:, :self.log_counter]
        DATA['tau_ffwd'] = self.tau_ffwd_log[:, :self.log_counter]

        savemat(filename, DATA)

        print('Reference saved in', filename)

    def get_current_frame_file(self):
        allfiles = [f for f in os.listdir('/tmp/camera_save') if os.path.isfile(os.path.join('/tmp/camera_save', f))]
        return max(allfiles)

    def save_video(self, path, start_file=None, filename='record', format='mkv',  fps=60, speedUpDown=1, remove_jpg=True):
        # only if camera_xxx.world has been used
        # for details on commands, check https://ffmpeg.org/ffmpeg.html
        if 'camera' not in self.world_name_str:
            print(colored('Cannot create a video of a not camera world (world_name:'+self.world_name_str+')', 'red'), flush=True)
            return
        #
        # # kill gazebo
        # os.system("killall rosmaster rviz gzserver gzclient")

        if start_file != None:
            # delete all the files before start_file
            allfiles = [f for f in os.listdir('/tmp/camera_save') if os.path.isfile(os.path.join('/tmp/camera_save', f))]
            for f in allfiles:
                if f < start_file:
                    #parentheses are not supported by bash
                    f = f.replace('(', '\(')
                    f = f.replace(')', '\)')
                    cmd = 'rm /tmp/camera_save/' + f
                    os.system(cmd)

        if '.' in filename:
            filename=filename[:, filename.find('.')]
        if path[-1] != '/':
            path+='/'
        videoname = path + filename + '.' + format
        save_video_cmd = "ffmpeg -hide_banner -loglevel error -r "+str(fps)+" -pattern_type glob -i '/tmp/camera_save/default_camera_link_my_camera*.jpg' -c:v libx264 "+videoname
        ret = os.system(save_video_cmd)
        saved = ''
        if ret == 0:
            saved = ' saved'
        else:
            saved = ' did not saved'
        print(colored('Video '+videoname+saved, color='green'), flush=True)


        if speedUpDown <= 0:
            print(colored('speedUpDown must be greather than 0.0','red'))
        else:
            if speedUpDown != 1:
                pts_multiplier = int(1 / speedUpDown)
                videoname_speedUpDown = path + filename + str(speedUpDown).replace('.', '')+'x.'+format
                speedUpDown_cmd = "ffmpeg -hide_banner -loglevel error -i "+videoname+" -filter:v 'setpts="+str(pts_multiplier)+"*PTS' "+videoname_speedUpDown
                ret = os.system(speedUpDown_cmd)
                saved = ''
                if ret == 0:
                    saved= ' saved'
                else:
                    saved = ' did not saved'
                print(colored('Video ' + videoname_speedUpDown+saved, color='green'), flush=True)

        if remove_jpg:
            remove_jpg_cmd = "rm /tmp/camera_save/default_camera_link_my_camera*.jpg"
            os.system(remove_jpg_cmd)
            print(colored('Jpg files removed', color='green'), flush=True)







if __name__ == '__main__':
    p = Controller('solo')
    try:
        #p.startController(world_name='slow.world')
        p.startController()

        while not ros.is_shutdown():
            p.tau_ffwd = p.gravityCompensation() + p.self_weightCompensation()
            p.send_command(p.q_des, p.qd_des, p.tau_ffwd)



    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()

    import matplotlib

    matplotlib.use('TkAgg')
    from base_controllers.utils.common_functions import plotJoint, plotCoM, plotGRFs

    plotJoint('position', 0, p.time_log.flatten(), p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None,
              p.tau_log,
              p.tau_ffwd_log, p.joint_names)
    plotJoint('torque', 1, p.time_log.flatten(), p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None,
              p.tau_log,
              p.tau_ffwd_log, p.joint_names)
    #plotCoM('position', 1, p.time_log.flatten(), None, p.basePoseW_log, None, p.baseTwistW_log, None, None)
    # stats = pstats.Stats(profiler).sort_stats('cumtime')
    # stats.print_stats()

#
# def talker(p):
#     p.start()
#     if (p.robot_name == 'aliengo') or (p.robot_name == 'solo_fw'):
#         p.custom_launch_file = True
#     p.startSimulator("slow.world")
#     p.loadModelAndPublishers()
#     p.initVars()
#     p.startupProcedure()
#
#     # loop frequency
#     rate = ros.Rate(1 / conf.robot_params[p.robot_name]['dt'])
#
#     firstTime = True
#     # control loop
#     while not ros.is_shutdown():
#         # update the kinematics
#         p.updateKinematics()
#         p.visualizeContacts()
#         #print(p.q)
#         p.tau_ffwd = p.gravityCompensation() #+ p.u.mapFromRos(p.h_joints)
#
#         p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
#
#         # log variables
#         p.logData()
#
#         if firstTime and (p.time > 3.0):
#             print("AAAAAAAAAAAAAAAAAAAAAAA")
#             p.q_des = np.copy(p.q)
#             firstTime = False
#
#         # wait for synconization of the control loop
#         rate.sleep()
#         p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999
#
#
#
# if __name__ == '__main__':
#     p = Controller('go1')
#     try:
#         talker(p)
#     except (ros.ROSInterruptException, ros.service.ServiceException):
#         ros.signal_shutdown("killed")
#         p.deregister_node()
#         if conf.plotting:
#             import matplotlib
#             matplotlib.use('TkAgg')
#             from base_controllers.utils.common_functions import plotJoint, plotCoM, plotGRFs
#             plotJoint('position', 0, p.time_log.flatten(), p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
#                       p.tau_ffwd_log, p.joint_names)
#             plotCoM('position', 1, p.time_log.flatten(), None, p.basePoseW_log, None, p.baseTwistW_log, None, None)
#             plotGRFs(2, p.time_log.flatten(), p.grForcesW_gt_log, p.grForcesW_log)
#
#

from __future__ import print_function
import sys
import os
from gazebo_ros import gazebo_interface
from base_controllers.utils.pidManager import PidManager
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from base_controllers.utils.common_functions import *
from base_controllers.utils.custom_robot_wrapper import RobotWrapper

import rospy as ros
import numpy as np
import rospkg
import pinocchio as pin
# utility functions
from base_controllers.utils.math_tools import *
from termcolor import colored
from base_controllers.quadruped_controller import QuadrupedController
import base_controllers.params as conf
np.set_printoptions(threshold=np.inf, precision=5,
                    linewidth=10000, suppress=True)
np.set_printoptions(threshold=np.inf, precision=5,
                    linewidth=1000, suppress=True)


class JumpAgent():
    def __init__(self, cfg: dict):

        self.cfg = cfg

        self.min_action = self.cfg["min_action"]
        self.max_action = self.cfg["max_action"]

        self.lerp_time = self.cfg["lerp_time"]

        self.t_th_min = self.cfg["t_th_min"]
        self.t_th_max = self.cfg["t_th_max"]

        self.x_theta_min = self.cfg["x_theta_min"]
        self.x_theta_max = self.cfg["x_theta_max"]

        self.x_r_min = self.cfg["x_r_min"]
        self.x_r_max = self.cfg["x_r_max"]

        self.xd_theta_min = self.cfg["xd_theta_min"]
        self.xd_theta_max = self.cfg["xd_theta_max"]

        self.xd_r_min = self.cfg["xd_r_min"]
        self.xd_r_max = self.cfg["xd_r_max"]

        self.psi_min = self.cfg["psi_min"]
        self.psi_max = self.cfg["psi_max"]

        self.theta_min = self.cfg["theta_min"]
        self.theta_max = self.cfg["theta_max"]

        self.phi_min = self.cfg["phi_min"]
        self.phi_max = self.cfg["phi_max"]

        self.psid_min = self.cfg["psid_min"]
        self.psid_max = self.cfg["psid_max"]

        self.thetad_min = self.cfg["thetad_min"]
        self.thetad_max = self.cfg["thetad_max"]

        self.phid_min = self.cfg["phid_min"]
        self.phid_max = self.cfg["phid_max"]

        self.xd_mult_min = self.cfg["xd_mult_min"]
        self.xd_mult_max = self.cfg["xd_mult_max"]

        self.l_expl_min = self.cfg["l_expl_min"]
        self.l_expl_max = self.cfg["l_expl_max"]

    def cart2sph(self, pos, threshold=1e-5):
        # Extract x, y, z components
        x = pos[:, 0]
        y = pos[:, 1]
        z = pos[:, 2]

        # Compute spherical coordinates
        hxy = np.hypot(x, y)
        r = np.hypot(hxy, z)
        el = np.arctan2(z, hxy)
        az = np.arctan2(y, x)

        # Concatenate azimuth, elevation, and radius
        spherical = np.stack((az, el, r), axis=1)

        return spherical

    def sph2cart(self, pos):
        # Extract az, el, r components
        az = pos[:, 0]
        el = pos[:, 1]
        r = pos[:, 2]

        rcos_theta = r * np.cos(el)
        x = rcos_theta * np.cos(az)
        y = rcos_theta * np.sin(az)
        z = r * np.sin(el)

        # Concatenate x, y, and z
        cartesian = np.stack((x, y, z), axis=1)

        return cartesian

    def map_range(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def process_actions(self, actions, target):

        self.t_th = self.map_range(
            actions[..., 0], self.min_action, self.max_action, self.t_th_min, self.t_th_max)
        self.t_th = self.t_th.reshape(-1, 1)

        x_xd_phi = self.cart2sph(target[None])[:, 0].item()

        # Trunk x_lo
        x_theta = self.map_range(
            actions[..., 1], self.min_action, self.max_action, self.x_theta_min, self.x_theta_max)
        x_r = self.map_range(
            actions[..., 2], self.min_action, self.max_action, self.x_r_min, self.x_r_max)
        print(x_xd_phi, x_theta, x_r)
        self.trunk_x_lo = self.sph2cart(
            np.stack((x_xd_phi, x_theta, x_r))[None])

        # Trunk xd_lo
        xd_theta = self.map_range(
            actions[..., 3], self.min_action, self.max_action, self.xd_theta_min, self.xd_theta_max)
        xd_r = self.map_range(
            actions[..., 4], self.min_action, self.max_action, self.xd_r_min, self.xd_r_max)
        self.trunk_xd_lo = self.sph2cart(
            np.stack((x_xd_phi, xd_theta, xd_r))[None])

        # Trunk o_lo
        psi = self.map_range(
            actions[..., 5], self.min_action, self.max_action, self.psi_min, self.psi_max)
        theta = self.map_range(
            actions[..., 6], self.min_action, self.max_action, self.theta_min, self.theta_max)
        phi = self.map_range(
            actions[..., 7], self.min_action, self.max_action, self.phi_min, self.phi_max)

        self.trunk_o_lo = np.stack((psi, theta, phi))

        # Trunk od_lo
        psid = self.map_range(
            actions[..., 8], self.min_action, self.max_action, self.psid_min, self.psid_max)
        thetad = self.map_range(
            actions[..., 9], self.min_action, self.max_action, self.thetad_min, self.thetad_max)
        phid = self.map_range(
            actions[..., 10], self.min_action, self.max_action, self.phid_min, self.phid_max)

        self.trunk_od_lo = np.stack((psid, thetad, phid))

        xd_mult = self.map_range(
            actions[..., 11], self.min_action, self.max_action, self.xd_mult_min, self.xd_mult_max)
        l_expl = self.map_range(
            actions[..., 12], self.min_action, self.max_action, self.l_expl_min, self.l_expl_max)

        trunk_xd_lo_un = self.trunk_xd_lo / np.linalg.norm(self.trunk_xd_lo)
        self.trunk_x_exp = self.trunk_x_lo + \
            trunk_xd_lo_un * l_expl.reshape(-1, 1)

        self.trunk_xd_exp = self.trunk_xd_lo * xd_mult.reshape(-1, 1)

        vf_n = np.linalg.norm(self.trunk_xd_exp)
        v0_n = np.linalg.norm(self.trunk_xd_lo)
        sf_n = np.linalg.norm(self.trunk_x_exp)
        s0_n = np.linalg.norm(self.trunk_x_lo)

        self.a = 0.5 * ((np.power(vf_n, 2) - np.power(v0_n, 2)) /
                        ((sf_n - s0_n) + 1e-15))

        self.t_exp = ((vf_n - v0_n) / (self.a + 1e-15)).reshape(-1, 1)
        self.t_th_total = self.t_th + self.t_exp

        print(f"Processed action:\n\
                    t_th: {self.t_th}\n\
                    trunk_x_lo: {self.trunk_x_lo}\n\
                    trunk_xd_lo: {self.trunk_xd_lo}\n\
                    trunk_o_lo: {self.trunk_o_lo}\n\
                    trunk_od_lo: {self.trunk_od_lo}\n\
                    xd_mult: {xd_mult}\n\
                    l_expl: {l_expl}\n\
                    trunk_x_exp: {self.trunk_x_exp}\n\
                    trunk_xd_exp: {self.trunk_xd_exp}\n\
                    t_exp: {self.t_exp}\n\
                    ")


class QuadrupedJumpController(QuadrupedController):
    def __init__(self, robot_name="hyq", launch_file=None):
        super(QuadrupedJumpController, self).__init__(robot_name, launch_file)
        self.use_gui = False
        self.DEBUG = False
        self.ffwd_impulse = False
        self.jumpAgent = JumpAgent(cfg={"min_action": -5,
                                        "max_action": 5,
                                        "lerp_time": 0.1,
                                        "t_th_min": 0.1,
                                        "t_th_max": 0.7,
                                        "x_theta_min": np.pi / 4,
                                        "x_theta_max": np.pi / 2,
                                        "x_r_min": 0.2,
                                        "x_r_max": 0.4,
                                        "xd_theta_min": np.pi / 6,
                                        "xd_theta_max": np.pi / 2,
                                        "xd_r_min": 0.1,
                                        "xd_r_max": 3,
                                        "psi_min": -np.pi / 4,
                                        "psi_max": np.pi / 4,
                                        "theta_min": -np.pi / 4,
                                        "theta_max": np.pi / 4,
                                        "phi_min": -np.pi,
                                        "phi_max": np.pi,
                                        "psid_min": -4,
                                        "psid_max": 4,
                                        "thetad_min": -4,
                                        "thetad_max": 4,
                                        "phid_min": -4,
                                        "phid_max": 4,
                                        "xd_mult_min": 1,
                                        "xd_mult_max": 5,
                                        "l_expl_min": 0,
                                        "l_expl_max": 0.3})
        self.go0_conf = 'standDown'
        self.q_0_td = conf.robot_params[self.robot_name]['q_0_td']
        self.q_0_lo = conf.robot_params[self.robot_name]['q_0_lo']
        print("Initialized Quadruped Jump controller---------------------------------------------------------------")

    def initVars(self):
        super().initVars()
        self.intermediate_com_position = []
        self.intermediate_flight_com_position = []
        self.ideal_landing = np.zeros(3)
        self.firstTime = True
        self.detectedApexFlag = False
        self.rearingFlag = False
        self.ideal_landing = np.zeros(3)
        self.landing_position = np.zeros(3)
        self.time = np.zeros(1)  # reset time for logging

        self.reset_joints = ros.ServiceProxy(
            '/gazebo/set_model_configuration', SetModelConfiguration)
        self.set_state = ros.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        if not self.real_robot:
            spawnModel('go1_description', 'jump_platform')

        # TODO
        # print("JumplegAgent services ready")
        # self.action_service = ros.ServiceProxy(
        #     'JumplegAgent/get_action', get_action)
        # self.target_service = ros.ServiceProxy(
        #     'JumplegAgent/get_target', get_target)
        # self.reward_service = ros.ServiceProxy(
        #     'JumplegAgent/set_reward', set_reward)

    def detectApex(self, threshold=-3):
        # foot tradius is 0.015
        foot_lifted_off = np.array([False, False, False, False])
        for leg in range(4):
            foot_lifted_off[leg] = self.W_contacts[leg][2] > 0.017
        if not self.detectedApexFlag and np.all(foot_lifted_off):
            # Reducing gains for more complaint landing
            p.pid.setPDjoints(conf.robot_params[p.robot_name]['kp'] / 4,
                              conf.robot_params[p.robot_name]['kd'], np.zeros(p.robot.na))
            if self.real_robot:
                if self.baseLinAccW[2] < threshold:
                    self.detectedApexFlag = True
                    print(colored("APEX detected", "red"))
                    self.q_apex = self.q.copy()
                    self.t_apex = self.time
            else:
                if self.baseTwistW[2] < 0.0:
                    self.detectedApexFlag = True
                    self.pause_physics_client()
                    for i in range(10):
                        self.setJumpPlatformPosition(self.target_CoM, com_0)
                    self.unpause_physics_client()
                    print(colored("APEX detected", "red"))
                    self.q_apex = self.q.copy()
                    self.t_apex = self.time

    def detectTouchDown(self):
        # contact = np.array([False,False,False,False])
        # for leg in range(4):
        #     contact[leg] = np.linalg.norm(self.grForcesW)>self.force_th
        if np.all(self.contact_state):
            print(colored("TOUCHDOWN detected", "red"))
            return True
        else:
            return False

    def resetRobot(self, basePoseDes=np.array([0, 0, 0.3, 0., 0., 0.])):
        # this sets the position of the joints
        gazebo_interface.set_model_configuration_client(
            self.robot_name, '', self.joint_names, self.qj_0, '/gazebo')
        self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
        # this sets the position of the base
        self.freezeBase(False,  basePoseW=basePoseDes)

    def computeHeuristicSolutionBezierLinear(self, com_0, com_lo, comd_lo, T_th):
        self.bezier_weights_lin = np.zeros([3, 4])
        comd_0 = np.zeros(3)
        self.bezier_weights_lin = np.zeros([3, 4])
        self.bezier_weights_lin[:, 0] = com_0
        self.bezier_weights_lin[:, 1] = (comd_0*(T_th/3.)) + com_0
        self.bezier_weights_lin[:, 2] = com_lo - (comd_lo*(T_th/3.))
        self.bezier_weights_lin[:, 3] = com_lo

    def computeHeuristicSolutionBezierAngular(self, eul_0, eul_lo, euld_lo, T_th):
        self.bezier_weights_ang = np.zeros([3, 4])
        euld_0 = np.zeros(3)
        self.bezier_weights_ang = np.zeros([3, 4])
        self.bezier_weights_ang[:, 0] = eul_0
        self.bezier_weights_ang[:, 1] = (euld_0*(T_th/3.)) + eul_0
        self.bezier_weights_ang[:, 2] = eul_lo - (euld_lo*(T_th/3.))
        self.bezier_weights_ang[:, 3] = eul_lo

    def evalBezier(self, t_, T_th, T_expl, ffwd_impulse=False, a_z=np.zeros((4))):
        T_th_total = T_th + T_expl

        if t_ < T_th:
            com = np.array(self.Bezier3(self.bezier_weights_lin, t_, T_th))
            comd = np.array(self.Bezier2(self.bezier_weights_lin, t_, T_th))
            comdd = np.array(self.Bezier1(self.bezier_weights_lin, t_, T_th))
        else:
            t = t_ - T_th
            t_0 = (t-0.002) / T_expl
            t_1 = np.clip(t / T_expl, 0, 1)
            com = self.lerp(self.jumpAgent.trunk_x_lo,
                            self.jumpAgent.trunk_x_exp, t_1)[0]
            comd = self.lerp(self.jumpAgent.trunk_xd_lo,
                             self.jumpAgent.trunk_xd_exp, t_1)[0]
            comd_p = self.lerp(self.jumpAgent.trunk_xd_lo,
                               self.jumpAgent.trunk_xd_exp, t_0)[0]
            # TODO: do not harcode time division
            comdd = (comd-comd_p)/0.002

        eul = np.array(self.Bezier3(self.bezier_weights_ang, t_, T_th_total))
        euld = np.array(self.Bezier2(self.bezier_weights_ang, t_, T_th_total))
        euldd = np.array(self.Bezier1(self.bezier_weights_ang, t_, T_th_total))

        if self.DEBUG:
            freq = 0.5
            amp_lin = np.array([0., 0., 0.05])
            amp_ang = np.array([0., 0.0, 0])
            com = self.initial_com + \
                np.multiply(amp_lin, np.sin(2*np.pi*freq * self.time))
            comd = np.multiply(2*np.pi*freq*amp_lin,
                               np.cos(2*np.pi*freq * self.time))
            comdd = np.multiply(
                np.power(2*np.pi*freq*amp_lin, 2), -np.sin(2*np.pi*freq * self.time))
            eul = np.array([0., 0.0, 0]) + np.multiply(amp_ang,
                                                       np.sin(2 * np.pi * freq * self.time))
            euld = np.multiply(2 * np.pi * freq * amp_ang,
                               np.cos(2 * np.pi * freq * self.time))
            euldd = np.multiply(
                np.power(2 * np.pi * freq * amp_ang, 2), -np.sin(2 * np.pi * freq * self.time))

        Jb = p.computeJcb(self.W_contacts_sampled, com, self.stance_legs)

        W_des_basePose = np.empty(6)
        W_des_basePose[self.u.sp_crd['LX']:self.u.sp_crd['LX'] + 3] = com
        W_des_basePose[self.u.sp_crd['AX']:self.u.sp_crd['AX'] + 3] = eul

        W_des_baseTwist = np.empty(6)
        W_des_baseTwist[self.u.sp_crd['LX']:self.u.sp_crd['LX'] + 3] = comd
        Jomega = self.math_utils.Tomega(eul)
        W_des_baseTwist[self.u.sp_crd['AX']:self.u.sp_crd['AX'] +
                        3] = self.math_utils.Tomega(eul).dot(euld)

        W_des_baseAcc = np.empty(6)
        W_des_baseAcc[self.u.sp_crd['LX']:self.u.sp_crd['LX'] + 3] = comdd
        # compute w_omega_dot =  Jomega* euler_rates_dot + Jomega_dot*euler_rates (Jomega already computed, see above)
        Jomega_dot = self.math_utils.Tomega_dot(eul, euld)
        W_des_baseAcc[self.u.sp_crd['AX']:self.u.sp_crd['AX'] + 3] = Jomega @ euldd + \
            Jomega_dot @ euld

        # map base twist into feet relative vel (wrt com/base)
        W_feetRelVelDes = -Jb.dot(W_des_baseTwist)
        w_R_b_des = self.math_utils.eul2Rot(eul)

        if ffwd_impulse:
            # compute ffwd torques
            self.bezier_weights_impulze_z = np.zeros((4))
            self.bezier_weights_impulze_z[0] = 0
            # this is to have the initial unloading
            self.bezier_weights_impulze_z[1] = -1
            self.bezier_weights_impulze_z[2] = 1.7
            self.bezier_weights_impulze_z[3] = 1  # this is to end up pushing
            impulse_z = np.array(self.Bezier3(
                self.bezier_weights_impulze_z, t_, T_th))

        grf_ffwd = np.zeros(12)
        tau_ffwd = np.zeros(12)
        qd_des = np.zeros(12)
        q_des = np.zeros(12)
        fbjoints = pin.neutral(self.robot.model)
        w_J = self.u.listOfArrays(4, np.zeros((3, 3)))
        # integrate relative Velocity
        for leg in range(self.robot.nee):
            # with this you do not have proper tracking of com and trunk orientation, I think there is a bug in the ik
            # self.W_feetRelPosDes[leg] += W_feetRelVelDes[3 * leg:3 * (leg+1)]*self.dt
            # this has better tracking
            # should use desired values to generate traj otherwise if it is unstable it detroys the ref signal
            self.W_feetRelPosDes[leg] = self.W_contacts_sampled[leg] - com
            # now we can do Ik
            q_des[3 * leg:3 * (leg+1)], isFeasible = self.IK.ik_leg(w_R_b_des.T.dot(self.W_feetRelPosDes[leg]),
                                                                    self.leg_names[leg],
                                                                    self.legConfig[self.leg_names[leg]][0],
                                                                    self.legConfig[self.leg_names[leg]][1])
            # for joint velocity we need to recompute the Jacobian (in the world frame) for the computed joint position q_des
            # you need to fill in also the floating base part
            quat_des = pin.Quaternion(w_R_b_des)
            fbjoints[:3] = com
            fbjoints[3:7] = np.array(
                [quat_des.x, quat_des.y, quat_des.z, quat_des.w])
            fbjoints[7:] = q_des

            pin.forwardKinematics(self.des_robot.model, self.des_robot.data, fbjoints, np.zeros(
                self.des_robot.model.nv),   np.zeros(self.des_robot.model.nv))
            pin.computeJointJacobians(
                self.des_robot.model, self.des_robot.data)
            pin.computeFrameJacobian(self.des_robot.model, self.des_robot.data,
                                     fbjoints, p.des_robot.model.getFrameId(self.ee_frames[leg]))
            w_J[leg] = pin.getFrameJacobian(self.des_robot.model, self.des_robot.data,
                                            p.des_robot.model.getFrameId(
                                                self.ee_frames[leg]),
                                            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 6 + leg * 3:6 + leg * 3 + 3]
            # compute joint variables
            qd_des[3 * leg:3 * (leg+1)] = np.linalg.pinv(w_J[leg]
                                                         ).dot(W_feetRelVelDes[3 * leg:3 * (leg+1)])

        if not ffwd_impulse:
            tau_ffwd = self.WBC(W_des_basePose, W_des_baseTwist, W_des_baseAcc,
                                comControlled=False, type='projection', stance_legs=self.stance_legs)
            # OLD
            # tau_ffwd = self.gravityCompensation()
        else:
            for leg in range(self.robot.nee):
                # todo do for x and y as well
                grfDes = np.array([0., 0., impulse_z * a_z[leg]])
                tau_leg = -w_J[leg].T.dot(grfDes)
                self.u.setLegJointState(leg, grfDes, grf_ffwd)
                self.u.setLegJointState(leg, tau_leg, tau_ffwd)
            tau_ffwd += self.gravityCompensation()  # this also sets grForcesW_des
            # on top we add our ffwd
            self.grForcesW_des += grf_ffwd

        # check unloading of front legs
        # grf_lf = self.u.getLegJointState(0, self.grForcesW)
        # grf_rf = self.u.getLegJointState(2, self.grForcesW)
        # #if unloaded raise front legs and compute Jb for a subset of lefs
        # if not self.rearingFlag  and (grf_lf[2] < 5.) and (grf_rf[2] < 5.): # LF RF
        #     self.rearingFlag = True
        #     self.stance_legs = [False, True, False, True]
        #     print(colored(f"rearing front legs at time {self.time}", "red"))
        #
        # #overwrite front legs
        # if self.rearingFlag:
        #     for leg in range(self.robot.nee):
        #         if self.stance_legs[leg]:
        #             q_des[3 * leg:3 * (leg+1)] = p.qj_0[3 * leg:3 * (leg+1)]
        #             qd_des[3 * leg:3 * (leg+1)] = np.zeros(3)
        #             #the small torques to compensate self weight of the legs in the air have been already computed by WBC
        return q_des, qd_des, tau_ffwd, W_des_basePose, W_des_baseTwist

    def plotTrajectoryBezier(self):
        # plot com intermediate positions
        for blob in range(len(self.intermediate_com_position)):
            self.ros_pub.add_marker(self.intermediate_com_position[blob], color=[
                blob * 1. / self.number_of_blobs, blob * 1. / self.number_of_blobs, blob * 1. / self.number_of_blobs],
                radius=0.05)

    def computeIdealLanding(self, com_lo, comd_lo, target_CoM):
        # get time of flight
        arg = comd_lo[2] * comd_lo[2] - 2 * 9.81 * (target_CoM[2] - com_lo[2])
        if arg < 0:
            print(colored("Point Beyond Reach, tagret too high", "red"))
            return False
        else:  # beyond apex
            self.T_fl = (comd_lo[2] + math.sqrt(arg)) / \
                9.81  # we take the highest value
            self.ideal_landing = np.hstack(
                (com_lo[:2] + self.T_fl * comd_lo[:2], target_CoM[2]))
            t = np.linspace(0, self.T_fl, self.number_of_blobs)
            com = np.zeros(3)
            for blob in range(self.number_of_blobs):
                com[2] = com_lo[2] + comd_lo[2] * t[blob] + \
                    0.5 * (-9.81) * t[blob] * t[blob]
                com[:2] = com_lo[:2] + comd_lo[:2] * t[blob]
                self.intermediate_flight_com_position.append(com.copy())
            return True

    def plotTrajectoryFlight(self):
        # plot com intermediate positions
        for blob in range(len(self.intermediate_flight_com_position)):
            self.ros_pub.add_marker(self.intermediate_flight_com_position[blob], color=[
                blob * 0. / self.number_of_blobs, blob * 0.0 / self.number_of_blobs, blob * 0.5 / self.number_of_blobs],
                radius=0.05)

    def computeTrajectoryBezier(self, T_th, T_th_total):
        self.number_of_blobs = 30
        t = np.linspace(0, T_th_total, self.number_of_blobs)
        for blob in range(self.number_of_blobs):
            if t[blob] < T_th:
                self.intermediate_com_position.append(
                    self.Bezier3(self.bezier_weights_lin, t[blob], T_th))
            else:
                t_exp = T_th_total - T_th
                t_ = (t[blob] - T_th)/t_exp
                self.intermediate_com_position.append(self.lerp(self.jumpAgent.trunk_x_lo,
                                                                self.jumpAgent.trunk_x_exp, t_)[0])

    def bernstein_pol(self, k, n, x):
        v = (np.math.factorial(n)/(np.math.factorial(k) *
             (np.math.factorial(n-k))))*np.power(x, k)*np.power(1-x, n-k)
        return v

    def Bezier3(self, w, t_ex, t_th):
        t = t_ex/t_th
        if w.ndim == 1:
            return w[0] * self.bernstein_pol(0, 3, t) + \
                w[1] * self.bernstein_pol(1, 3, t) + \
                w[2] * self.bernstein_pol(2, 3, t) + \
                w[3] * self.bernstein_pol(3, 3, t)
        else:
            return w[:, 0]*self.bernstein_pol(0, 3, t) +\
                w[:, 1]*self.bernstein_pol(1, 3, t) +\
                w[:, 2]*self.bernstein_pol(2, 3, t) +\
                w[:, 3]*self.bernstein_pol(3, 3, t)

    def Bezier2(self, w, t_ex, t_th):
        t = t_ex/t_th
        return ((w[:, 1]-w[:, 0]))*(3/t_th)*self.bernstein_pol(0, 2, t) +\
            ((w[:, 2]-w[:, 1]))*(3/t_th)*self.bernstein_pol(1, 2, t) +\
            ((w[:, 3]-w[:, 2]))*(3/t_th)*self.bernstein_pol(2, 2, t)

    def Bezier1(self, w, t_ex, t_th):
        t = t_ex / t_th
        return ((w[:, 2] - 2 * w[:, 2] + w[:, 0])) * (6 / np.power(t_th, 2)) * self.bernstein_pol(0, 1, t) + \
               ((w[:, 3] - 2 * w[:, 2] + w[:, 1])) * \
            (6 / np.power(t_th, 2)) * self.bernstein_pol(1, 1, t)

    def computeJcb(self, feetW, com, stance_legs):
        Jb = np.zeros([3 * self.robot.nee, 6])  # Newton-Euler matrix
        for leg in range(self.robot.nee):
            start_row = 3 * leg
            end_row = 3 * (leg + 1)
            if stance_legs[leg]:
                # ---> linear part
                # identity matrix (I avoid to rewrite zeros)
                Jb[start_row:end_row, :3] = np.identity(3)
                # ---> angular part
                # all in a function
                Jb[start_row:end_row, 3:] = -pin.skew(feetW[leg] - com)
            else:
                Jb[start_row:end_row, 3:] = np.zeros(3)
                Jb[start_row:end_row, :3] = np.zeros(3)
        return Jb

    def setJumpPlatformPosition(self, target, com0):
        # create the message
        set_platform_position = SetModelStateRequest()
        # create model state
        model_state = ModelState()
        model_state.model_name = 'jump_platform'
        model_state.pose.position.x = target[0]
        model_state.pose.position.y = target[1]
        model_state.pose.position.z = target[2]-com0[2]
        model_state.pose.orientation.w = 1.0
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0
        set_platform_position.model_state = model_state
        # send request and get response (in this case none)
        self.set_state(set_platform_position)

    def customStartupProcedure(self):
        print(colored("Custom Startup Procedure", "red"))
        self.q_des = self.qj_0
        self.pid = PidManager(self.joint_names)
        # set joint pdi gains
        self.pid.setPDjoints(conf.robot_params[self.robot_name]['kp'],
                             conf.robot_params[self.robot_name]['kd'],  np.zeros(self.robot.na))
        p.resetRobot(basePoseDes=np.array([0, 0, 0.3,  0., 0., 0.]))
        while self.time <= self.startTrust:
            self.updateKinematics()
            self.tau_ffwd = p.gravityCompensation()
            self.send_command(p.q_des, p.qd_des, p.tau_ffwd)

    def lerp(self, start, end, weight):
        return start + weight * (end - start)


if __name__ == '__main__':
    p = QuadrupedJumpController('go1')
    world_name = 'fast.world'

    try:
        # p.startController(world_name='slow.world')
        p.startController(world_name=world_name,
                          use_ground_truth_pose=True,
                          use_ground_truth_contacts=True,
                          additional_args=['gui:='+str(p.use_gui),
                                           'go0_conf:='+p.go0_conf])
        # initialize data stucture to use them with desired values
        p.des_robot = RobotWrapper.BuildFromURDF(
            os.environ.get('LOCOSIM_DIR') + "/robot_urdf/generated_urdf/" + p.robot_name + ".urdf", root_joint=pinocchio.JointModelFreeFlyer())

        if p.real_robot:
            p.startTrust = 20.
            p.startupProcedure()
            p.updateKinematics()  # neeeded to cal legodom to have an initial estimate of com position
        else:
            p.startTrust = 1.
            p.customStartupProcedure()

        ros.sleep(2.)

        p.target_CoM = np.array([0.6, 0., 0.3])
        action = np.array([-2.7202e-01,  1.9136e+00, -1.8142e+00, -3.0357e+00, -8.4845e-01,
          3.4539e-02, -1.1952e+00, -2.4705e-02,  1.9945e-05,  1.7154e+00,
         -4.9221e-02, -4.8772e+00, -2.7106e+00])

        p.jumpAgent.process_actions(action, p.target_CoM)
        # initial pose
        # linear
        com_lo = p.jumpAgent.trunk_x_lo[0]
        comd_lo = p.jumpAgent.trunk_xd_lo[0]
        com_exp = p.jumpAgent.trunk_x_exp[0]
        comd_exp = p.jumpAgent.trunk_xd_exp[0]
        com_0 = p.basePoseW[:3].copy()
        # angular
        eul_0 = p.basePoseW[3:].copy()
        eul_lo = p.jumpAgent.trunk_o_lo
        euld_lo = p.jumpAgent.trunk_od_lo
        # t_th
        p.T_th = p.jumpAgent.t_th[0].item()
        p.T_exp = p.jumpAgent.t_exp[0].item()
        p.T_th_total = p.jumpAgent.t_th_total[0].item()

        # impulse bezier
        p.a_z = np.array([25, 25, 25, 25])  # LF LH RF RH

        p.lerp_time = 0.1

        if p.DEBUG:
            p.initial_com = np.copy(com_0)
            print(f"Initial Com Position is {p.initial_com}")
            print(f"Initial Joint Position is {p.q}")
            print(f"Initial Joint torques {p.tau_ffwd}")
            p.T_th = 5.

        p.computeHeuristicSolutionBezierLinear(com_0, com_lo, comd_lo, p.T_th)
        # p.computeHeuristicSolutionBezierAngular(eul_0, eul_lo, euld_lo, p.T_th)
        p.computeHeuristicSolutionBezierAngular(
            eul_0, eul_lo, euld_lo, p.T_th_total)

        # this is for visualization
        # p.computeTrajectoryBezier(p.T_th)
        p.computeTrajectoryBezier(p.T_th, p.T_th_total)
        p.stance_legs = [True, True, True, True]
        # print(p.computeJcb(p.W_contacts, com_0))
        # reset integration of feet
        p.W_feetRelPosDes = np.copy(p.W_contacts - com_0)
        p.W_contacts_sampled = np.copy(p.W_contacts)

        if not p.real_robot:
            p.setSimSpeed(dt_sim=0.001, max_update_rate=200, iters=1500)

        while not ros.is_shutdown():
            p.updateKinematics()
            if (p.time > p.startTrust):
                # release base
                if p.firstTime:
                    p.firstTime = False
                    p.trustPhaseFlag = True
                    p.T_fl = None
                    # if (p.computeIdealLanding(com_lo, comd_lo, p.target_CoM)):
                    if (p.computeIdealLanding(com_exp, comd_exp, p.target_CoM)):
                        error = np.linalg.norm(p.ideal_landing - p.target_CoM)
                    else:
                        break
                # compute joint reference
                if (p.trustPhaseFlag):
                    t = p.time - p.startTrust
                    # TODO: compute this with explosive part
                    # p.q_des, p.qd_des, p.tau_ffwd, p.basePoseW_des, p.baseTwistW_des = p.evalBezier(
                    #     t, p.T_th, p.ffwd_impulse, p.a_z)
                    p.q_des, p.qd_des, p.tau_ffwd, p.basePoseW_des, p.baseTwistW_des = p.evalBezier(
                        t, p.T_th, p.T_exp, p.ffwd_impulse, p.a_z)

                    # if p.time >= (p.startTrust + p.T_th):
                    if p.time >= (p.startTrust + p.T_th_total):
                        p.trustPhaseFlag = False
                        # we se this here to have enough retraction (important)
                        p.q_t_th = p.q.copy()
                        p.qd_des = np.zeros(12)
                        p.tau_ffwd = np.zeros(12)
                        print(
                            colored(f"thrust completed! at time {p.time}", "red"))
                        if p.DEBUG:
                            break
                else:
                    p.detectApex()
                    if (p.detectedApexFlag):
                        # set jump position (avoid collision in jumping)
                        elapsed_time_apex = p.time - p.t_apex
                        elapsed_ratio_apex = np.clip(
                            elapsed_time_apex / p.lerp_time, 0, 1)
                        p.q_des = p.lerp(p.q_apex, p.q_0_td, elapsed_ratio_apex).copy()
                        if p.detectTouchDown():
                            p.landing_position = p.u.linPart(p.basePoseW)
                            perc_err = 100. * \
                                np.linalg.norm(
                                    p.target_CoM - p.landing_position) / np.linalg.norm(com_0 - p.target_CoM)
                            print(
                                colored(f"landed at {p.basePoseW} with perc.  error {perc_err}", "green"))
                            break
                    else:
                        elapsed_time = p.time - (p.startTrust + p.T_th_total)
                        elapsed_ratio = np.clip(
                            elapsed_time / p.lerp_time, 0, 1)
                        p.q_des = p.lerp(p.q_t_th, p.q_0_lo,
                                         elapsed_ratio).copy()

            p.plotTrajectoryBezier()
            p.plotTrajectoryFlight()
            # plot target
            p.ros_pub.add_marker(p.target_CoM, color="blue", radius=0.1)

            if p.DEBUG:
                p.ros_pub.add_marker(
                    p.bezier_weights_lin[:, 0], color="red", radius=0.02)
                p.ros_pub.add_marker(
                    p.bezier_weights_lin[:, 1], color="red", radius=0.02)
                p.ros_pub.add_marker(
                    p.bezier_weights_lin[:, 2], color="red", radius=0.02)
                p.ros_pub.add_marker(
                    p.bezier_weights_lin[:, 3], color="red", radius=0.02)

            if (np.linalg.norm(p.ideal_landing) > 0.):
                p.ros_pub.add_marker(
                    p.ideal_landing, color="purple", radius=0.1)
            # com at LIFT OFF given by the N network
            if p.DEBUG:
                p.ros_pub.add_arrow(com_lo, comd_lo, "red")
                p.ros_pub.add_marker(com_lo, color="red", radius=0.1)

            p.visualizeContacts()
            p.logData()

            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
            # log variables
            p.rate.sleep()
            p.sync_check()
            # np.array([self.loop_time]), 3)
            p.time = np.round(p.time + p.dt, 3)

    except (ros.ROSInterruptException, ros.service.ServiceException):
        p.pid.setPDjoints(np.zeros(p.robot.na), np.zeros(
            p.robot.na), np.zeros(p.robot.na))
        ros.signal_shutdown("killed")
        p.deregister_node()


    p.deregister_node()
    if conf.plotting:
        plotJoint('position', time_log=p.time_log, q_log=p.q_log,
                  q_des_log=p.q_des_log, sharex=True, sharey=False, start=0, end=-1)
        # plotJoint('velocity', time_log=p.time_log, qd_log=p.qd_log, qd_des_log=p.qd_des_log, sharex=True, sharey=False,   start=0, end=-1)
        plotJoint('torque', time_log=p.time_log, tau_log=p.tau_log,
                  tau_des_log=p.tau_des_log, sharex=True, sharey=False, start=0, end=-1)
        plotFrame('position', time_log=p.time_log, des_Pose_log=p.basePoseW_des_log, Pose_log=p.basePoseW_log,
                  title='Base', frame='W', sharex=True, sharey=False, start=0, end=-1)
        plotFrame('velocity', time_log=p.time_log, des_Twist_log=p.baseTwistW_des_log,
                  Twist_log=p.baseTwistW_log,  title='Base', frame='W', sharex=True, sharey=False, start=0, end=-1)
        plotContacts('GRFs', time_log=p.time_log, des_Forces_log=p.grForcesW_des_log,
                     Forces_log=p.grForcesW_log, contact_states=p.contact_state_log, frame='W')

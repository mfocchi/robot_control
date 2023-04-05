# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

from __future__ import print_function

import time

import rospy as ros
from utils.math_tools import *
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
import matplotlib.pyplot as plt
from base_controller import BaseController
from base_controllers.utils.common_functions import plotFrame, plotJoint
import pinocchio as pin
import  params as conf
import numpy as np
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.msg import ModelState
from base_controllers.utils.math_tools import cross_mx
robotName = "starbot"

class StarbotController(BaseController):
    
    def __init__(self, robot_name="solo"):
        super().__init__(robot_name=robot_name)
        self.freezeBaseFlag = False
        print("Initialized starbot controller---------------------------------------------------------------")
        # this is consistent with Pinocchio
        self.leg_map = {
            "LF": 0,
            "LH": 1,
            "RF": 2,
            "RH": 3
        }
        self.prewheel_joints = np.array([12,13,14,15])
        self.prismatic_joints = np.array([8, 9, 10,11])
        self.wheel_joints = np.array([16, 17, 18, 19])

        self.wheels_position = np.zeros((4,3))

    def initVars(self):
        super().initVars()
        ## add your variables to initialize here
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']
        self.n_feet = len(conf.robot_params[self.robot_name]['ee_frames'])
        self.NEMatrix = np.zeros([6, 3 * self.n_feet])  # Newton-Euler matrix
        self.grForcesW_des = np.empty(3 * self.n_feet) * np.nan
        self.comPoseW_log = np.empty((6, conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
        self.pipeContact = [False] * 4
    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                ## add your logs here
                self.comPoseW_log[:, self.log_counter] = self.comPoseW
            super().logData()

    def mapToPinocchio(self, vec):
        vec_pin=np.empty((len(self.joint_names)))*np.nan
        joint_idx = 0
        # order of joints in pinocchio is  lf XX, lh XX., rf XX, rh XX whis is different from our joint names
        for pin_fr in self.robot.model.names:
            for fr in self.joint_names:
                if pin_fr == fr:
                    vec_pin[joint_idx] = vec[self.joint_names.index(fr)]
                    joint_idx+=1
        return vec_pin

    def mapFromPinocchio(self, vec_pin):
        pin_joint_names = []
        for f in self.robot.model.names:
            if not ( f == 'universe') and not ( f == 'floating_base_joint'):
                pin_joint_names.append(f)
        vec=np.empty((len(self.joint_names)))*np.nan
        joint_idx = 0
        # order of joints in pinocchio is  lf XX, lh XX., rf XX, rh XX whis is different from our joint names
        for fr in self.joint_names:
            for pin_fr in pin_joint_names:
                if fr == pin_fr:
                    vec[joint_idx] = vec_pin[pin_joint_names.index(pin_fr)]
                    joint_idx+=1
        return vec

    def updateKinematics(self):
        # q is continuously updated
        # to compute in the base frame you should put neutral base
        b_X_w = motionVectorTransform(np.zeros(3), self.b_R_w)
        gen_velocities = np.hstack((b_X_w.dot(self.baseTwistW), self.mapToPinocchio(self.qd)))
        neutral_fb_jointstate = np.hstack((pin.neutral(self.robot.model)[0:7], self.mapToPinocchio(self.q)))
        pin.forwardKinematics(self.robot.model, self.robot.data, neutral_fb_jointstate, gen_velocities)
        pin.computeJointJacobians(self.robot.model, self.robot.data)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        ee_frames = conf.robot_params[self.robot_name]['ee_frames']

        for leg in range(4):
            self.B_contacts[leg] = self.robot.framePlacement(neutral_fb_jointstate,
                                                             self.robot.model.getFrameId(ee_frames[leg]),
                                                             update_kinematics=True).translation

            self.W_contacts[leg] = self.mapBaseToWorld(self.B_contacts[leg].transpose())
            if self.use_ground_truth_contacts:
                self.w_R_lowerleg[leg] = self.b_R_w.transpose().dot(
                    self.robot.data.oMf[self.lowerleg_index[leg]].rotation)

        for leg in range(4):
            leg_joints = range(6 + leg* 5, 6 + leg * 5 + 5)
            self.J[leg] = self.robot.frameJacobian(neutral_fb_jointstate, self.robot.model.getFrameId(ee_frames[leg]),
                                                   pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, leg_joints]
            self.wJ[leg] = self.b_R_w.transpose().dot(self.J[leg])

            self.wheels_position[leg] = self.robot.framePlacement(neutral_fb_jointstate, self.robot.model.getFrameId(ee_frames[leg])).translation

        # Pinocchio Update the joint and frame placements
        gen_velocities = np.hstack((b_X_w.dot(self.baseTwistW), self.mapToPinocchio(self.qd)))
        configuration = np.hstack((self.u.linPart(self.basePoseW), self.quaternion, self.mapToPinocchio(self.q)))
        self.M = self.robot.mass(configuration)
        self.h = pin.nonLinearEffects(self.robot.model, self.robot.data, configuration, gen_velocities)
        self.h_joints = self.h[6:]

        self.comPoseW = np.copy(self.basePoseW)
        self.comPoseW[self.u.sp_crd["LX"]:self.u.sp_crd["LX"] + 3] = self.robot.robotComW(configuration)

        # compute contact forces (TODO fix this torques are a bit strange)
        self.estimateContactForces()

    def estimateContactForces(self):
        # estimate ground reaction forces from tau
        for leg in range(4):
            try:
                # print(self.wJ[leg])
                # print(self.wJ[leg].T)
                # you need to extract the torque of each leg in the convention of Pinocchio cause the Jacobian assumes that
                tau_leg_without_wheel = self.getLegJointTorques(leg, self.h_joints - self.mapToPinocchio(self.tau))[:-1]
                grf = np.linalg.pinv(self.wJ[leg][:,:-1].T).dot(tau_leg_without_wheel)
            except np.linalg.linalg.LinAlgError as error:
                grf = np.zeros(3)
            self.setLegContactForce(leg, grf, self.grForcesW)
            # TODO update this in the tunnel with the wheelTobase vector
            if self.contact_normal[leg].dot(grf) >= conf.robot_params[self.robot_name]['force_th']:
                self.contact_state[leg] = True
            else:
                self.contact_state[leg] = False
            if self.use_ground_truth_contacts:
                grfLocal_gt = self.getLegContactForce(leg,  self.grForcesLocal_gt)
                # rotate the vector because it is in the lowerleg frame
                grf_gt = self.w_R_lowerleg[leg] @ grfLocal_gt
                self.setLegContactForce(leg, grf_gt, self.grForcesW_gt)

    def setLegJointTorques(self, legid,  input, torques):
        if isinstance(legid, str):
            torques[self.leg_map[legid] * 5:self.leg_map[legid] * 5 + 5] = input
        elif isinstance(legid, int):
            torques[legid * 5:legid * 5 + 5] = input

    def getLegJointTorques(self, legid,  torques):
        if isinstance(legid, str):
            return torques[self.leg_map[legid]*5:self.leg_map[legid]*5+5]
        elif isinstance(legid, int):
            return torques[legid * 5:legid * 5 + 5]

    def setLegContactForce(self, legid,  input, forces):
        if isinstance(legid, str):
            forces[self.leg_map[legid]*3:self.leg_map[legid]*3+3] = input
        elif isinstance(legid, int):
            forces[legid*3:legid*3+3] = input

    def getLegContactForce(self, legid,  forces):
        if isinstance(legid, str):
            return forces[self.leg_map[legid]*3:self.leg_map[legid]*3+3]
        elif isinstance(legid, int):
            return forces[legid * 3:legid * 3 + 3]

    # comoute gravity torques assuming gravity is applied to com
    def computeGravityTorques(self, ee_frames):
        util = Utils()

        # compute gravity wrench
        Wg = np.zeros(6)
        mg = self.robot.robotMass * self.robot.model.gravity.vector[:3]
        Wg[util.sp_crd["LX"]:util.sp_crd["LX"] + 3] = -mg

        # clean the matrix
        self.NEMatrix[:, :] = 0.
        self.grForcesW_des = np.zeros(12)
        # wrench = NEMatrix @ grfs
        for leg in range(len(ee_frames)):
            if self.contact_state[leg]:
                start_col = 3 * leg
                end_col = 3 * (leg + 1)
                # ---> linear part
                # identity matrix (I avoid to rewrite zeros)
                self.NEMatrix[self.u.sp_crd["LX"], start_col] = 1.
                self.NEMatrix[self.u.sp_crd["LY"], start_col + 1] = 1.
                self.NEMatrix[self.u.sp_crd["LZ"], start_col + 2] = 1.
                # ---> angular part
                # all in a function
                self.NEMatrix[self.u.sp_crd["AX"]:self.u.sp_crd["AX"] + 3, start_col:end_col] = \
                    pin.skew(self.W_contacts[leg] - self.u.linPart(self.comPoseW))

        # Map the desired wrench to grf (note the vector is 12D and is ee_frames convention)
        self.grForcesW_des = np.linalg.pinv(self.NEMatrix, 1e-06) @ Wg

        # we want to map grfs into torques only considering the joint before the wheel, so I select the first 4 columns of the Jacobian!
        pin_gravity_torques = np.empty(20)
        for leg in range(len(ee_frames)):
            # note tau leg has 5 elements
            tau_leg = -self.wJ[leg].T @ self.getLegContactForce(leg, self.grForcesW_des)
            # we remove the torque on the wheel
            tau_leg [4] = 0.0
            self.setLegJointTorques(leg, tau_leg, pin_gravity_torques)
        return self.mapFromPinocchio(pin_gravity_torques)

    def freezeBase(self, flag, basePoseW=None, baseTwistW=None):

        if flag:
            self.setGravity(0)

            # create the message
            req_reset_world = SetModelStateRequest()
            # create model state
            model_state = ModelState()
            model_state.model_name = self.robot_name

            model_state.pose.position.x = self.u.linPart(basePoseW)[0]
            model_state.pose.position.y = self.u.linPart(basePoseW)[1]
            model_state.pose.position.z = self.u.linPart(basePoseW)[2]
            quaternion = pin.Quaternion(pin.rpy.rpyToMatrix(self.u.angPart(basePoseW)))
            model_state.pose.orientation.x = quaternion.x
            model_state.pose.orientation.y = quaternion.y
            model_state.pose.orientation.z = quaternion.z
            model_state.pose.orientation.w = quaternion.w
            req_reset_world.model_state = model_state
            # send request and get response (in this case none)
            self.reset_world(req_reset_world)

        else:
            self.setGravity(-9.81)


# return skew symmetric matrix of vector x
def skew(x):
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])

# function to generate trajectories given start position, final position, motion time and actual time
def quinitic_trajectory(q0, qf, tf, time):
    a0 = q0
    a1 = 0
    a2 = 0
    a3 = (20 * qf - 20 * q0) / (2 * (tf ** 3))
    a4 = (30 * q0 - 30 * qf) / (2 * (tf ** 4))
    a5 = (12 * qf - 12 * q0) / (2 * (tf ** 5))

    pos = a0 + a1*time + a2*(time**2) + a3*(time**3) + a4*(time**4) + a5*(time**5)
    #vel = a1 + 2*a2*time + 3*a3*(time**2) + 4*a4*(time**3) + 5*a5*(time**4)

    return pos

# compute distance wheels has to travel from init state to car state
def wheel_arc_carState(leg_length):
    return leg_length - leg_length/(np.sqrt(2))

# generate new q desired
def generate_q(joints):
    return np.array([joints["shoulder"][0],
                     joints["shoulder"][1],
                     joints["shoulder"][2],
                     joints["shoulder"][3],
                     joints["upper_leg"][0],
                     joints["upper_leg"][1],
                     joints["upper_leg"][2],
                     joints["upper_leg"][3],
                     joints["lower_leg"][0],
                     joints["lower_leg"][1],
                     joints["lower_leg"][2],
                     joints["lower_leg"][3],
                     joints["pre_wheel"][0],
                     joints["pre_wheel"][1],
                     joints["pre_wheel"][2],
                     joints["pre_wheel"][3],
                     joints["wheel"][0],
                     joints["wheel"][1],
                     joints["wheel"][2],
                     joints["wheel"][3]])

# compute the perimeter of the circle in which the wheels are placed
def get_external_perimeter(p):
    ee_frames = conf.robot_params[p.robot_name]['ee_frames']
    radius = np.zeros(4)

    for leg in range(4):
        id = p.robot.model.getFrameId(ee_frames[leg])
        id = p.robot.model.frames[id].parent

        radius[leg] = abs(math.dist([np.array(p.robot.data.oMi[1])[0][3],np.array(p.robot.data.oMi[1])[1][3]],
                                    [np.array(p.robot.data.oMi[id])[0][3],np.array(p.robot.data.oMi[id])[1][3]])) + \
                      np.array(p.robot.data.oMi[id])[2][3]

    radius = np.mean(radius)
    return radius*2*np.pi

# convert distance expressed in meters into radians for the wheels
def metersToradians(dist):
    # with 0.07 radius of the wheels
    return dist/(0.07*2*np.pi)*360*np.pi/180


def distance_contact_force(leg):
    wheel_force = p.getLegContactForce(leg, p.grForcesW / (6 * p.robot.robot_mass))
    # torque of primsatic joint
    tau_leg = p.getLegJointTorques(leg, p.h_joints - p.mapToPinocchio(p.tau))[2]

    # define which leg to consider
    ee_frames = conf.robot_params[p.robot_name]['ee_frames']
    id = p.robot.model.getFrameId(ee_frames[leg])
    id = p.robot.model.frames[id].parent

    if (id == "lf_wheel" or id == "rh_wheel"):
        # asse y
        tot_force = wheel_force[0] + wheel_force[2]
    else:
        # asse x
        tot_force = wheel_force[1] + wheel_force[2]

    print("CONTROLLO", tau_leg / tot_force)

    distance = (tau_leg / tot_force) - 0.2

    return distance

# finds useful data of the robot in init position
def initialization_state(p):
    ee_frames = conf.robot_params[p.robot_name]['ee_frames']
    shoulder_frames = ["lf_bs_joint","lh_bs_joint","rf_bs_joint","rh_bs_joint"]
    shoulder_pos = np.zeros(4)
    leg_length = np.zeros(4)
    for leg in range(4):
        id = p.robot.model.getFrameId(ee_frames[leg])
        id = p.robot.model.frames[id].parent

        id_S = p.robot.model.getFrameId(shoulder_frames[leg])
        id_S = p.robot.model.frames[id_S].parent
        if (ee_frames[leg] == 'rf_wheel' or ee_frames[leg] == 'lh_wheel' ):
            shoulder_pos[leg] = abs(np.array(p.robot.data.oMi[id_S])[0][3])
            leg_length[leg] = abs(np.array(p.robot.data.oMi[id])[0][3]) + abs(np.array(p.robot.data.oMf[id])[0][3]) - shoulder_pos[leg]
        else:
            shoulder_pos[leg] = abs(np.array(p.robot.data.oMi[id_S])[1][3])
            leg_length[leg] = abs(np.array(p.robot.data.oMi[id])[1][3]) + abs(np.array(p.robot.data.oMf[id])[0][3]) - shoulder_pos[leg]

    leg_length = np.mean(leg_length)
    shoulder_pos = np.mean(shoulder_pos)
    #print(leg_length)

    return np.copy(p.q_des_q0), leg_length, shoulder_pos

# state change, from init configuration to car one
def initTocar_state(p, joints, prev_state, initTocar_motion_time, act_time, shoulder_motion, wheel_arc_dist, wheel_motion, contact_time):
    # move shoulder to lower the 4 legs and lift the central body
    if act_time < initTocar_motion_time/2:
        joints["shoulder"][0] = quinitic_trajectory(0, -shoulder_motion, initTocar_motion_time / 2, act_time)
        joints["shoulder"][1] = quinitic_trajectory(0, shoulder_motion, initTocar_motion_time / 2, act_time)
        joints["shoulder"][2] = quinitic_trajectory(0, shoulder_motion, initTocar_motion_time / 2, act_time)
        joints["shoulder"][3] = quinitic_trajectory(0, -shoulder_motion, initTocar_motion_time / 2, act_time)

        # rotate wheels to finish lifting motion
        if p.contact_state[0] == True:
            if contact_time == 0:
                contact_time = act_time

            joints["wheel"][0] = prev_state["wheel"][0] + quinitic_trajectory(0, wheel_arc_dist, initTocar_motion_time / 2, act_time)
            joints["wheel"][1] = prev_state["wheel"][1] - quinitic_trajectory(0, wheel_arc_dist, initTocar_motion_time / 2, act_time)
            joints["wheel"][2] = prev_state["wheel"][2] - quinitic_trajectory(0, wheel_arc_dist, initTocar_motion_time / 2, act_time)
            joints["wheel"][3] = prev_state["wheel"][3] + quinitic_trajectory(0, wheel_arc_dist, initTocar_motion_time / 2, act_time)

        q_des = generate_q(joints)

    # change wheels orientation to enable car configuration movement
    else:
        joints["pre_wheel"][0] = quinitic_trajectory(0, 0., initTocar_motion_time / 2, act_time - (initTocar_motion_time / 2))
        joints["pre_wheel"][1] = quinitic_trajectory(0, -wheel_motion, initTocar_motion_time / 2, act_time - (initTocar_motion_time / 2))
        joints["pre_wheel"][2] = quinitic_trajectory(0, +wheel_motion, initTocar_motion_time / 2, act_time - (initTocar_motion_time / 2))
        joints["pre_wheel"][3] = quinitic_trajectory(0, 0., initTocar_motion_time / 2, act_time - (initTocar_motion_time / 2))
        q_des = generate_q(joints)

    return joints, q_des, contact_time

# approaching tunnel state
def approacing_state(p, joints, prev_state, dist_from_tunnel, approaching_tunnel_time, act_time):
    joints["wheel"][0] = prev_state["wheel"][0] - quinitic_trajectory(0, dist_from_tunnel, approaching_tunnel_time, act_time)
    joints["wheel"][1] = prev_state["wheel"][1] + quinitic_trajectory(0, dist_from_tunnel, approaching_tunnel_time, act_time)
    joints["wheel"][2] = prev_state["wheel"][2] + quinitic_trajectory(0, dist_from_tunnel, approaching_tunnel_time, act_time)
    joints["wheel"][3] = prev_state["wheel"][3] + quinitic_trajectory(0, dist_from_tunnel, approaching_tunnel_time, act_time)
    q_des = generate_q(joints)

    return joints, q_des

# state change, from car configuration to star one (inside tunnel)
def carTostar_state(p, joints, prev_state, carTostar_motion_time, act_time, wheel_motion, robot_rotation, straight_leg_angle, wheel_arc_dist):
    # change wheels orientation to enable the robot to rotate its body on his Z axis
    if act_time <= carTostar_motion_time/8:
        joints["pre_wheel"][0] = quinitic_trajectory(0, -wheel_motion, carTostar_motion_time / 8, act_time)
        joints["pre_wheel"][1] = joints["pre_wheel"][1]
        joints["pre_wheel"][2] = joints["pre_wheel"][2]
        joints["pre_wheel"][3] = quinitic_trajectory(0, +wheel_motion, carTostar_motion_time / 8, act_time)
        q_des = generate_q(joints)

        if act_time == carTostar_motion_time/8:
            prev_state["shoulder"] = np.copy(joints["shoulder"])
            prev_state["upper_leg"] = np.copy(joints["upper_leg"])
            prev_state["lower_leg"] = np.copy(joints["lower_leg"])
            prev_state["pre_wheel"] = np.copy(joints["pre_wheel"])
            prev_state["wheel"] = np.copy(joints["wheel"])

    # rotate wheels to change robot orientation
    elif act_time > carTostar_motion_time/8 and act_time <= (carTostar_motion_time/8)*2:
        joints["wheel"][0] = prev_state["wheel"][0] - quinitic_trajectory(0, robot_rotation, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8))
        joints["wheel"][1] = prev_state["wheel"][1] + quinitic_trajectory(0, robot_rotation, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8))
        joints["wheel"][2] = prev_state["wheel"][2] - quinitic_trajectory(0, robot_rotation, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8))
        joints["wheel"][3] = prev_state["wheel"][3] + quinitic_trajectory(0, robot_rotation, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8))
        q_des = generate_q(joints)

        if act_time == carTostar_motion_time/8*2:
            prev_state["shoulder"] = np.copy(joints["shoulder"])
            prev_state["upper_leg"] = np.copy(joints["upper_leg"])
            prev_state["lower_leg"] = np.copy(joints["lower_leg"])
            prev_state["pre_wheel"] = np.copy(joints["pre_wheel"])
            prev_state["wheel"] = np.copy(joints["wheel"])

    # close legs to enable robot raise inside the tunnel
    elif act_time > (carTostar_motion_time/8)*2 and act_time <= (carTostar_motion_time/8)*3:
        joints["upper_leg"][0] = quinitic_trajectory(0, 0.6, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8 * 2))
        joints["upper_leg"][1] = quinitic_trajectory(0, -0.6, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8 * 2))
        joints["upper_leg"][2] = quinitic_trajectory(0, -0.6, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8 * 2))
        joints["upper_leg"][3] = quinitic_trajectory(0, 0.6, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8 * 2))
        q_des = generate_q(joints)

        if act_time == carTostar_motion_time / 8*3:
            prev_state["shoulder"] = np.copy(joints["shoulder"])
            prev_state["upper_leg"] = np.copy(joints["upper_leg"])
            prev_state["lower_leg"] = np.copy(joints["lower_leg"])
            prev_state["pre_wheel"] = np.copy(joints["pre_wheel"])
            prev_state["wheel"] = np.copy(joints["wheel"])

    # change wheels orientation to robot raise inside the tunnel
    elif act_time > (carTostar_motion_time / 8) * 3 and act_time <= (carTostar_motion_time / 8) * 4:
        joints["pre_wheel"][0] = quinitic_trajectory(prev_state["pre_wheel"][0], 0, carTostar_motion_time / 8,
                                                     act_time - (carTostar_motion_time / 8 * 3))
        joints["pre_wheel"][1] = quinitic_trajectory(prev_state["pre_wheel"][1], 0, carTostar_motion_time / 8,
                                                     act_time - (carTostar_motion_time / 8 * 3))
        joints["pre_wheel"][2] = quinitic_trajectory(prev_state["pre_wheel"][2], 0, carTostar_motion_time / 8,
                                                     act_time - (carTostar_motion_time / 8 * 3))
        joints["pre_wheel"][3] = quinitic_trajectory(prev_state["pre_wheel"][3], 0, carTostar_motion_time / 8,
                                                     act_time - (carTostar_motion_time / 8 * 3))
        q_des = generate_q(joints)

        if act_time == carTostar_motion_time / 8 * 4:
            prev_state["shoulder"] = np.copy(joints["shoulder"])
            prev_state["upper_leg"] = np.copy(joints["upper_leg"])
            prev_state["lower_leg"] = np.copy(joints["lower_leg"])
            prev_state["pre_wheel"] = np.copy(joints["pre_wheel"])
            prev_state["wheel"] = np.copy(joints["wheel"])

    # robot raise inside tunnel, wheels rotation, shoulder joint straightening and leg extension
    elif act_time > (carTostar_motion_time / 8) * 4 and act_time <= (carTostar_motion_time / 8) * 5:
        joints["shoulder"][0] = prev_state["shoulder"][0] + quinitic_trajectory(0, 0.785, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8*4))
        joints["shoulder"][1] = prev_state["shoulder"][1] - quinitic_trajectory(0, 0.785, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8*4))
        joints["shoulder"][2] = prev_state["shoulder"][2] - quinitic_trajectory(0, 0.785, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8*4))
        joints["shoulder"][3] = prev_state["shoulder"][3] + quinitic_trajectory(0, 0.785, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8*4))

        joints["lower_leg"][0] = prev_state["lower_leg"][0] + quinitic_trajectory(0, 0.07, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8 * 4))
        joints["lower_leg"][1] = prev_state["lower_leg"][1] + quinitic_trajectory(0, 0.07, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8 * 4))
        joints["lower_leg"][2] = prev_state["lower_leg"][2] + quinitic_trajectory(0, 0.07, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8 * 4))
        joints["lower_leg"][3] = prev_state["lower_leg"][3] + quinitic_trajectory(0, 0.07, carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8 * 4))

        joints["wheel"][0] = prev_state["wheel"][0] - quinitic_trajectory(0, metersToradians((1.236*2*np.pi) * 116 / 360),
                                                                          carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8*4))
        joints["wheel"][1] = prev_state["wheel"][1] - quinitic_trajectory(0, metersToradians((1.236*2*np.pi) * 36/360),
                                                                          carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8*4))
        joints["wheel"][2] = prev_state["wheel"][2] + quinitic_trajectory(0, metersToradians((1.236*2*np.pi) * 116/360),
                                                                          carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8*4))
        joints["wheel"][3] = prev_state["wheel"][3] + quinitic_trajectory(0, metersToradians((1.236*2*np.pi) * 36/360),
                                                                          carTostar_motion_time / 8, act_time - (carTostar_motion_time / 8*4))
        q_des = generate_q(joints)

        if act_time == carTostar_motion_time / 8*5:
            prev_state["shoulder"] = np.copy(joints["shoulder"])
            prev_state["upper_leg"] = np.copy(joints["upper_leg"])
            prev_state["lower_leg"] = np.copy(joints["lower_leg"])
            prev_state["pre_wheel"] = np.copy(joints["pre_wheel"])
            prev_state["wheel"] = np.copy(joints["wheel"])

    # elif act_time > (carTostar_motion_time / 6) * 4 and act_time <= (carTostar_motion_time / 6) * 5:
    #     joints["pre_wheel"][0] = quinitic_trajectory(0, -0.9, carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6 * 4))
    #     joints["pre_wheel"][1] = quinitic_trajectory(0, +0.9, carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6 * 4))
    #     joints["pre_wheel"][2] = quinitic_trajectory(0, +0.9, carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6 * 4))
    #     joints["pre_wheel"][3] = quinitic_trajectory(0, -0.9, carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6 * 4))
    #     q_des = generate_q(joints)
    #
    #     if act_time == carTostar_motion_time / 6 * 5:
    #         prev_state["shoulder"] = np.copy(joints["shoulder"])
    #         prev_state["upper_leg"] = np.copy(joints["upper_leg"])
    #         prev_state["lower_leg"] = np.copy(joints["lower_leg"])
    #         prev_state["pre_wheel"] = np.copy(joints["pre_wheel"])
    #         prev_state["wheel"] = np.copy(joints["wheel"])




    # elif act_time > (carTostar_motion_time / 6) * 4 and act_time <= (carTostar_motion_time / 6) * 5:
    #     joints["shoulder"][0] = prev_state["shoulder"][0] + quinitic_trajectory(0, 0.785, carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6*4))
    #     joints["shoulder"][1] = prev_state["shoulder"][1] - quinitic_trajectory(0, 0.785, carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6*4))
    #     joints["shoulder"][2] = prev_state["shoulder"][2] - quinitic_trajectory(0, 0.785, carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6*4))
    #     joints["shoulder"][3] = prev_state["shoulder"][3] + quinitic_trajectory(0, 0.785, carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6*4))
    #
    #     joints["lower_leg"][0] = prev_state["lower_leg"][0] + quinitic_trajectory(0, distance_contact_force(0), carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6 * 4))
    #     joints["lower_leg"][1] = prev_state["lower_leg"][1] + quinitic_trajectory(0, distance_contact_force(1), carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6 * 4))
    #     joints["lower_leg"][2] = prev_state["lower_leg"][2] + quinitic_trajectory(0, distance_contact_force(2), carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6 * 4))
    #     joints["lower_leg"][3] = prev_state["lower_leg"][3] + quinitic_trajectory(0, distance_contact_force(3), carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6 * 4))
    #
    #     joints["wheel"][0] = prev_state["wheel"][0] - quinitic_trajectory(0, metersToradians((1.236*2*np.pi) * 45 / 360),
    #                                                                       carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6*4))
    #     joints["wheel"][1] = prev_state["wheel"][1] + quinitic_trajectory(0, metersToradians((1.236*2*np.pi) * 45/360),
    #                                                                       carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6*4))
    #     joints["wheel"][2] = prev_state["wheel"][2] + quinitic_trajectory(0, metersToradians((1.236*2*np.pi) * 45/360),
    #                                                                       carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6*4))
    #     joints["wheel"][3] = prev_state["wheel"][3] - quinitic_trajectory(0, metersToradians((1.236*2*np.pi) * 45/360),
    #                                                                       carTostar_motion_time / 6, act_time - (carTostar_motion_time / 6*4))
    #     q_des = generate_q(joints)
    #
    #     if act_time == carTostar_motion_time / 6*5:
    #         prev_state["shoulder"] = np.copy(joints["shoulder"])
    #         prev_state["upper_leg"] = np.copy(joints["upper_leg"])
    #         prev_state["lower_leg"] = np.copy(joints["lower_leg"])
    #         prev_state["pre_wheel"] = np.copy(joints["pre_wheel"])
    #         prev_state["wheel"] = np.copy(joints["wheel"])

    else:
        q_des = p.q_des

    return joints, q_des

# find euler angles to align to pipe according to ros convention correspondent to perform a rotation about X followed by a one about Z
def alignToPipe(roll, yaw):
    c_roll = np.cos(roll)
    s_roll = np.sin(roll)

    c_yaw = np.cos(yaw)
    s_yaw = np.sin(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, c_roll, -s_roll],
                   [0, s_roll, c_roll]]);



    Rz = np.array([[c_yaw, -s_yaw, 0],
                   [s_yaw, c_yaw, 0],
                   [0, 0, 1]]);

    R = Rx.dot(Rz)
    math_utils = Math()
    return math_utils.rot2eul(R)

#def climbing_state():

def talker(p):
    p.start()
    p.startSimulator(world_name = 'starbot_tunnel.world')
    #p.startSimulator()
    p.loadModelAndPublishers()
    p.initVars()
    p.initSubscribers()
    p.startupProcedure()
    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    p.q_des = np.copy(p.q_des_q0)
    #p.setGravity(0.)

    # joint values
    joints = {
        "shoulder" : np.zeros(4),
        "upper_leg" : np.zeros(4),
        "lower_leg" : np.zeros(4),
        "pre_wheel" : np.zeros(4),
        "wheel": np.zeros(4)
    }

    prev_state = {
        "shoulder": np.zeros(4),
        "upper_leg": np.zeros(4),
        "lower_leg": np.zeros(4),
        "pre_wheel": np.zeros(4),
        "wheel": np.zeros(4)
    }

    # simulation parameter
    start_time_simulation = p.time
    start_time_motion=1
    initTocar_motion_time = 4
    approaching_tunnel_time = 5
    carTostar_motion_time = 12
    state_flag=0
    leg_length=0
    dist_from_tunnel = 60

    ros.sleep(2.)
    print("State 0: init configuration")
    p.freezeBase(True, basePoseW=np.hstack((np.array([0, 4., 0.802]), alignToPipe(1.833, 0.7854))))
    w_R_d = p.math_utils.eul2Rot(alignToPipe(1.833, 0.7854))

    delta_sing = np.array([1, -1, -1, 1]) # -1 it means that a positive rotation of the wheel contributes negatively to the base forward speed (along -Z axis)
    S_a = np.array([[0, 0, 0], [0, 1, 0], [0, 0, 1]])
    n_p = p.math_utils.eul2Rot(np.array([0.2618, 0, 0])).dot(np.array([0, 1, 0]))
    print(n_p)
    K_p = 100
    K_d = 25
    base_velocity = 1
    J_w = np.array([[delta_sing[0]/0.07, 0, 0, 0],
                              [0, delta_sing[1]/0.07, 0, 0],
                              [0, 0, delta_sing[2]/0.07, 0],
                              [0, 0, 0, delta_sing[3]/0.07]])

    while not ros.is_shutdown():
        act_time = p.time-start_time_simulation

        # update the kinematics
        p.updateKinematics()
        #p.tau_ffwd = 0.*np.array([0,0,0,0,   0,0,0,0 , 0 , 0, 0,0  ,0,0,0,0, 1.,1.,1.,1.])#(p.robot.na)
        # if p.time >3 and state_flag < 3:
        #     p.tau_ffwd = p.computeGravityTorques(conf.robot_params[p.robot_name]['ee_frames'])


        # approach
        if (state_flag == 0) and p.time > 1.:
            state_flag = 1

            for leg in range(4):
                p.contact_normal[leg] = (p.u.linPart(p.basePoseW) - p.W_contacts[leg])/np.linalg.norm(p.u.linPart(p.basePoseW) - p.W_contacts[leg])

        if state_flag == 1:
            #approach kinematically
            p.q_des[p.prismatic_joints] += 0.0001

            # check contacts
            for leg in range(4):
                if p.contact_normal[leg].dot(p.getLegContactForce(leg, p.grForcesW)) >= 200:
                   p.pipeContact[leg] = True
                else:
                   p.pipeContact[leg] = False
            if all(p.pipeContact):
                state_flag = 2
                print("starting")
                # approach with force (controller)
                #TODO fix this
                # p.tau_ffwd = p.computeGravityTorques(['lh_wheel', 'lh_wheel'])
                # p.tau_ffwd[p.prismatic_joints] += 100
                p.tau_ffwd[p.prismatic_joints] += 100
                p.pid.setPDjoint(p.prismatic_joints, 0, 0, 0)

                # this restores gravity (only when everything works)
                p.freezeBase(False)
        if state_flag == 2:
            # go forward + orientation control + pre_wheel controller
            #p.q_des[p.wheel_joints] += delta_sing*0.01
            # base orientation
            w_R_a = p.math_utils.eul2Rot(p.euler)

            # compute rotation matrix from actual orientation of the base to the desired one
            a_R_d = w_R_a.T.dot(w_R_d)
            # compute the angle-axis representation of the associated orientation error
            arg = (a_R_d[0,0]+ a_R_d[1,1]+ a_R_d[2,2]-1)/2
            delta_theta = np.arccos(arg)
            # compute the axis (deal with singularity)
            # if delta_theta == 0.0:
            #     e_error_o = np.zeros(3)
            # else:
            r_hat = 1/(2*np.sin(delta_theta))*np.array([a_R_d[2,1]-a_R_d[1,2], a_R_d[0,2]-a_R_d[2,0], a_R_d[1,0]-a_R_d[0,1]])
            # compute the orientation error
            e_error_o = delta_theta * r_hat
            # we need to map it in the error back into world frame
            w_error_o = w_R_a.dot(S_a.dot(e_error_o))

            #computation of J_b matrix
            J_b = np.array([(-n_p.T.dot(skew(p.mapBaseToWorld(p.wheels_position[0])))),
                            (-n_p.T.dot(skew(p.mapBaseToWorld(p.wheels_position[1])))),
                            (-n_p.T.dot(skew(p.mapBaseToWorld(p.wheels_position[2])))),
                            (-n_p.T.dot(skew(p.mapBaseToWorld(p.wheels_position[3]))))])

            omega_fbk  = K_p*w_error_o - K_d*p.u.angPart(p.baseTwistW)
            W_w = J_w.dot(np.array([base_velocity, base_velocity, base_velocity, base_velocity])) + J_w.dot(J_b.dot(omega_fbk))

            #print(W_w)

            # Forward Euler integration
            p.q_des[p.wheel_joints] += W_w*conf.robot_params[p.robot_name]['dt']
            #print(p.q_des[p.wheel_joints])




        # initialization phase
        # if state_flag == 0:
        #     if act_time <= start_time_motion:
        #         ret = initialization_state(p)
        #         p.q_des = ret[0]
        #         leg_length = ret[1]
        #         shoulder_pos = ret[2]
        #     else:
        #         print("Start robot lifting, from init configuration to car one")
        #         state_flag = 1
        #         start_time_simulation = p.time
        #         contact_time = 0
        #
        #         prev_state["shoulder"] = np.copy(joints["shoulder"])
        #         prev_state["upper_leg"] = np.copy(joints["upper_leg"])
        #         prev_state["lower_leg"] = np.copy(joints["lower_leg"])
        #         prev_state["pre_wheel"] = np.copy(joints["pre_wheel"])
        #         prev_state["wheel"] = np.copy(joints["wheel"])
        #
        # # movement from init configuration to car one
        # elif state_flag == 1:
        #     if act_time <= initTocar_motion_time:
        #         ret = initTocar_state(p, joints, prev_state, initTocar_motion_time, act_time, 0.785, metersToradians(wheel_arc_carState(leg_length)),
        #                               1.57, contact_time)
        #         p.q_des = ret[1]
        #         joints = ret[0]
        #         contact_time = ret[2]
        #     else:
        #         print("State 1: car configuration")
        #         print("Start tunnel approaching")
        #         state_flag = 2
        #         start_time_simulation = p.time
        #         prev_state["shoulder"] = np.copy(joints["shoulder"])
        #         prev_state["upper_leg"] = np.copy(joints["upper_leg"])
        #         prev_state["lower_leg"] = np.copy(joints["lower_leg"])
        #         prev_state["pre_wheel"] = np.copy(joints["pre_wheel"])
        #         prev_state["wheel"] = np.copy(joints["wheel"])
        #
        #     # id = p.robot.model.getFrameId('lf_wheel')
        #     # id = p.robot.model.frames[id].parent
        #     # # leg 0 touch ground
        #     # if(p.contact_state[0] == True):
        #     #     print("check")
        #     # print(p.robot.data.oMi[id])
        #     # print(np.array(p.robot.data.oMi[id])[1][3])
        #
        # # tunnel approaching
        # elif state_flag == 2:
        #     if act_time <= approaching_tunnel_time:
        #         ret = approacing_state(p, joints, prev_state, dist_from_tunnel, approaching_tunnel_time, act_time)
        #         p.q_des = ret[1]
        #         joints = ret[0]
        #     else:
        #         print("State 2: car configuration inside tunnel")
        #         print("Start configuration change from car to star")
        #         state_flag = 3
        #         start_time_simulation = p.time
        #         robot_rotation = metersToradians(get_external_perimeter(p)/8)
        #         prev_state["shoulder"] = np.copy(joints["shoulder"])
        #         prev_state["upper_leg"] = np.copy(joints["upper_leg"])
        #         prev_state["lower_leg"] = np.copy(joints["lower_leg"])
        #         prev_state["pre_wheel"] = np.copy(joints["pre_wheel"])
        #         prev_state["wheel"] = np.copy(joints["wheel"])
        #         print(robot_rotation)
        #
        # # configuration change from car to star
        # elif state_flag == 3:
        #     if act_time <= carTostar_motion_time:
        #         ret = carTostar_state(p, joints, prev_state, carTostar_motion_time, act_time, wheel_motion=1.57, robot_rotation = robot_rotation,
        #                               straight_leg_angle = 0.5236, wheel_arc_dist=metersToradians(wheel_arc_carState(leg_length)))
        #         p.q_des = ret[1]
        #         joints = ret[0]
        #     else:
        #         print("State 3: star configuration")
        #         print("Start tunnel climbing")
        #         state_flag = 4
        #         start_time_simulation = p.time
        #         prev_state["shoulder"] = np.copy(joints["shoulder"])
        #         prev_state["upper_leg"] = np.copy(joints["upper_leg"])
        #         prev_state["lower_leg"] = np.copy(joints["lower_leg"])
        #         prev_state["pre_wheel"] = np.copy(joints["pre_wheel"])
        #         prev_state["wheel"] = np.copy(joints["wheel"])
        #
        # elif state_flag == 4:
        #     p.q_des = p.q_des

        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

        # log variables
        p.logData()

        # plot actual (green) and desired (blue) contact forces
        for leg in range(4):
            p.ros_pub.add_arrow(p.W_contacts[leg], p.contact_state[leg] * p.getLegContactForce(leg, p.grForcesW / (6 * p.robot.robot_mass)),
                                "green")
            p.ros_pub.add_arrow(p.W_contacts[leg], p.contact_state[leg] * p.getLegContactForce(leg, p.grForcesW_des / (6 * p.robot.robot_mass)),
                                "blue")
            if p.contact_state[leg]:
                p.ros_pub.add_marker(p.W_contacts[leg], radius=0.1, color="green")
            else:
                p.ros_pub.add_marker(p.W_contacts[leg], radius=0.1, color="red")
            p.ros_pub.add_marker(p.pipeContact[leg]*p.W_contacts[leg], color="green", radius=0.1)
            if (p.use_ground_truth_contacts):
                p.ros_pub.add_arrow(p.W_contacts[leg], p.getLegContactForce(leg, p.grForcesW_gt / (6 * p.robot.robot_mass)),
                                    "red")
        p.ros_pub.publishVisual()

        # wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999

if __name__ == '__main__':

    p = StarbotController(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        if conf.plotting:
            plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
                      p.tau_ffwd_log, joint_names=p.joint_names)
            plotFrame('position', time_log=p.time_log, des_Pose_log=p.basePoseW_log, Pose_log=p.comPoseW_log,
                      title='base lin/ang position', frame='W', sharex=True, sharey=False, start=0, end=-1)


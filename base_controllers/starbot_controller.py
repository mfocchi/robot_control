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
from base_controllers.utils.common_functions import plotCoM, plotJoint
import pinocchio as pin
import  params as conf
import numpy as np
robotName = "starbot"

class StarbotController(BaseController):
    
    def __init__(self, robot_name="solo"):
        super().__init__(robot_name=robot_name)
        self.freezeBaseFlag = False
        print("Initialized starbot controller---------------------------------------------------------------")
        self.leg_map = {
            "LF": 0,
            "LH": 1,
            "RF": 2,
            "RH": 3
        }
    def initVars(self):
        super().initVars()
        ## add your variables to initialize here
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']

    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                ## add your logs here
                pass
            super().logData()

    def mapToPinocchio(self, q):
        q_pin=np.empty((len(self.joint_names)))*np.nan
        joint_idx = 0
        for pin_fr in self.robot.model.names:
            for fr in self.joint_names:
                if pin_fr == fr:
                    q_pin[joint_idx] = q[self.joint_names.index(fr)]
                    joint_idx+=1
        return q_pin


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
            leg_joints = range(6 + self.u.mapIndexToRos(leg) * 5, 6 + self.u.mapIndexToRos(leg) * 5 + 5)
            self.J[leg] = self.robot.frameJacobian(neutral_fb_jointstate, self.robot.model.getFrameId(ee_frames[leg]),
                                                   pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, leg_joints]
            self.wJ[leg] = self.b_R_w.transpose().dot(self.J[leg])

        # Pinocchio Update the joint and frame placements
        gen_velocities = np.hstack((b_X_w.dot(self.baseTwistW), self.mapToPinocchio(self.qd)))
        configuration = np.hstack((self.u.linPart(self.basePoseW), self.quaternion, self.mapToPinocchio(self.q)))
        self.M = self.robot.mass(configuration)
        self.h = pin.nonLinearEffects(self.robot.model, self.robot.data, configuration, gen_velocities)
        self.h_joints = self.h[6:]

        # compute contact forces (TODO fix this torques are a bit strange)
        self.estimateContactForces()

    def estimateContactForces(self):
        # estimate ground reaction forces from tau
        for leg in range(4):
            try:
                # print(self.wJ[leg])
                # print(self.wJ[leg].T)
                grf = np.linalg.pinv(self.wJ[leg].T).dot(self.getLegJointTorques(leg, self.h_joints - self.mapToPinocchio(self.tau)))
            except np.linalg.linalg.LinAlgError as error:
                grf = np.zeros(3)
            self.setLegJointState(leg, grf, self.grForcesW)

            # print(grf)

            if self.contact_normal[leg].dot(grf) >= conf.robot_params[self.robot_name]['force_th']:
                self.contact_state[leg] = True
            else:
                self.contact_state[leg] = False
            if self.use_ground_truth_contacts:
                grfLocal_gt = self.getLegJointState(leg,  self.grForcesLocal_gt)
                grf_gt = self.w_R_lowerleg[leg] @ grfLocal_gt
                self.getLegJointState(leg, grf_gt, self.grForcesW_gt)

    def getLegJointTorques(self, legid,  jointState):
        if isinstance(legid, str):
            return jointState[self.leg_map[legid]*5:self.leg_map[legid]*5+5]
        elif isinstance(legid, int):
            return jointState[legid * 5:legid * 5 + 5]

    def setLegJointState(self, legid,  input, jointState):
        if isinstance(legid, str):
            jointState[self.leg_map[legid]*3:self.leg_map[legid]*3+3] = input
        elif isinstance(legid, int):
            jointState[legid*3:legid*3+3] = input

    def getLegJointState(self, legid,  jointState):
        if isinstance(legid, str):
            return jointState[self.leg_map[legid]*3:self.leg_map[legid]*3+3]
        elif isinstance(legid, int):
            return jointState[legid * 3:legid * 3 + 3]

def quinitic_trajectory(q0, qf, tf, time):
    a0 = q0
    a1 = 0
    a2 = 0
    a3 = (20 * qf - 20 * q0) / (2 * (tf ** 3))
    a4 = (30 * q0 - 30 * qf) / (2 * (tf ** 4))
    a5 = (12 * qf - 12 * q0) / (2 * (tf ** 5))

    pos = a0 + a1*time + a2*(time**2) + a3*(time**3) + a4*(time**4) + a5*(time**5)
    vel = a1 + 2*a2*time + 3*a3*(time**2) + 4*a4*(time**3) + 5*a5*(time**4)

    return pos

def wheel_arc_carState(leg_length):
    return leg_length - leg_length/(np.sqrt(2))

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

def initialization_state(p):
    # TODO add the other 3 legs
    id = p.robot.model.getFrameId('lf_wheel')
    id = p.robot.model.frames[id].parent
    wheel_arc_distance = wheel_arc_carState(np.array(p.robot.data.oMi[id])[1][3])
    # print(wheel_arc_distance)
    return np.copy(p.q_des_q0), wheel_arc_distance

def initTocar_state(p, rate, joints, initTocar_motion_time, act_time, shoulder_motion, wheel_arc_distance, wheel_motion):
    if act_time < initTocar_motion_time/2:
        joints["shoulder"][0] = quinitic_trajectory(0, -shoulder_motion, initTocar_motion_time / 2, act_time)
        joints["shoulder"][1] = quinitic_trajectory(0, shoulder_motion, initTocar_motion_time / 2, act_time)
        joints["shoulder"][2] = quinitic_trajectory(0, shoulder_motion, initTocar_motion_time / 2, act_time)
        joints["shoulder"][3] = quinitic_trajectory(0, -shoulder_motion, initTocar_motion_time / 2, act_time)

        if p.contact_state[0] == True:
            step = (wheel_arc_distance-joints["wheel"][1])/(((initTocar_motion_time / 2)-act_time)/float(rate))

            joints["wheel"][0] -= step
            joints["wheel"][1] += step
            joints["wheel"][2] += step
            joints["wheel"][3] -= step

            q_des = generate_q(joints)

        else:
            q_des = generate_q(joints)
    else:
        joints["pre_wheel"][0] = quinitic_trajectory(0, -wheel_motion, initTocar_motion_time / 2, act_time - (initTocar_motion_time / 2))
        joints["pre_wheel"][1] = quinitic_trajectory(0, 0., initTocar_motion_time / 2, act_time - (initTocar_motion_time / 2))
        joints["pre_wheel"][2] = quinitic_trajectory(0, 0., initTocar_motion_time / 2, act_time - (initTocar_motion_time / 2))
        joints["pre_wheel"][3] = quinitic_trajectory(0, -wheel_motion, initTocar_motion_time / 2, act_time - (initTocar_motion_time / 2))
        q_des = generate_q(joints)

    return joints, q_des


def approacing_state(p, joints):
    joints["wheel"][0] += 0.01
    joints["wheel"][1] += 0.01
    joints["wheel"][2] -= 0.01
    joints["wheel"][3] -= 0.01
    q_des = generate_q(joints)

    return joints, q_des

#def carTostar_state():

#def climbing_state():

def talker(p):
    p.start()
    #p.startSimulator(world_name = 'starbot_tunnel.world')
    p.startSimulator()
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

    # simulation parameter
    start_time_simulation = p.time
    start_time_motion=1
    initTocar_motion_time = 4
    state_flag=0
    wheel_arc_distance=0

    while not ros.is_shutdown():
        act_time = p.time-start_time_simulation

        # update the kinematics
        p.updateKinematics()
        p.tau_ffwd = 0.*np.array([0,0,0,0,   0,0,0,0 , 0 , 0, 0,0  ,0,0,0,0, 1.,1.,1.,1.])#(p.robot.na)

        # initialization phase
        if state_flag == 0:
            if act_time <= start_time_motion:
                ret = initialization_state(p)
                p.q_des = ret[0]
                wheel_arc_distance = ret[1]
            else:
                state_flag = 1
                start_time_simulation = p.time

        # movement from initial position to car state
        elif state_flag == 1:
            if act_time <= initTocar_motion_time:
                ret = initTocar_state(p, conf.robot_params[p.robot_name]['dt'], joints, initTocar_motion_time, act_time, shoulder_motion = 0.785, wheel_arc_distance = wheel_arc_distance, wheel_motion = 1.57)
                p.q_des = ret[1]
                joints = ret[0]
            else:
                state_flag = 2
                start_time_simulation = p.time

            # id = p.robot.model.getFrameId('lf_wheel')
            # id = p.robot.model.frames[id].parent
            # # leg 0 touch ground
            # if(p.contact_state[0] == True):
            #     print("check")
            # print(p.robot.data.oMi[id])
            # print(np.array(p.robot.data.oMi[id])[1][3])

        # tunnel approaching (still lo implement!!!!)
        elif state_flag == 2:
            p.q_des = p.q_des
            ret = approacing_state(p, joints)
            p.q_des = ret[1]
            joints = ret[0]

        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

        # log variables
        p.logData()

        # plot actual (green) and desired (blue) contact forces
        for leg in range(4):
            p.ros_pub.add_arrow(p.W_contacts[leg], p.getLegJointState(leg, p.grForcesW / (6 * p.robot.robot_mass)),
                                "green")
            p.ros_pub.add_marker(p.W_contacts[leg], radius=0.1)
            if (p.use_ground_truth_contacts):
                p.ros_pub.add_arrow(p.W_contacts[leg], p.getLegJointState(leg, p.grForcesW_gt / (6 * p.robot.robot_mass)),
                                    "red")
        p.ros_pub.publishVisual()

        # wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999

        # stops the while loop if  you prematurely hit CTRL+C
        if ros.is_shutdown():
            print("Shutting Down")
            break

            # restore PD when finished
    p.pid.setPDs(conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'], 0.0)
    ros.sleep(1.0)
    print("Shutting Down")
    ros.signal_shutdown("killed")
    p.deregister_node()

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



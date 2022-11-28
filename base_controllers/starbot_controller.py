# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

from __future__ import print_function
import rospy as ros
from utils.math_tools import *
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
import matplotlib.pyplot as plt
from base_controller import BaseController
from base_controllers.utils.common_functions import plotCoM, plotJoint
import pinocchio as pin
import  params as conf
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
        # to compute in the base frame  you should put neutral base
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
        #self.estimateContactForces()

    def estimateContactForces(self):
        # estimate ground reaxtion forces from tau
        for leg in range(4):
            try:
                grf = np.linalg.pinv(self.wJ[leg].T).dot(self.getLegJointTorques(leg, self.h_joints - self.mapToPinocchio(self.tau)))
            except np.linalg.linalg.LinAlgError as error:
                grf = np.zeros(3)
            self.setLegJointState(leg, grf, self.grForcesW)
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

    while not ros.is_shutdown():
        # update the kinematics
        p.updateKinematics()
        p.tau_ffwd = 1*np.array([0,0,0,0,   0,0,0,0 , 0 , 0, 0,0  ,0,0,0,0, 1.,1.,1.,1.])#(p.robot.na)
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



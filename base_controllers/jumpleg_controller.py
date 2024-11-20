# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

from __future__ import print_function

import sys

import base_controllers.params as conf
from jumpleg_rl.srv import *
from gazebo_msgs.srv import SetModelStateRequest
from base_controllers.utils.kin_dyn_utils import fifthOrderPolynomialTrajectory
from base_controllers.utils.common_functions import getRobotModel
from base_controllers.utils.ros_publish import RosPub
from base_controllers.base_controller_fixed import BaseControllerFixed
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelConfigurationRequest
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.msg import ContactsState
import roslaunch
import os
from termcolor import colored
from base_controllers.utils.common_functions import plotJoint, plotFrameLinear
from numpy import nan
import matplotlib.pyplot as plt

import rospkg
import rospy as ros
from base_controllers.utils.math_tools import *
import pinocchio as pin
np.set_printoptions(threshold=np.inf, precision=5,
                    linewidth=1000, suppress=True)


robotName = "jumpleg"


class Cost():
    def __init__(self):
        self.unilateral = 0
        self.friction = 0
        self.singularity = 0
        self.joint_range = 0
        self.joint_torques = 0
        self.error_vel_liftoff = 0
        self.error_pos_liftoff = 0
        self.unfeasible_vertical_velocity = 0
        self.no_touchdown = 100
        self.target = 0

        self.weights = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])

    def reset(self):
        self.unilateral = 0
        self.friction = 0
        self.singularity = 0
        self.joint_range = 0
        self.joint_torques = 0
        self.error_vel_liftoff = 0
        self.error_pos_liftoff = 0
        self.unfeasible_vertical_velocity = 0
        self.no_touchdown = 100
        self.target = 0


    def printCosts(self):
        return f"unil:{self.unilateral}  " \
               f"friction:{self.friction}  " \
               f"sing: {self.singularity} " \
               f"jointkin:{self.joint_range} " \
               f"torques:{self.joint_torques} " \
               f"error_vel_liftoff:{self.error_vel_liftoff} " \
               f"error_pos_liftoff:{self.error_pos_liftoff} " \
               f"unfeasible_vertical_velocity:{self.unfeasible_vertical_velocity} " \
               f"no_touchdown:{self.no_touchdown} " \
               f"target:{self.target} "


    def printWeightedCosts(self):
        return f"unil:{self.weights[0]*self.unilateral}  " \
               f"friction:{self.weights[1]*self.friction}  " \
               f"sing: {self.weights[2]*self.singularity} " \
               f"jointkin:{self.weights[3]*self.joint_range} " \
               f"torques:{self.weights[4]*self.joint_torques} " \
               f"error_vel_liftoff:{self.weights[5]*self.error_vel_liftoff} " \
               f"error_pos_liftoff:{self.weights[6]*self.error_pos_liftoff} " \
               f"unfeasible_vertical_velocity:{self.weights[7] * self.unfeasible_vertical_velocity} " \
               f"no_touchdown:{self.weights[8] * self.no_touchdown} " \
               f"target:{self.weights[9]*self.target} "


class JumpLegController(BaseControllerFixed):

    def __init__(self, robot_name="jumpleg"):
        super().__init__(robot_name=robot_name)
        self.agentMode = 'inference'
        self.agentRL = 'PPO'
        self.restoreTrain = False
        self.gui = False
        self.model_name = 'latest'
        self.EXTERNAL_FORCE = False
        self.DEBUG = False
        self.freezeBaseFlag = False
        self.inverseDynamicsFlag = False
        self.no_gazebo = False
        self.use_ground_truth_contacts = True

        if self.use_ground_truth_contacts:
            self.sub_contact_lf = ros.Subscriber("/" + self.robot_name + "/lf_foot_bumper", ContactsState,
                                                 callback=self._receive_contact_lf, queue_size=1, buff_size=2 ** 24, tcp_nodelay=True)
        print("Initialized jump leg controller---------------------------------------------------------------")

    def applyForce(self):
        from geometry_msgs.msg import Wrench, Point
        wrench = Wrench()
        wrench.force.x = 0
        wrench.force.y = 0
        wrench.force.z = 30
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0
        # you can apply forces only in this frame because this service is buggy, it will ignore any other frame
        reference_frame = "world"
        reference_point = Point(x=0, y=0, z=0)
        try:
            self.apply_body_wrench(body_name=self.robot_name+"::base_link", reference_frame=reference_frame,
                                   reference_point=reference_point, wrench=wrench, duration=ros.Duration(10))
        except:
            pass

    def updateKinematicsDynamics(self):
        # q is continuously updated
        # to compute in the base frame  you should put neutral base
        self.q_fixed = np.hstack((np.zeros(3), self.q[3:]))
        self.qd_fixed = np.hstack((np.zeros(3), self.qd[3:]))

        self.robot.computeAllTerms(self.q, self.qd)
        # joint space inertia matrix
        self.M = self.robot.mass(self.q)
        # bias terms
        self.h = self.robot.nle(self.q, self.qd)
        # gravity terms
        self.g = self.robot.gravity(self.q)
        # compute ee position  in the world frame
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        # this is expressed in a workdframe with the origin attached to the base frame origin
        self.x_ee = self.robot.framePlacement(
            self.q_fixed, self.robot.model.getFrameId(frame_name)).translation
        self.w_x_ee = self.robot.framePlacement(
            self.q, self.robot.model.getFrameId(frame_name)).translation

        # compute jacobian of the end effector in the world frame (take only the linear part and the actuated joints part)
        self.J6 = self.robot.frameJacobian(self.q, self.robot.model.getFrameId(
            frame_name), True, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]
        self.J = self.J6[:, 3:]
        self.dJdq = self.robot.frameClassicAcceleration(
            self.q, self.qd, None,  self.robot.model.getFrameId(frame_name), False).linear

        # compute com variables accordint to a frame located at the foot
        robotComB = pin.centerOfMass(
            self.robot.model, self.robot.data, self.q_fixed)

        # only for real robot
        #self.com = -self.x_ee + robotComB
        #self.comd = -self.J.dot(self.qd[3:])
        #self.comdd = -self.J.dot(self.qdd)

        # from ground truth
        self.com = self.q[:3] + robotComB
        self.comd = self.qd[:3]

        # compute contact forces
        self.estimateContactForces()

    def estimateContactForces(self):
        if not self.inverseDynamicsFlag and not self.use_ground_truth_contacts:
            self.contactForceW = np.linalg.inv(
                self.J.T).dot((self.h-self.tau)[3:])

    def _receive_contact_lf(self, msg):
        if (self.use_ground_truth_contacts):
            grf = np.zeros(3)
            grf[0] = msg.states[0].wrenches[0].force.x
            grf[1] = msg.states[0].wrenches[0].force.y
            grf[2] = msg.states[0].wrenches[0].force.z
            self.contactForceW = self.robot.framePlacement(
                self.q, self.robot.model.getFrameId("lf_lower_leg")).rotation.dot(grf)
        else:
            pass

    def initVars(self):
        super().initVars()
        self.a = np.empty((3, 6))
        self.cost = Cost()

        # unilateral friction singularity joint_range joint_torques error_vel_liftoff error_pos_liftoff unfeasible_vertical_velocity no_touchdown target
        # self.cost.weights = np.array([1000., 0.1, 10., 0.01, 1000., 300., 1000., 10., 10., 1.])
        self.cost.weights = np.array([1000., 0.1, 10., 0.01, 1000., 30., 100., 100., 10., 1.])

        self.mu = 0.8

        self.qdd_des = np.zeros(self.robot.na)
        self.base_accel = np.zeros(3)
        self.com_des =np.zeros(3)
        self.comd_des = np.zeros(3)
        self.comdd_des = np.zeros(3)
        # init new logged vars here
        self.com_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.comd_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.com_des_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.comd_des_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.comdd_des_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.qdd_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.w_x_ee_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']

        self.joint_names = conf.robot_params[self.robot_name]['joint_names']

        if self.no_gazebo:
            self.q = conf.robot_params[self.robot_name]['q_0']

        self.set_state = ros.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        print("JumplegAgent services ready")
        self.action_service = ros.ServiceProxy(
            'JumplegAgent/get_action', get_action)
        self.target_service = ros.ServiceProxy(
            'JumplegAgent/get_target', get_target)
        self.reward_service = ros.ServiceProxy(
            'JumplegAgent/set_reward', set_reward)

    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            self.com_log[:, self.log_counter] = self.com
            self.comd_log[:, self.log_counter] = self.comd
            self.com_des_log[:, self.log_counter] = self.com_des
            self.comd_des_log[:, self.log_counter] = self.comd_des
            #self.comdd_des_log[:, self.log_counter] = self.comdd_des
            self.qdd_des_log[:, self.log_counter] = self.qdd_des
            self.w_x_ee_log[:, self.log_counter] = self.w_x_ee
        super().logData()

    def resetBase(self):
        # create the message
        req_reset_joints = SetModelConfigurationRequest()
        req_reset_joints.model_name = self.robot_name
        req_reset_joints.urdf_param_name = 'robot_description'
        req_reset_joints.joint_names = self.joint_names
        req_reset_joints.joint_positions = self.q_des_q0
        # send request and get response (in this case none)
        # for i in range(10):
        #     self.reset_joints(req_reset_joints)
        # return True

        self.reset_joints(req_reset_joints)

    def freezeBase(self, flag):
        if not self.no_gazebo:
            if (self.freezeBaseFlag):
                self.freezeBaseFlag = flag
                #print(colored("releasing base", "red"))
                # gravity compensation
                self.tau_ffwd[2] = 0.
                # set base joints PD to zero
                self.pid.setPDjoint(0, 0., 0., 0.)
                self.pid.setPDjoint(1, 0., 0., 0.)
                self.pid.setPDjoint(2, 0., 0., 0.)

                # setting PD joints to 0 (needed otherwise it screws up inverse dynamics)
                if self.inverseDynamicsFlag:
                    print("setting PD joints to 0 for inv dyn")
                    self.pid.setPDjoint(3, 0., 0., 0.)
                    self.pid.setPDjoint(4, 0., 0., 0.)
                    self.pid.setPDjoint(5, 0., 0., 0.)

            if (not self.freezeBaseFlag) and (flag):
                #print(colored("freezing base", "red"))
                #print(colored(f"resetting base: {p.resetBase()}", "magenta"))

                self.freezeBaseFlag = flag
                self.q_des = self.q_des_q0.copy()
                self.qd_des = np.zeros(self.robot.na).copy()
                self.tau_ffwd = np.zeros(self.robot.na).copy()
                # compensate gravitu in the virtual joint to go exactly there
                self.tau_ffwd[2] = self.g[2]
                self.pid.setPDjoints(
                    conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'],  np.zeros(self.robot.na))
                self.resetBase()

    def thirdOrderPolynomialTrajectory(self, tf, q_0, q_lo):
        # Matrix used to solve the linear system of equations for the polynomial trajectory
        polyMatrix = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [1, tf, np.power(tf, 2), np.power(tf, 3)],
                               [0, 1, 2 * tf, 3 * np.power(tf, 2)]])

        polyVector = np.array([q_0, 0,  q_lo, 0])
        matrix_inv = np.linalg.inv(polyMatrix)
        polyCoeff = matrix_inv.dot(polyVector)

        return polyCoeff

    def forwardEulerIntegration(self, tau, force):
        # dont use gazebo
        Kp = 1000000
        Kd = 100
        w_pf = p.base_offset + p.q[:3] + p.x_ee
        w_pd_d = self.J @ self.qd
        if w_pf[2] <= 0.0:
            forceW = Kp*(-w_pf) -Kd*w_pd_d
        else:
            forceW = np.zeros(3)
        M_inv = np.linalg.inv(self.M)
        qdd = M_inv.dot(tau - self.h + self.J6.T.dot(forceW))
        # Forward Euler Integration
        self.qd += qdd * conf.robot_params[self.robot_name]['dt']
        self.q += conf.robot_params[self.robot_name]['dt'] * self.qd + \
            0.5 * pow(conf.robot_params[self.robot_name]['dt'], 2) * qdd

    def computeFeedbackAction(self, start_idx, end_idx):
        pd = 30* np.subtract(self.q_des[start_idx:end_idx], self.q[start_idx:end_idx]) \
            + 10* np.subtract(self.qd_des[start_idx:end_idx], self.qd[start_idx:end_idx])
        return pd

    def computeInverseDynamics(self, contact=None):
        S = np.vstack((np.zeros((3, 3)), np.eye(3)))
        # # add feedback part on the joints
        pd = p.computeFeedbackAction(3, 6)
        if contact:
            # debug of idyn(stabilization of base) override references and set q0
            # p.q_des = conf.robot_params[p.robot_name]['q_0']
            # p.qd_des = np.zeros(6)
            # p.qdd_des = np.zeros(6)
            # # add fback on base
            # pd = p.computeFeedbackAction(0, 3)
            # # compute constraint consinstent joint accell
            # p.qdd_des[3:] = np.linalg.inv(p.J).dot( -( p.qdd_des[:3] + pd)  -p.dJdq)
            # qdd_ref = np.zeros(6)
            # qdd_ref[:3] = p.qdd_des[:3] + pd
            # qdd_ref[3:] = p.qdd_des[3:]


            # compute constraint consinstent base accell
            p.base_accel = -p.J.dot(p.qdd_des[3:] + pd) - p.dJdq
            qdd_ref = np.zeros(6)
            qdd_ref[:3] =  p.base_accel
            qdd_ref[3:] = p.qdd_des[3:] + pd # add pd also on base

            # form matrix A , x = [tau_joints, f]
            A = np.zeros((6, 6))
            A[:6, :3] = S
            A[:6, 3:] = p.J6.T
            x = np.linalg.pinv(A).dot(p.M.dot(qdd_ref) + p.h)
            tau_leg =x[:3]
            feet_force = x[3:]
        else:
            qdd_ref = np.zeros(6)
            qdd_ref[3:] = p.qdd_des[3:] + pd  # add pd also on base
            tau_leg = S.T.dot(p.M.dot(qdd_ref) + p.h)
            feet_force = np.zeros(3)
        return tau_leg, feet_force

    def detectApex(self):
        foot_pos_w = self.base_offset + self.q[:3] + self.x_ee
        # foot tradius is 0.015
        foot_lifted_off = (foot_pos_w[2] > 0.017)
        if not self.detectedApexFlag and foot_lifted_off:
            if (self.qd[2] < 0.0):
                self.detectedApexFlag = True
                self.pause_physics_client()
                for i in range(10):
                    self.setJumpPlatformPosition(self.target_CoM)
                self.unpause_physics_client()
                self.contactForceW = np.zeros(3)
                print(colored("APEX detected", "red"))
                self.q_des[3:] = self.q_des_q0[3:]

    def computeHeuristicSolution(self, com_0, com_lo, comd_lo, T_th):
        # boundary conditions
        # position
        q_0_leg, ik_success, initial_out_of_workspace = p.ikin.invKinFoot(-com_0,
                                                                          conf.robot_params[p.robot_name]['ee_frame'],
                                                                          p.q_des_q0[3:].copy(), verbose=False)
        q_lo_leg, ik_success, final_out_of_workspace = p.ikin.invKinFoot(-com_lo,
                                                                        conf.robot_params[p.robot_name]['ee_frame'],
                                                                        p.q_des_q0[3:].copy(), verbose=False)

        # print(q_0_leg,q_f_leg)

        # we need to recompute the jacobian  for the final joint position
        J_lo = p.robot.frameJacobian(np.hstack((np.zeros(3), q_lo_leg)),
                                        p.robot.model.getFrameId(
                                            conf.robot_params[p.robot_name]['ee_frame']), True,
                                        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 3:]

        if (initial_out_of_workspace) or final_out_of_workspace:
            # put seuper high reward here
            print(colored("initial or final value out of workspace!!!!!!", "red"))

        # velocity
        qd_0_leg = np.zeros(3)
        qd_lo_leg = np.linalg.inv(J_lo).dot(-comd_lo)
        # accelerations ( we neglect Jd qd)
        qdd_0_leg = np.zeros(3)
        # we assume it stops decelerating
        Jdqd_lo = self.robot.frameClassicAcceleration(np.hstack((np.zeros(3), q_lo_leg)), np.hstack(
            (comd_lo, qd_lo_leg)), None, self.robot.model.getFrameId(conf.robot_params[p.robot_name]['ee_frame'])).linear
        comdd_lo = np.zeros(3)  # stops accelerating at the end
        qdd_lo_leg = np.linalg.inv(J_lo).dot(
            comdd_lo - Jdqd_lo)  # np.array([0.,0.,-9.81]

        # a = np.empty((3, 4))
        # for i in range(3):
        #      a[i,:] = p.thirdOrderPolynomialTrajectory(T_th, q_0_leg[i], q_f_leg[i])
        poly_coeff = np.empty((3, 6))
        for i in range(3):
            poly_coeff[i, :] = fifthOrderPolynomialTrajectory(
                T_th, q_0_leg[i], q_lo_leg[i], qd_0_leg[i], qd_lo_leg[i], qdd_0_leg[i], qdd_lo_leg[i])

        return poly_coeff
    
    def computeHeuristicSolutionBezier(self, com_0, com_lo, comd_lo, T_th):

        self.bezier_weights = np.zeros([3, 4])

        comd_0 = np.zeros(3)

        self.bezier_weights = np.zeros([3, 4])
        self.bezier_weights[:, 0] = com_0
        self.bezier_weights[:, 1] = (comd_0*(T_th/3.)) + com_0
        self.bezier_weights[:, 2] = com_lo - (comd_lo*(T_th/3.))
        self.bezier_weights[:, 3] = com_lo

    def evalBezier(self, t_, T_th):

        com = np.array(self.Bezier3(self.bezier_weights,t_,T_th))

        # it is fundamental to scale the derivative of the bezier! cause it is on a different time frame
        comd = np.array(self.Bezier2(self.bezier_weights,t_,T_th))

        q_leg, ik_success, initial_out_of_workspace = p.ikin.invKinFoot(
            -com,  conf.robot_params[p.robot_name]['ee_frame'],  p.q_des_q0[3:].copy(), verbose=False)
        # we need to recompute the jacobian  for the final joint position
        q_full = np.zeros(6)
        q_full[3:] = q_leg

        pin.forwardKinematics(self.robot.model, self.robot.data, q_full, np.zeros(self.robot.model.nv),
                              np.zeros(self.robot.model.nv))
        pin.computeJointJacobians(self.robot.model, self.robot.data)
        pin.computeFrameJacobian(self.robot.model, self.robot.data, q_full,
                                 p.robot.model.getFrameId(conf.robot_params[p.robot_name]['ee_frame']))
        J = pin.getFrameJacobian(self.robot.model, self.robot.data,
                                 p.robot.model.getFrameId(conf.robot_params[p.robot_name]['ee_frame']),
                                 pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 3:]
        # this creates spikes in qd
        #J = p.robot.frameJacobian(q_full, p.robot.model.getFrameId(conf.robot_params[p.robot_name]['ee_frame']), True,   pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 3:]



        # print(q_full.T)
        # print(J)
        # print("\n")

        qd_leg = np.linalg.pinv(J).dot(-comd)

        # we assume it stops decelerating
        Jdqd = self.robot.frameClassicAcceleration(np.hstack((np.zeros(3), q_leg)), np.hstack(
            (comd, qd_leg)), None, self.robot.model.getFrameId(conf.robot_params[p.robot_name]['ee_frame'])).linear

        comdd = np.array(self.Bezier1(self.bezier_weights,t_,T_th))

        qdd_leg = np.linalg.inv(J).dot(comdd - Jdqd)
        qdd_leg = np.zeros(3)
        return q_leg, qd_leg, qdd_leg, com, comd, comdd

    def plotTrajectory(self, T_th, poly_coeff):
        number_of_blobs = 10
        t = np.linspace(0, T_th, number_of_blobs)
        self.intermediate_com_position = []

        for blob in range(number_of_blobs):
            q_traj = np.zeros(3)
            for i in range(3):
                q_traj[i] = poly_coeff[i, 0] + poly_coeff[i, 1] * t[blob] + poly_coeff[i, 2] * pow(t[blob], 2) + poly_coeff[i, 3] * pow(
                    t[blob], 3) + poly_coeff[i, 4] * pow(t[blob], 4) + poly_coeff[i, 5] * pow(t[blob], 5)
            self.intermediate_com_position.append(-self.robot.framePlacement(np.hstack((np.zeros(
                3), q_traj)), self.robot.model.getFrameId(conf.robot_params[p.robot_name]['ee_frame'])).translation)

    def plotTrajectoryBezier(self, T_th):
        self.number_of_blobs = 10
        t = np.linspace(0, T_th, self.number_of_blobs)
        self.intermediate_com_position = []
        for blob in range(self.number_of_blobs):
            self.intermediate_com_position.append(
                self.Bezier3(self.bezier_weights, t[blob], T_th))

    def plotTrajectoryFlight(self, com_lo, comd_lo, T_fl):
        self.number_of_blobs = 10
        if T_fl is None:
            T_fl = 0.3
        t = np.linspace(0, T_fl, self.number_of_blobs)
        self.intermediate_flight_com_position = []
        com = np.zeros(3)
        for blob in range(self.number_of_blobs):
            com[2] = com_lo[2] + comd_lo[2]*t[blob] + 0.5*(-9.81)*t[blob]*t[blob]
            com[:2] = com_lo[:2] + comd_lo[:2]*t[blob]
            self.intermediate_flight_com_position.append(com.copy())

    def bernstein_pol(self,k,n,x):
        v = (np.math.factorial(n)/(np.math.factorial(k)*(np.math.factorial(n-k))))*np.power(x,k)*np.power(1-x,n-k)
        return v

    def Bezier3(self,w,t_ex,t_th):
        t = t_ex/t_th
        return  w[:,0]*self.bernstein_pol(0,3,t)+\
                w[:,1]*self.bernstein_pol(1,3,t)+\
                w[:,2]*self.bernstein_pol(2,3,t)+\
                w[:,3]*self.bernstein_pol(3,3,t)

    def Bezier2(self,w,t_ex,t_th):
        t = t_ex/t_th
        return ((w[:,1]-w[:,0]))*(3/t_th)*self.bernstein_pol(0,2,t)+\
            ((w[:,2]-w[:,1]))*(3/t_th)*self.bernstein_pol(1,2,t)+\
            ((w[:,3]-w[:,2]))*(3/t_th)*self.bernstein_pol(2,2,t)

    def Bezier1(self,w,t_ex,t_th):
        t = t_ex/t_th
        return ((w[:,2]-2*w[:,2]+w[:,0]))*(6/np.power(t_th,2))*self.bernstein_pol(0,1,t)+\
            ((w[:,3]-2*w[:,2]+w[:,1]))*(6/np.power(t_th,2))*self.bernstein_pol(1,1,t)

    def detectTouchDown(self):
        # foot_pos_w = p.base_offset + p.q[:3] + p.x_ee
        # if (foot_pos_w[2] <= 0.017 ):
        #print(p.contactForceW)
        contact_force = np.linalg.norm(self.contactForceW)
        if contact_force > 1.:
            print(colored("TOUCHDOWN detected", "red"))
            self.cost.no_touchdown = 0
            return True
        else:
            return False

    def loadRLAgent(self, mode='train', rl='TD3', data_path=None, model_name='latest', restore_train=False):
        print(colored(f"Starting {rl} RLagent in  {mode} mode", "red"))
        package = 'jumpleg_rl'
        executable = f'JumplegAgent_{rl}.py'
        name = 'rlagent'
        namespace = '/'
        args = f'--mode {mode} --data_path {data_path}_{rl} --model_name {model_name} --restore_train {restore_train}'
        node = roslaunch.core.Node(
            package, executable, name, namespace, args=args, output="screen")
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        process = self.launch.launch(node)

        # wait for agent service to start
        print("Waiting for JumplegAgent services")
        ros.wait_for_service('JumplegAgent/get_action')
        ros.wait_for_service('JumplegAgent/get_target')
        ros.wait_for_service('JumplegAgent/set_reward')

    def computeActivationFunction(self, activationType, value, lower, upper):

        if (activationType == 'linear'):
            return abs(min(value - lower, 0) + max(value-upper, 0))

        if (activationType == 'quadratic'):
            return pow(min(value - lower, 0), 2)/2.0 + pow(max(value-upper, 0), 2)/2.0

    def evaluateRunningCosts(self):

        inerruptEpisode = False
        cumsum_joint_range = 0
        cumsum_joint_torque = 0

        # only for joint variables
        for i in range(3):
            cumsum_joint_range += self.computeActivationFunction(
                'linear', self.q_des[3 + i], self.robot.model.lowerPositionLimit[3 + i], self.robot.model.upperPositionLimit[3+i])
            cumsum_joint_torque += self.computeActivationFunction(
                'linear', self.tau_ffwd[3 + i], -self.robot.model.effortLimit[3 + i], self.robot.model.effortLimit[3 + i])
        self.cost.joint_range += cumsum_joint_range
        self.cost.joint_torques += cumsum_joint_torque

        # debug
        # for i in range(3):
        #     if (self.q_des[3 + i] >= self.robot.model.upperPositionLimit[3+i]):
        #         print(colored("upper end-stop limit hit in "+str(i)+"-th leg joint","red"))
        #     if (self.q_des[3 + i] <= self.robot.model.lowerPositionLimit[3 + i]):
        #         print(colored("lower end-stop limit hit in " + str(i) + "-th leg joint", "red"))
        #     if (self.tau_ffwd[3 + i] >= self.robot.model.effortLimit[3 + i]):
        #         print(colored("upper torque limit hit in " + str(i) + "-th leg joint", "red"))
        #     if (self.tau_ffwd[3 + i] <= -self.robot.model.effortLimit[3 + i]):
        #         print(colored("lower torque limit hit in " + str(i) + "-th leg joint", "red"))

        # friction constraints
        residual = np.linalg.norm(
            p.contactForceW[:2]) - p.mu*p.contactForceW[2]
        self.cost.friction += self.computeActivationFunction(
            'linear', residual, -np.inf, 0.0)

        # unilateral constraints
        min_uloading_force = 0.
        self.cost.unilateral += self.computeActivationFunction(
            'linear', p.contactForceW[2], min_uloading_force, np.inf)

        # singularity
        # the lower singular value is also achieved when the leg squats which is not what we want
        # smallest_svalue = np.sqrt(np.min((np.linalg.eigvals(np.nan_to_num(p.J.T.dot(p.J)))))) #added nan -> 0
        # if smallest_svalue <= 0.035:
        if np.linalg.norm(self.x_ee) > 0.32:
            #self.cost.singularity = 1./(1e-05 + smallest_svalue)
            self.cost.singularity = 100
            inerruptEpisode = True
            print(colored("Getting singular configuration", "red"))


        return inerruptEpisode

    def evalTotalReward(self, com_lo, comd_lo):
        colored("Evaluating costs", "blue")
        # evaluate final target cost
        self.cost.target = np.linalg.norm(self.com - self.target_CoM)
        target_cost = 1/((50*self.cost.target) + 1e-15)
        target_cost = np.log(1+target_cost)*1000

        # evaluate final com velocity error at lift off cost
        self.cost.error_vel_liftoff = np.linalg.norm(self.actual_comd_lo - comd_lo)
        self.cost.error_pos_liftoff = np.linalg.norm(self.actual_com_lo - com_lo)

        msg = set_rewardRequest()
        print(colored("Costs: " + self.cost.printCosts(), "green"))
        print(colored("Weighted Costs: " + self.cost.printWeightedCosts(), "green"))

        total_cost = (self.cost.weights[0] * self.cost.unilateral +
                       self.cost.weights[1] * self.cost.friction +
                       self.cost.weights[2] * self.cost.singularity +
                       self.cost.weights[3] * self.cost.joint_range +
                       self.cost.weights[4] * self.cost.joint_torques +
                       self.cost.weights[5] * self.cost.error_vel_liftoff +
                       self.cost.weights[6] * self.cost.error_pos_liftoff +
                       self.cost.weights[7] * self.cost.unfeasible_vertical_velocity +
                       self.cost.weights[8] * self.cost.no_touchdown)

        reward = self.cost.weights[9] * target_cost - total_cost


        if (reward < 0):
            reward = 0

        msg.next_state = np.concatenate((self.com, self.target_CoM))
        msg.reward = reward
        # msg.target_cost = target_cost
        # msg.unilateral = self.cost.unilateral
        # msg.friction = self.cost.friction
        # msg.singularity = self.cost.singularity
        # msg.joint_range = self.cost.joint_range
        # msg.joint_torques = self.cost.joint_torques
        # msg.error_vel_liftoff = self.cost.error_vel_liftoff
        # msg.error_pos_liftoff = self.cost.error_pos_liftoff
        # msg.unfeasible_vertical_velocity = self.cost.unfeasible_vertical_velocity
        # msg.no_touchdown = self.cost.no_touchdown

        msg.target_cost = self.cost.weights[9] * target_cost
        msg.unilateral = self.cost.weights[0] * self.cost.unilateral
        msg.friction = self.cost.weights[1] * self.cost.friction
        msg.singularity = self.cost.weights[2] * self.cost.singularity
        msg.joint_range = self.cost.weights[3] * self.cost.joint_range
        msg.joint_torques = self.cost.weights[4] * self.cost.joint_torques
        msg.error_vel_liftoff = self.cost.weights[5] * self.cost.error_vel_liftoff
        msg.error_pos_liftoff = self.cost.weights[6] * self.cost.error_pos_liftoff
        msg.unfeasible_vertical_velocity = self.cost.weights[7] * self.cost.unfeasible_vertical_velocity
        msg.no_touchdown = self.cost.weights[8] * self.cost.no_touchdown
        msg.total_cost = total_cost

        self.reward_service(msg)

    def deregister_node(self):
        super().deregister_node()
        os.system(" rosnode kill /"+self.robot_name +
                  "/ros_impedance_controller")
        os.system(" rosnode kill /gzserver /gzclient")
        os.system(" pkill rosmaster")

    def setJumpPlatformPosition(self, target):
        if not self.no_gazebo:
            # create the message
            set_platform_position = SetModelStateRequest()
            # create model state
            model_state = ModelState()
            model_state.model_name = 'jump_platform'
            model_state.pose.position.x = target[0]
            model_state.pose.position.y = target[1]
            model_state.pose.position.z = target[2]-0.25252
            model_state.pose.orientation.w = 1.0
            model_state.pose.orientation.x = 0.0
            model_state.pose.orientation.y = 0.0
            model_state.pose.orientation.z = 0.0
            set_platform_position.model_state = model_state
            # send request and get response (in this case none)
            self.set_state(set_platform_position)

    def loadModelAndPublishers(self,  xacro_path = None, additional_urdf_args = None):
        super().loadModelAndPublishers()
        self.reset_joints = ros.ServiceProxy(
            '/gazebo/set_model_configuration', SetModelConfiguration)

    def computeIdealLanding(self, com_lo, comd_lo, target_CoM):
        #get time of flight
        arg = comd_lo[2]*comd_lo[2] - 2*9.81*(target_CoM[2] - com_lo[2])
        if arg<0:
            print(colored("Point Beyond Reach, tagret too high","red"))
            return False
        else:  #beyond apex
            self.T_fl = (comd_lo[2] + math.sqrt(arg))/9.81 # we take the highest value
            self.ideal_landing = np.hstack((com_lo[:2] + self.T_fl*comd_lo[:2], target_CoM[2]))
            return True


def talker(p):

    p.start()
    # dont use gazebo
    if p.no_gazebo:
        p.ros_pub = RosPub("jumpleg")
        p.robot = getRobotModel("jumpleg")

    else:
        if p.DEBUG:
            additional_args=['gui:=true']
        else:
            additional_args = [f'gui:={p.gui}']
        p.startSimulator("jump_platform.world", additional_args=additional_args)
        # p.startSimulator()
        p.loadModelAndPublishers()
        p.startupProcedure()


    p.loadRLAgent(mode=p.agentMode, rl=p.agentRL ,data_path=os.environ["LOCOSIM_DIR"] + "/robot_control/jumpleg_rl/runs", model_name=p.model_name, restore_train=p.restoreTrain)

    p.initVars()
    ros.sleep(1.0)
    p.q_des = np.copy(p.q_des_q0)

    # loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    # compute coeff first time
    p.updateKinematicsDynamics()

    # initial com posiiton
    com_0 = np.array([-0.01303,  0.00229,  0.25252])
    p.number_of_episode = 0

    # here the RL loop...
    while True:
        # Reset variables
        p.initVars()
        p.q_des = np.copy(p.q_des_q0)

        p.time = 0.
        startTrust = 0.2
        max_episode_time = 2.0
        p.number_of_episode += 1
        p.freezeBase(True)
        p.firstTime = True
        p.detectedApexFlag = False
        p.trustPhaseFlag = False
        p.intermediate_com_position = []
        p.intermediate_flight_com_position = []
        p.actual_com_lo = np.zeros(3)
        p.actual_comd_lo = np.zeros(3)
        p.ideal_landing = np.zeros(3)
        p.target_CoM = (p.target_service()).target_CoM

        p.pause_physics_client()
        for i in range(10):
            p.setJumpPlatformPosition(com_0-[0,0,0.3])
        p.unpause_physics_client()


        if p.target_CoM[2] == -1:
            print(colored("# RECIVED STOP TARGET_COM SIGNAL #", "red"))
            break


        state = np.concatenate((com_0, p.target_CoM))
        action = p.action_service(state).action
        #print("Action from agent:", action)
        p.T_th = action[0]
        com_lo = np.array(action[1:4])
        comd_lo = np.array(action[4:])

        #debug
        # p.T_th = 0.4851008642464877
        # com_f= np.array([-0.07669,  0.05105 , 0.25583])
        # comd_f= np.array([-0.878,    0.58451 , 0.83095])
        # p.target_CoM = np.array([1,.0,1])
        p.computeHeuristicSolutionBezier(com_0, com_lo, comd_lo, p.T_th)
        p.plotTrajectoryBezier(p.T_th)

        # OLD way
        # p.a = p.computeHeuristicSolution(com_0, com_f, comd_f, p.T_th)
        # print(f"Actor action:\n"
        #       f"T_th: {p.T_th}\n"
        #       f"haa: {p.a[0, :]}\n"
        #       f"hfe: {p.a[1, :]}\n"
        #       f"kfe: {p.a[2, :]}\n")

        # Control loop
        while not ros.is_shutdown():
            # update the kinematics
            p.updateKinematicsDynamics()
            if (p.time > startTrust):
                if (p.time > startTrust + max_episode_time):
                    # max episode time elapsed
                    print(colored("--Max time elapsed!--", "blue"))
                    break
                # release base
                if p.firstTime:
                    p.firstTime = False
                    # to debug the trajectory comment this and set q0[2] = 0.3 om the param file
                    p.freezeBase(False)

                    print("\n\n")
                    print(colored(f"STARTING A NEW EPISODE--------------------------------------------# :{p.number_of_episode}", "red"))
                    print("Target position from agent:", p.target_CoM)
                    print(f"Actor action:\n"
                          f"T_th: {p.T_th}\n"
                          f"com_lo: {com_lo}\n"
                          f"comd_lo: {comd_lo}")

                    p.trustPhaseFlag = True
                    p.T_fl = None
                    # Error landing
                    if (p.computeIdealLanding(com_lo, comd_lo, p.target_CoM)):
                        error = np.linalg.norm(p.ideal_landing - p.target_CoM)
                        p.cost.unfeasible_vertical_velocity = error
                    else:
                        p.cost.unfeasible_vertical_velocity = 100.
                        break
                    p.plotTrajectoryFlight(com_lo, comd_lo, p.T_fl)
                # compute joint reference
                if (p.trustPhaseFlag):
                    t = p.time - startTrust
                    p.q_des[3:], p.qd_des[3:], p.qdd_des[3:] , p.com_des, p.comd_des, p.comdd_des = p.evalBezier(t, p.T_th)

                    if p.evaluateRunningCosts():
                        break
                    if p.time > (startTrust + p.T_th):
                        p.trustPhaseFlag = False
                        # sample actual com position and velocity
                        p.actual_comd_lo = np.copy(p.comd)
                        p.actual_com_lo = np.copy(p.com)
                        #we se this here to have enough retraction (important)
                        p.q_des[3:] = p.q_des_q0[3:]
                        p.qd_des[3:] = np.zeros(3)
                else:
                    # apex detection
                    p.detectApex()
                    if (p.detectedApexFlag):

                        # set jump position (avoid collision in jumping)
                        if p.detectTouchDown():
                            break

                # compute control action
                if p.inverseDynamicsFlag:  # TODO fix this
                    if (p.time < startTrust + p.T_th):
                        contact = True
                    else:
                        contact = False
                    p.tau_ffwd[3:], p.contactForceW = p.computeInverseDynamics(contact)

                else:
                    if (p.time < startTrust + p.T_th):
                        # add gravity compensation
                        p.tau_ffwd[3:] = - p.J.T.dot(p.g[:3])# +p.robot.robot_mass*p.comdd)
                    else:
                        p.tau_ffwd[3:] = np.zeros(3)

                # send commands to gazebo
                if p.no_gazebo:
                    p.forwardEulerIntegration(p.tau_ffwd, p.contactForceW)
                    p.ros_pub.publish(p.robot, p.q)

            if not p.no_gazebo:
                p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

            # log variables
            if p.DEBUG:
                p.logData()

            # disturbance force
            # if (p.time > 3.0 and p.EXTERNAL_FORCE):
            #     p.applyForce()
            #     p.EXTERNAL_FORCE = False

            # plot end-effector and contact force
            if not p.use_ground_truth_contacts:
                p.ros_pub.add_arrow(
                    p.base_offset + p.q[:3] + p.x_ee, p.contactForceW / (10 * p.robot.robot_mass), "green")
            else:
                p.ros_pub.add_arrow(
                    p.base_offset + p.q[:3] + p.x_ee, p.contactForceW / (10 * p.robot.robot_mass), "red")
            #plot end-effector
            p.ros_pub.add_marker(p.base_offset + p.q[:3] + p.x_ee, radius=0.05)
            p.ros_pub.add_cone(
                p.base_offset + p.q[:3] + p.x_ee, np.array([0, 0, 1.]), p.mu, height=0.05, color="blue")
            # p.contactForceW = np.zeros(3) # to be sure it does not retain any "memory" when message are not arriving, so avoid to compute wrong rewards
            p.ros_pub.add_marker(p.target_CoM, color="blue", radius=0.1)

            if (np.linalg.norm(p.ideal_landing) > 0.):
                p.ros_pub.add_marker(p.ideal_landing, color="purple", radius=0.1)
            #reachable space
            #p.ros_pub.add_marker([0, 0, 0], color="green", radius=0.64)

            # plot com intermediate positions
            for blob in range(len(p.intermediate_com_position)):
                p.ros_pub.add_marker(p.intermediate_com_position[blob], color=[
                                     blob*1./p.number_of_blobs, blob*1./p.number_of_blobs, blob*1./p.number_of_blobs], radius=0.02)
            # plot com intermediate positions
            for blob in range(len(p.intermediate_flight_com_position)):
                p.ros_pub.add_marker(p.intermediate_flight_com_position[blob], color=[blob * 1. / p.number_of_blobs, blob * 1. / p.number_of_blobs,
                                                blob * 1. / p.number_of_blobs], radius=0.02)
            if p.DEBUG:
                p.ros_pub.add_marker(p.bezier_weights[:, 0], color="red",  radius=0.02)
                p.ros_pub.add_marker(p.bezier_weights[:, 1], color="red", radius=0.02)
                p.ros_pub.add_marker(p.bezier_weights[:, 2], color="red", radius=0.02)
                p.ros_pub.add_marker(p.bezier_weights[:, 3], color="red", radius=0.02)


            # com at LIFT OFF given by the N network
            if p.DEBUG:
                p.ros_pub.add_arrow(com_lo, comd_lo, "red")
                p.ros_pub.add_marker(com_lo, color="red", radius=0.1)

            if (not p.trustPhaseFlag):
                if p.DEBUG:
                    p.ros_pub.add_arrow(p.actual_com_lo, p.actual_comd_lo, "green")
                    p.ros_pub.add_marker(p.actual_com_lo, color="green", radius=0.1)

            p.ros_pub.publishVisual()

            # wait for synconization of the control loop
            rate.sleep()

            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 4) # to avoid issues of dt 0.0009999

        p.pause_physics_client()
        # eval rewards
        p.evalTotalReward(com_lo, comd_lo)
        p.cost.reset()

        if p.DEBUG:
            plotJoint('position', p.time_log, q_log=p.q_log, q_des_log=p.q_des_log,
                      joint_names=conf.robot_params[p.robot_name]['joint_names'])
            plt.savefig(f'{p.number_of_episode}.png')
            plt.close('all')
            #break


if __name__ == '__main__':
    p = JumpLegController(robotName)

    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
    finally:
        ros.signal_shutdown("killed")
        p.deregister_node()
        if p.DEBUG:
            print("PLOTTING")
            plotFrameLinear('velocity', p.time_log, des_Twist_log=p.comd_des_log)
            plotFrameLinear('wrench', p.time_log,  Wrench_log=p.contactForceW_log)
            plotJoint('position', p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
            plotJoint('velocity', p.time_log, qd_log=p.qd_log, qd_des_log=p.qd_des_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
            #plotJoint('acceleration', p.time_log, qdd_log=p.qdd_log,qdd_des_log=p.qdd_des_log,joint_names=conf.robot_params[p.robot_name]['joint_names'])
        ros.signal_shutdown("killed")
        p.deregister_node()

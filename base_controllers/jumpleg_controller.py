# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

from __future__ import print_function
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
        self.unfeasible_vertical_velocity =0
        self.target = 0


        self.weights = np.array([0., 0., 0., 0., 0., 0., 0., 0.])

    def reset(self):
        self.unilateral = 0
        self.friction = 0
        self.singularity = 0
        self.joint_range = 0
        self.joint_torques = 0
        self.error_vel_liftoff = 0
        self.unfeasible_vertical_velocity = 0
        self.target = 0


    def printCosts(self):
        return f"unil:{self.unilateral}  " \
               f"friction:{self.friction}  " \
               f"sing: {self.singularity} " \
               f"jointkin:{self.joint_range} " \
               f"torques:{self.joint_torques} " \
               f"error_vel_liftoff:{self.error_vel_liftoff} " \
               f"unfeasible_vertical_velocity:{self.unfeasible_vertical_velocity}"\
               f"target:{self.target}"


    def printWeightedCosts(self):
        return f"unil:{self.weights[0]*self.unilateral}  " \
               f"friction:{self.weights[1]*self.friction}  " \
               f"sing: {self.weights[2]*self.singularity} " \
               f"jointkin:{self.weights[3]*self.joint_range} " \
               f"torques:{self.weights[4]*self.joint_torques} " \
               f"error_vel_liftoff:{self.weights[5]*self.error_vel_liftoff} " \
               f"target:{self.weights[7] * self.unfeasible_vertical_velocity}"\
               f"target:{self.weights[6]*self.target}"


class JumpLegController(BaseControllerFixed):

    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)
        self.AgentMode = 'train'

        self.EXTERNAL_FORCE = False
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

        #  unilateral  friction   singularity      joint_range  joint_torques   error_vel_liftoff  unfeasible vertical velocity target = 0
        # self.cost.weights = np.array([10., 10., 10., 10., 10., 100., 1.])
        self.cost.weights = np.array([1., 1., 10., 0.01, 1., 10.,10., 1.])

        self.mu = 0.8

        self.qdd_des = np.zeros(self.robot.na)
        self.base_accel = np.zeros(3)
        # init new logged vars here
        self.com_log = np.empty(
            (3, conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.comd_log = np.empty(
            (3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        #self.comdd_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.qdd_des_log = np.empty(
            (self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan

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
            #self.comdd_log[:, self.log_counter] = self.comdd
            self.qdd_des_log[:, self.log_counter] = self.qdd_des
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

    def thirdOrderPolynomialTrajectory(self, tf, q0, qf):
        # Matrix used to solve the linear system of equations for the polynomial trajectory
        polyMatrix = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [1, tf, np.power(tf, 2), np.power(tf, 3)],
                               [0, 1, 2 * tf, 3 * np.power(tf, 2)]])

        polyVector = np.array([q0, 0,  qf, 0])
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
        foot_pos_w = p.base_offset + p.q[:3] + p.x_ee
        # foot tradius is 0.015
        foot_lifted_off = (foot_pos_w[2] > 0.017)
        if not self.detectedApexFlag and foot_lifted_off:
            if (self.qd[2] < 0.0):
                self.detectedApexFlag = True
                p.pause_physics_client()
                for i in range(10):
                    p.setJumpPlatformPosition(p.target_CoM)
                p.unpause_physics_client()
                p.contactForceW = np.zeros(3)
                print(colored("APEX detected", "red"))
                self.q_des[3:] = self.q_des_q0[3:]

    def computeHeuristicSolution(self, com_0, com_f, comd_f, T_th):
        # boundary conditions
        # position
        q_0_leg, ik_success, initial_out_of_workspace = p.ikin.invKinFoot(-com_0,
                                                                          conf.robot_params[p.robot_name]['ee_frame'],
                                                                          p.q_des_q0[3:].copy(), verbose=False)
        q_f_leg, ik_success, final_out_of_workspace = p.ikin.invKinFoot(-com_f,
                                                                        conf.robot_params[p.robot_name]['ee_frame'],
                                                                        p.q_des_q0[3:].copy(), verbose=False)

        # print(q_0_leg,q_f_leg)

        # we need to recompute the jacobian  for the final joint position
        J_final = p.robot.frameJacobian(np.hstack((np.zeros(3), q_f_leg)),
                                        p.robot.model.getFrameId(
                                            conf.robot_params[p.robot_name]['ee_frame']), True,
                                        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 3:]

        if (initial_out_of_workspace) or final_out_of_workspace:
            # put seuper high reward here
            print(colored("initial or final value out of workspace!!!!!!", "red"))

        # velocity
        qd_0_leg = np.zeros(3)
        qd_f_leg = np.linalg.inv(J_final).dot(-comd_f)
        # accelerations ( we neglect Jd qd)
        qdd_0_leg = np.zeros(3)
        # we assume it stops decelerating
        Jdqd_f = self.robot.frameClassicAcceleration(np.hstack((np.zeros(3), q_f_leg)), np.hstack(
            (comd_f, qd_f_leg)), None, self.robot.model.getFrameId(conf.robot_params[p.robot_name]['ee_frame'])).linear
        comdd_f = np.zeros(3)  # stops accelerating at the end
        qdd_f_leg = np.linalg.inv(J_final).dot(
            comdd_f - Jdqd_f)  # np.array([0.,0.,-9.81]

        # a = np.empty((3, 4))
        # for i in range(3):
        #      a[i,:] = p.thirdOrderPolynomialTrajectory(T_th, q_0_leg[i], q_f_leg[i])
        poly_coeff = np.empty((3, 6))
        for i in range(3):
            poly_coeff[i, :] = fifthOrderPolynomialTrajectory(
                T_th, q_0_leg[i], q_f_leg[i], qd_0_leg[i], qd_f_leg[i], qdd_0_leg[i], qdd_f_leg[i])

        return poly_coeff

    def computeHeuristicSolutionBezier(self, com_0, com_f, comd_f, T_th):
        self.bezier_weights = np.zeros([3, 4])
        self.bezier_weights_der = np.zeros([3, 3])
        self.bezier_weights_2der = np.zeros([3, 2])
        comd_0 = np.zeros(3)
        self.bezier_weights[:, 0] = com_0
        self.bezier_weights[:, 1] = comd_0*T_th / 3. + com_0
        self.bezier_weights[:, 2] = com_f - comd_f*T_th / 3.
        self.bezier_weights[:, 3] = com_f
        self.bezier_weights_der[:, 0] = 1/T_th * 3 * \
            (self.bezier_weights[:, 1] - self.bezier_weights[:, 0])
        self.bezier_weights_der[:, 1] = 1/T_th *  3 * \
            (self.bezier_weights[:, 2] - self.bezier_weights[:, 1])
        self.bezier_weights_der[:, 2] = 1/T_th *  3 * \
            (self.bezier_weights[:, 3] - self.bezier_weights[:, 2])

        self.bezier_weights_2der[:, 0] = 1/(T_th*T_th) * 2 * (self.bezier_weights_der[:, 1] - self.bezier_weights_der[:, 0])
        self.bezier_weights_2der[:, 1] = 1/(T_th*T_th) * 2 * (self.bezier_weights_der[:, 2] - self.bezier_weights_der[:, 1])
        # print(com_0)
        # print(com_f)
        # print(comd_f)

    def evalBezier(self, t_, T_th):
        t = t_ / T_th
        com = self.Bezier3(t, self.bezier_weights)
        # it is fundamental to scale the derivative of the bezier! cause it is on a different time frame
        comd = self.Bezier2(t, self.bezier_weights_der)
        q_leg, ik_success, initial_out_of_workspace = p.ikin.invKinFoot(
            -com,  conf.robot_params[p.robot_name]['ee_frame'],  p.q_des_q0[3:].copy(), verbose=False)
        # we need to recompute the jacobian  for the final joint position
        J= p.robot.frameJacobian(np.hstack((np.zeros(3), q_leg)),   p.robot.model.getFrameId(
            conf.robot_params[p.robot_name]['ee_frame']), True,   pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 3:]
        qd_leg = np.linalg.inv(J).dot(-comd)
        # we assume it stops decelerating
        Jdqd = self.robot.frameClassicAcceleration(np.hstack((np.zeros(3), q_leg)), np.hstack(
            (comd, qd_leg)), None, self.robot.model.getFrameId(conf.robot_params[p.robot_name]['ee_frame'])).linear

        comdd = (self.bezier_weights_2der[:,0]*(1-t) + self.bezier_weights_2der[:,1]*t)
        comdd = (self.bezier_weights_2der[:,0]*(1-t) + self.bezier_weights_2der[:,1]*t)
        qdd_leg = np.linalg.inv(J).dot(comdd - Jdqd)
        qdd_leg = np.zeros(3)
        return q_leg, qd_leg, qdd_leg, comdd

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
                self.Bezier3(t[blob]/T_th, self.bezier_weights))

    def plotTrajectoryFlight(self, com_f, comd_f, T_fl):
        self.number_of_blobs = 10
        if T_fl is None:
            T_fl = 0.3
        t = np.linspace(0, T_fl, self.number_of_blobs)
        self.intermediate_flight_com_position = []
        com = np.zeros(3)
        for blob in range(self.number_of_blobs):
            com[2] = com_f[2] + comd_f[2]*t[blob] + 0.5*(-9.81)*t[blob]*t[blob]
            com[:2] = com_f[:2] + comd_f[:2]*t[blob]
            self.intermediate_flight_com_position.append(com.copy())

    # 2 order bezier
    def Bezier2(self, t, weights):
        t2 = t * t
        mt = 1 - t
        mt2 = mt * mt
        return weights[:, 0] * mt2 + weights[:, 1] * 2 * mt * t + weights[:, 2] * t2

    def Bezier3(self, t, weights):
        t2 = t * t
        t3 = t2 * t
        mt = 1 - t
        mt2 = mt * mt
        mt3 = mt2 * mt
        return weights[:, 0] * mt3 + 3 * weights[:, 1] * mt2 * t + 3 * weights[:, 2] * mt * t2 + weights[:, 3] * t3

    def detectTouchDown(self):
        # foot_pos_w = p.base_offset + p.q[:3] + p.x_ee
        # if (foot_pos_w[2] <= 0.017 ):
        #print(p.contactForceW)
        contact_force = np.linalg.norm(p.contactForceW)
        if contact_force > 1.:
            print(colored("TOUCHDOWN detected", "red"))
            return True
        else:
            return False

    def loadRLAgent(self, mode='train', data_path=None, model_name='latest', restore_train=False):
        print(colored(f"Starting RLagent in  {mode} mode", "red"))
        package = 'jumpleg_rl'
        executable = 'JumplegAgent.py'
        name = 'rlagent'
        namespace = '/'
        args = f'--mode {mode} --data_path {data_path} --model_name {model_name} --restore_train {restore_train}'
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

    def evalTotalReward(self, comd_f):
        colored("Evaluating costs", "blue")
        # evaluate final target cost
        self.cost.target = np.linalg.norm(self.com - self.target_CoM)
        target_cost = 1/((50*self.cost.target) + 1e-15)
        target_cost = np.log(1+target_cost)*1000
        # evaluate final com velocity error at lift off cost
        self.cost.error_vel_liftoff = np.linalg.norm(self.comd_lo - comd_f)

        msg = set_rewardRequest()
        print(colored("Costs: " + self.cost.printCosts(), "green"))
        print(colored("Weighted Costs: " + self.cost.printWeightedCosts(), "green"))

        reward = self.cost.weights[7] * target_cost - (self.cost.weights[0]*self.cost.unilateral +
                                                       self.cost.weights[1]*self.cost.friction +
                                                       self.cost.weights[2] * self.cost.singularity +
                                                       self.cost.weights[3]*self.cost.joint_range +
                                                       self.cost.weights[4] * self.cost.joint_torques +
                                                       self.cost.weights[5] * self.cost.error_vel_liftoff+
                                                       self.cost.weights[6] * self.cost.unfeasible_vertical_velocity)


        if (reward < 0):
            reward = 0

        # unil  friction sing jointrange torques target
        msg.next_state = np.concatenate((self.com, self.target_CoM))
        msg.reward = reward
        msg.target_cost = target_cost
        msg.unilateral = self.cost.unilateral
        msg.friction = self.cost.friction
        msg.singularity = self.cost.singularity
        msg.joint_range = self.cost.joint_range
        msg.joint_torques = self.cost.joint_torques
        msg.error_vel_liftoff = self.cost.error_vel_liftoff

        self.reward_service(msg)

    def deregister_node(self):
        super().deregister_node()
        os.system(" rosnode kill /"+self.robot_name +
                  "/ros_impedance_controller")
        os.system(" rosnode kill /gzserver /gzclient")
        os.system(" pkill rosmaster")

    def setJumpPlatformPosition(self, target):
        if not p.no_gazebo:
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

    def computeIdealLanding(self, com_f, comd_f, target_CoM):
        #get time of flight
        arg = comd_f[2]*comd_f[2] - 2*9.81*(target_CoM[2] - com_f[2])
        if arg<0:
            print(colored("Point Beyond Reach, tagret too high","red"))
            return False
        else:  #beyond apex
            self.T_fl = (comd_f[2] + math.sqrt(arg))/9.81 # we take the highest value
            self.ideal_landing = np.hstack((com_f[:2] + self.T_fl*comd_f[:2], target_CoM[2]))
            return True


def talker(p):

    p.start()
    # dont use gazebo
    if p.no_gazebo:
        p.ros_pub = RosPub("jumpleg")
        p.robot = getRobotModel("jumpleg")

    else:
        additional_args=['gui:=true']
        p.startSimulator("jump_platform.world", additional_args=additional_args)
        # p.startSimulator()
        p.loadModelAndPublishers()
        p.startupProcedure()


    if p.AgentMode == 'inference':
        p.loadRLAgent(mode='inference', data_path=os.environ["LOCOSIM_DIR"] + "/robot_control/jumpleg_rl/final_runs", model_name='latest', restore_train=False)
    else:
        #p.loadRLAgent(mode='test', data_path=os.environ["LOCOSIM_DIR"] + "/robot_control/jumpleg_rl/runs", model_name='latest', restore_train=False)
        p.loadRLAgent(mode='train', data_path=os.environ["LOCOSIM_DIR"]+"/robot_control/jumpleg_rl/runs", model_name='latest', restore_train=False)

    p.initVars()
    ros.sleep(1.0)
    p.q_des = np.copy(p.q_des_q0)

    # loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    # compute coeff first time
    p.updateKinematicsDynamics()

    # initial com posiiton
    com_0 = np.array([-0.01303,  0.00229,  0.25252])

    plt.ion()
    figure = plt.figure(figsize=(15, 10))
    p.number_of_episode = 0

    # here the RL loop...
    while True:
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
        p.comd_lo = np.zeros(3)
        p.ideal_landing = np.zeros(3)
        p.target_CoM = (p.target_service()).target_CoM


        for i in range(10):
            p.setJumpPlatformPosition(com_0-[0,0,0.2])

        if p.target_CoM[2] == -1:
            print(colored("# RECIVED STOP TARGET_COM SIGNAL #", "red"))
            break

        state = np.concatenate((com_0, p.target_CoM))
        action = p.action_service(state).action
        #print("Action from agent:", action)
        p.T_th = action[0]
        com_f = np.array(action[1:4])
        comd_f = np.array(action[4:])

        #debug
        # p.T_th = 0.4851008642464877
        # com_f= np.array([-0.07669,  0.05105 , 0.25583])
        # comd_f= np.array([-0.878,    0.58451 , 0.83095])
        # p.target_CoM = np.array([1,.0,1])
        p.computeHeuristicSolutionBezier(com_0, com_f, comd_f, p.T_th)
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
                          f"com_f: {com_f}\n"
                          f"comd_f: {comd_f}")

                    p.trustPhaseFlag = True
                    p.T_fl = None
                    if (p.computeIdealLanding(com_f, comd_f, p.target_CoM)):
                        error = np.linalg.norm(p.ideal_landing - p.target_CoM)
                    # else:
                    #     p.cost.unfeasible_vertical_velocity = 0.
                    #     break
                    p.plotTrajectoryFlight(com_f, comd_f, p.T_fl)
                # compute joint reference
                if (p.trustPhaseFlag):
                    t = p.time - startTrust
                    # for i in range(3):
                    #     # third order
                    #     # p.q_des[3+i] = p.a[i, 0] + p.a[i,1] * t + p.a[i,2] * pow(t, 2) + p.a[i,3] * pow(t, 3)
                    #     # p.qd_des[3+i] = p.a[i, 1] + 2 * p.a[i,2] * t + 3 * p.a[i,3] * pow(t, 2)
                    #     #fifth order
                    #     p.q_des[3 + i] = p.a[i, 0] + p.a[i, 1] * t + p.a[i, 2] * pow(t, 2) + p.a[i, 3] * pow(t, 3) +  p.a[i,4] *pow(t, 4) + p.a[i,5] *pow(t, 5)
                    #     p.qd_des[3 + i] = p.a[i, 1] + 2 * p.a[i, 2] * t + 3 * p.a[i, 3] * pow(t, 2) + 4 * p.a[i,4] * pow(t, 3) + 5 * p.a[i,5] * pow(t, 4)
                    #     p.qdd_des[3 + i] = 2 * p.a[i,2] + 6 * p.a[i,3] * t + 12 * p.a[i,4] * pow(t, 2) + 20 * p.a[i,5] * pow(t, 3)

                    p.q_des[3:], p.qd_des[3:], p.qdd_des[3:] , p.comdd = p.evalBezier(t, p.T_th)

                    # uncomment in inference mode

                    if p.evaluateRunningCosts():
                        break
                    if p.time > (startTrust + p.T_th):
                        p.trustPhaseFlag = False
                        # sample actual com velocity
                        p.comd_lo = np.copy(p.comd)
                    #singluarity = p.evaluateRunningCosts()

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
            #p.logData()

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
            # com at LIFT OFF
            p.ros_pub.add_marker(com_f, color="red", radius=0.1)

            p.ros_pub.add_marker(p.ideal_landing, color="green", radius=0.1)
            #reachable space
            #p.ros_pub.add_marker([0, 0, 0], color="green", radius=0.64)

            p.ros_pub.add_arrow(com_f, comd_f/10.0, "red")
            # plot com intermediate positions
            for blob in range(len(p.intermediate_com_position)):
                p.ros_pub.add_marker(p.intermediate_com_position[blob], color=[
                                     blob*1./p.number_of_blobs, blob*1./p.number_of_blobs, blob*1./p.number_of_blobs], radius=0.02)
            # plot com intermediate positions
            for blob in range(len(p.intermediate_flight_com_position)):
                p.ros_pub.add_marker(p.intermediate_flight_com_position[blob], color=[blob * 1. / p.number_of_blobs, blob * 1. / p.number_of_blobs,
                                                blob * 1. / p.number_of_blobs], radius=0.02)
            p.ros_pub.add_marker(p.bezier_weights[:, 0], color="green",  radius=0.02)
            p.ros_pub.add_marker(p.bezier_weights[:, 1], color="green", radius=0.02)
            p.ros_pub.add_marker(p.bezier_weights[:, 2], color="green", radius=0.02)
            p.ros_pub.add_marker(p.bezier_weights[:, 3], color="green", radius=0.02)
            if (not p.trustPhaseFlag):
                p.ros_pub.add_arrow(com_f, p.comd_lo/10., "green")

            p.ros_pub.publishVisual()

            # wait for synconization of the control loop
            rate.sleep()

            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999


        # eval rewards
        p.evalTotalReward(comd_f)
        p.cost.reset()
        plt.cla()


if __name__ == '__main__':
    p = JumpLegController(robotName)

    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
    finally:
        if conf.plotting:
            print("PLOTTING")
            # plotFrameLinear('com position', 1, p.time_log, None, p.com_log)
            # # plotFrameLinear('contact force', 2, p.time_log,
            # #               None, p.contactForceW_log)
            # plotJoint('position', 3, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, p.qdd_des_log, None,
            #             joint_names=conf.robot_params[p.robot_name]['joint_names'])
            # plotJoint('velocity', 4, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, p.qdd_des_log, None,
            #           joint_names=conf.robot_params[p.robot_name]['joint_names'])
            # plotJoint('acceleration', 5, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, p.qdd_des_log, None,
            #           joint_names=conf.robot_params[p.robot_name]['joint_names'])


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
from base_controllers.utils.common_functions import plotJoint
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
        self.target = 0

        self.weights = np.array([0., 0., 0., 0., 0., 0.])

    def reset(self):
        self.unilateral = 0
        self.friction = 0
        self.singularity = 0
        self.joint_range = 0
        self.joint_torques = 0
        self.error_vel_liftoff = 0
        self.target = 0

    def printCosts(self):
        return f"unil:{self.unilateral}  " \
               f"friction:{self.friction}  " \
               f"sing: {self.singularity} " \
               f"jointkin:{self.joint_range} " \
               f"torques:{self.joint_torques} " \
               f"target:{self.target}"

    def printWeightedCosts(self):
        return f"unil:{self.weights[0]*self.unilateral}  " \
               f"friction:{self.weights[1]*self.friction}  " \
               f"sing: {self.weights[2]*self.singularity} " \
               f"jointkin:{self.weights[3]*self.joint_range} " \
               f"torques:{self.weights[4]*self.joint_torques} " \
               f"target:{self.weights[5]*self.target}"


class JumpLegController(BaseControllerFixed):

    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)
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

        #  unilateral  friction   singularity   joint_range  joint_torques target 
        self.cost.weights = np.array([1., 1., 10., 0.01, 1., 1.])

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

    def forwardEulerIntegration(self, tau, force):
        # dont use gazebo
        M_inv = np.linalg.inv(self.M)
        qdd = M_inv.dot(tau - self.h + self.J6.T.dot(force))
        # Forward Euler Integration
        self.qd += qdd * conf.robot_params[self.robot_name]['dt']
        self.q += conf.robot_params[self.robot_name]['dt'] * self.qd + \
            0.5 * pow(conf.robot_params[self.robot_name]['dt'], 2) * qdd

    def computeFeedbackAction(self, start_idx, end_idx):
        pd = np.multiply(conf.robot_params[self.robot_name]['kp'][start_idx:end_idx], np.subtract(self.q_des[start_idx:end_idx], self.q[start_idx:end_idx])) \
            + np.multiply(conf.robot_params[self.robot_name]['kd'][start_idx:end_idx], np.subtract(
                self.qd_des[start_idx:end_idx], self.qd[start_idx:end_idx]))
        return pd

    def computeInverseDynamics(self):
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

        # # add feedback part on the joints
        pd = p.computeFeedbackAction(3, 6)
        # compute constraint consinstent base accell
        p.base_accel = -p.J.dot(p.qdd_des[3:] + pd) - p.dJdq
        qdd_ref = np.zeros(6)
        qdd_ref[:3] = p.base_accel
        qdd_ref[3:] = p.qdd_des[3:] + pd

        # form matrix A , x = [tau_joints, f]
        S = np.vstack((np.zeros((3, 3)), np.eye(3)))
        A = np.zeros((6, 6))
        A[:6, :3] = S
        A[:6, 3:] = p.J6.T
        x = np.linalg.pinv(A).dot(p.M.dot(qdd_ref) + p.h)
        print(x)
        return x[:3], x[3:]

    def detectApex(self):
        foot_pos_w = p.base_offset + p.q[:3] + p.x_ee
        # foot tradius is 0.015
        foot_lifted_off = (foot_pos_w[2] > 0.017)
        if not self.detectedApexFlag and foot_lifted_off:
            if (self.qd[2] < 0.0):
                self.detectedApexFlag = True
                #for i in range(10):
                p.pause_physics_client()
                for i in range(10):
                    p.setJumpPlatformPosition(p.target_CoM)
                p.unpause_physics_client()
                p.contactForceW = np.zeros(3)
                print(colored("APEX detected", "red"))
                # not apply initial configuration
                # self.q_des[3:] = self.q_des_q0[3:]

    def detectTouchDown(self):
        # foot_pos_w = p.base_offset + p.q[:3] + p.x_ee
        # if (foot_pos_w[2] <= 0.017 ):
        # print(p.contactForceW)
        contact_force = np.linalg.norm(p.contactForceW)
        if contact_force > 0:
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

        singularity = False
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
        #if np.linalg.norm(self.x_ee) > 0.32:
        if p.q[2]<0.05:
            #self.cost.singularity = 1./(1e-05 + smallest_svalue)
            self.cost.singularity = 100
            singularity = True
            print(colored("Robot has fallen", "red"))

        return singularity

    def evalTotalReward(self, done):
        colored("Evaluating costs", "blue")

        # evaluate final target cost
        self.cost.target = np.linalg.norm(self.com - self.target_CoM)
        target_cost = 1/((50*self.cost.target) + 1e-15)
        target_cost = np.log(1+target_cost)*1000
        # evaluate final com velocity error at lift off cost

        msg = set_rewardRequest()

        if done == 1:
            print(colored("EPISODE DONE", "red"))
            print(colored("Costs: " + self.cost.printCosts(), "green"))
            print(colored("Weighted Costs: " + self.cost.printWeightedCosts(), "green"))

        reward = self.cost.weights[5] * target_cost - (self.cost.weights[0]*self.cost.unilateral +
                                                       self.cost.weights[1]*self.cost.friction +
                                                       self.cost.weights[2] * self.cost.singularity +
                                                       self.cost.weights[3]*self.cost.joint_range +
                                                       self.cost.weights[4] * self.cost.joint_torques)

        # unil  friction sing jointrange torques target
        msg.next_state = np.concatenate((self.q, self.qd,  self.target_CoM))

        msg.reward = reward
        msg.done = done
        msg.target_cost = target_cost
        msg.unilateral = self.cost.unilateral
        msg.friction = self.cost.friction
        msg.singularity = self.cost.singularity
        msg.joint_range = self.cost.joint_range
        msg.joint_torques = self.cost.joint_torques
        self.reward_service(msg)

    def deregister_node(self):
        super().deregister_node()
        os.system(" rosnode kill /"+self.robot_name +
                  "/ros_impedance_controller")
        os.system(" rosnode kill /gzserver /gzclient")
        os.system(" pkill rosmaster")

    def setJumpPlatformPosition(self, target):
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

def talker(p):

    p.start()
    # dont use gazebo
    if p.no_gazebo:
        p.ros_pub = RosPub("jumpleg")
        p.robot = getRobotModel("jumpleg")
    else:
        additional_args=['gui:=false']
        p.startSimulator("jump_platform.world", additional_args=additional_args)
        # p.startSimulator()
        p.loadModelAndPublishers()
        p.startupProcedure()



    # p.loadRLAgent(mode='inference', data_path=os.environ["LOCOSIM_DIR"] + "/robot_control/jumpleg_rl/runs", model_name='latest', restore_train=False)
    # p.loadRLAgent(mode='test', data_path=os.environ["LOCOSIM_DIR"] + "/robot_control/jumpleg_rl/runs", model_name='latest', restore_train=False)
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
        ros.sleep(0.3)
        p.time = 0.
        startTrust = 0.2
        max_episode_time = 2
        p.number_of_episode += 1
        p.freezeBase(True)
        p.firstTime = True
        p.detectedApexFlag = False
        p.trustPhaseFlag = False
        p.intermediate_com_position = []
        p.comd_lo = np.zeros(3)
        p.target_CoM = (p.target_service()).target_CoM
        p.pause_physics_client()
        for i in range(10):
            p.setJumpPlatformPosition(com_0-[0,0,0.2])
        p.unpause_physics_client()
        if p.target_CoM[2] == -1:
            print(colored("# RECIVED STOP TARGET_COM SIGNAL #", "red"))
            break


        # Control loop for one episode
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

                    p.freezeBase(False)

                    print("\n\n")
                    print(colored(f"STARTING A NEW EPISODE--------------------------------------------# :{p.number_of_episode}", "red"))
                    print("Target position from agent:", p.target_CoM)

                    # setting leg pd = 0
                    p.pid.setPDjoint(3, 0., 0., 0.)
                    p.pid.setPDjoint(4, 0., 0., 0.)
                    p.pid.setPDjoint(5, 0., 0., 0.)
                    p.trustPhaseFlag = True

                # Ask for torque value
                state = np.concatenate((p.q, p.qd, p.target_CoM))
                action = p.action_service(state).action
                #print(f"Actor action with torques:\n {action}\n")

                # Apply action
                p.tau_ffwd[3] = np.array(action[0])
                p.tau_ffwd[4] = np.array(action[1])
                p.tau_ffwd[5] = np.array(action[2])

                if p.evaluateRunningCosts():
                    break # robot has fallen

                p.detectApex() # just to set jump platform 
                if (p.detectedApexFlag):
                        # set jump platform (avoid collision in jumping)
                        if p.detectTouchDown():
                            break # END STATE

                # send commands to gazebo
                if p.no_gazebo:
                    p.forwardEulerIntegration(p.tau_ffwd, p.contactForceW)
                    p.ros_pub.publish(p.robot, p.q)

                if not p.no_gazebo:
                    p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

                #send reward with done state = 0
                p.evalTotalReward(0)


            # plot end-effector and contact force
            # if not p.use_ground_truth_contacts:
            #     p.ros_pub.add_arrow(
            #         p.base_offset + p.q[:3] + p.x_ee, p.contactForceW / (10 * p.robot.robot_mass), "green")
            # else:
            #     p.ros_pub.add_arrow(
            #         p.base_offset + p.q[:3] + p.x_ee, p.contactForceW / (10 * p.robot.robot_mass), "red")
            #plot end-effector
            #p.ros_pub.add_marker(p.base_offset + p.q[:3] + p.x_ee, radius=0.05)
            # p.ros_pub.add_cone(
            #     p.base_offset + p.q[:3] + p.x_ee, np.array([0, 0, 1.]), p.mu, height=0.05, color="blue")
            # # p.contactForceW = np.zeros(3) # to be sure it does not retain any "memory" when message are not arriving, so avoid to compute wrong rewards
            #p.ros_pub.add_marker(p.target_CoM, color="blue", radius=0.1)

            # p.ros_pub.add_marker(com_f, color="red", radius=0.1)
            # reachabe space
            # p.ros_pub.add_marker([0, 0, 0], color="green", radius=0.64)

            # p.ros_pub.publishVisual()

            # wait for synconization of the control loop
            rate.sleep()

            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999

        # send reward with state = 1
        p.evalTotalReward(1)

        p.cost.reset()
        plt.cla()


if __name__ == '__main__':
    p = JumpLegController(robotName)

    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
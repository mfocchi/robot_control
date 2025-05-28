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


robotName = "invpend"

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


class SwingController(BaseControllerFixed):

    def __init__(self, robot_name="jumpleg"):
        super().__init__(robot_name=robot_name)
        self.agentMode = 'train'
        self.agentRL = 'PPO'
        self.restoreTrain = False
        self.gui = True
        self.model_name = 'latest'
        self.DEBUG = False
        self.set_state = ros.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        print("JumplegAgent services ready")
        self.action_service = ros.ServiceProxy(
            'JumplegAgent/get_action', get_action)
        self.target_service = ros.ServiceProxy(
            'JumplegAgent/get_target', get_target)
        self.reward_service = ros.ServiceProxy(
            'JumplegAgent/set_reward', set_reward)

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


    def initVars(self):
        super().initVars()
        self.a = np.empty((3, 6))
        self.cost = Cost()

        # unilateral friction singularity joint_range joint_torques error_vel_liftoff error_pos_liftoff unfeasible_vertical_velocity no_touchdown target
        # self.cost.weights = np.array([1000., 0.1, 10., 0.01, 1000., 300., 1000., 10., 10., 1.])
        self.cost.weights = np.array([1000., 0.1, 10., 0.01, 1000., 30., 100., 100., 10., 1.])

        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']

        self.joint_names = conf.robot_params[self.robot_name]['joint_names']




    def resetJoints(self, q):
        # create the message
        req_reset_joints = SetModelConfigurationRequest()
        req_reset_joints.model_name = self.robot_name
        req_reset_joints.urdf_param_name = 'robot_description'
        req_reset_joints.joint_names = self.joint_names
        req_reset_joints.joint_positions = q
        print(f"resetting joints to {q}")
        self.reset_joints(req_reset_joints)


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


    def loadModelAndPublishers(self,  xacro_path = None, additional_urdf_args = None):
        super().loadModelAndPublishers()
        self.reset_joints = ros.ServiceProxy(
            '/gazebo/set_model_configuration', SetModelConfiguration)

    def deregister_node(self):
        super().deregister_node()
        os.system(" rosnode kill /"+self.robot_name +
                  "/ros_impedance_controller")
        os.system(" rosnode kill /gzserver /gzclient")
        os.system(" pkill rosmaster")


def talker(p):

    p.start()
    additional_args = [f'gui:={p.gui}']
    p.startSimulator(additional_args=additional_args)
    p.loadModelAndPublishers()
    p.startupProcedure()
    p.loadRLAgent(mode=p.agentMode, rl=p.agentRL ,data_path=os.environ["LOCOSIM_DIR"] + "/robot_control/jumpleg_rl/runs", model_name=p.model_name, restore_train=p.restoreTrain)
    p.initVars()
    p.q_des = np.copy(p.q_des_q0)

    # loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])
    p.number_of_episode = 0

    # here the RL loop...
    while True:
        # Reset variables
        p.initVars()
        p.q_des = np.copy(p.q_des_q0)
        p.time = 0.
        max_episode_time = 5.0
        p.number_of_episode += 1
        p.firstTime = True

        # state = np.concatenate((com_0, p.target_CoM))
        # action = p.action_service(state).action
        #print("Action from agent:", action)
        # p.T_th = action[0]

        # Control loop
        while not ros.is_shutdown():
            if (p.time > 0.):
                if (p.time > max_episode_time):
                    # max episode time elapsed
                    print(colored("--Episod termination--", "blue"))
                    break
                # release base
                if p.firstTime:
                    p.firstTime = False
                    p.resetJoints(p.q_des_q0 + np.random.uniform(low=-0.1, high=0.1, size=2))

                    print("\n\n")
                    print(colored(f"STARTING A NEW EPISODE--------------------------------------------# :{p.number_of_episode}", "red"))
                    # print(f"Actor action:\n"
                    #       f"T_th: {p.T_th}\n"
                    #       f"com_lo: {com_lo}\n"
                    #       f"comd_lo: {comd_lo}")

            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
            # log variables
            if p.DEBUG:
                p.logData()
            p.ros_pub.publishVisual()

            # wait for synconization of the control loop
            rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 4) # to avoid issues of dt 0.0009999

        # eval rewards
        #p.evalTotalReward(com_lo, comd_lo)
        p.cost.reset()

        plotJoint('position', p.time_log, q_log=p.q_log, q_des_log=p.q_des_log,
                  joint_names=conf.robot_params[p.robot_name]['joint_names'])
        plt.close('all')

if __name__ == '__main__':
    p = SwingController(robotName)

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
            plotJoint('position', p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
        ros.signal_shutdown("killed")
        p.deregister_node()

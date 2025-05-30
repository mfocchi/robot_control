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
        self.target = 0

        self.weights = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])

    def reset(self):

        self.target = 0


    def printCosts(self):
        return  f"target:{self.target} "


    def printWeightedCosts(self):
        return    f"target:{self.weights[9]*self.target} "


class SwingController(BaseControllerFixed):

    def __init__(self, robot_name="jumpleg"):
        super().__init__(robot_name=robot_name)
        self.agentMode = 'train'
        self.agentRL = 'PPO'
        self.restoreTrain = False
        self.gui = False
        self.model_name = 'latest'
        self.DEBUG = False

        self.set_state = ros.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        print("JumplegAgentInstantPos services ready")
        self.action_service = ros.ServiceProxy(
            'SwingupAgent/get_action', get_action)
        self.target_service = ros.ServiceProxy(
            'SwingupAgent/get_target', get_target)
        self.reward_service = ros.ServiceProxy(
            'SwingupAgent/set_reward', set_reward_original)

        np.random.seed(136)
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
        self.cost = Cost()

        # unilateral friction singularity joint_range joint_torques error_vel_liftoff error_pos_liftoff unfeasible_vertical_velocity no_touchdown target
        # self.cost.weights = np.array([1000., 0.1, 10., 0.01, 1000., 300., 1000., 10., 10., 1.])
        self.cost.weights = np.array([1000., 0.1, 10., 0.01, 1000., 30., 100., 100., 10., 1.])
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']
        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        self.state = np.zeros(4)
        self.action = np.zeros(1)

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
        executable = f'SwingupAgent_{rl}.py'
        name = 'rlagent'
        namespace = '/'
        args = f'--mode {mode} --data_path {data_path}_{rl} --model_name {model_name} --restore_train {restore_train}'
        node = roslaunch.core.Node(
            package, executable, name, namespace, args=args, output="screen")
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        process = self.launch.launch(node)

        # wait for agent service to start
        print("Waiting for SwingupAgent services")
        ros.wait_for_service('SwingupAgent/get_action')
        ros.wait_for_service('SwingupAgent/get_target')
        ros.wait_for_service('SwingupAgent/set_reward')

    def computeActivationFunction(self, activationType, value, lower, upper):

        if (activationType == 'linear'):
            return abs(min(value - lower, 0) + max(value-upper, 0))

        if (activationType == 'quadratic'):
            return pow(min(value - lower, 0), 2)/2.0 + pow(max(value-upper, 0), 2)/2.0

    def evalReward(self, q, qd, done = False):
        #https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_tasks/isaaclab_tasks/direct/cartpole/cartpole_env.py
        colored("Evaluate reward", "blue")
        next_state = np.concatenate((q, qd))
        msg = set_reward_originalRequest()
        if done == True:
            print(colored("EPISODE DONE", "red"))
            print(colored("Costs: " + self.cost.printCosts(), "green"))
            reward = 0
        if done == False:
            target_up = np.array([0, 0, 0, 0])
            reward = 1e03 * (1. - 0.1 * pow(np.linalg.norm(next_state - target_up), 2))
        msg.reward = reward
        msg.next_state = next_state
        msg.state = self.state
        msg.action = self.action
        msg.done = done
        self.reward_service(msg)

        #update the state memory with the atcual state
        self.state = next_state

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
    p.startSimulator(additional_args=additional_args, launch_file='standard')
    p.loadModelAndPublishers()
    p.startupProcedure()
    p.pid.setPDjoints(np.zeros(p.robot.na), np.zeros(p.robot.na), np.zeros(p.robot.na))
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
        # Control loop
        while not ros.is_shutdown():
            # release base
            if p.firstTime:
                p.firstTime = False
                p.training_region_size = 1. #rad
                p.resetJoints(p.q_des_q0 + np.random.uniform(low=-p.training_region_size, high=p.training_region_size, size=2))
                print("\n\n")
                print(colored(f"STARTING A NEW EPISODE--------------------------------------------# :{p.number_of_episode}", "red"))

            # eval rewards from previous and update state
            p.evalReward(p.q, p.qd)
            p.action = p.action_service(p.state).action

            #cart_force = p.action[0]
            #print("Action from agent:", p.action)

            #episode termination
            # print("q",p.q.T)
            # print("qd", p.qd.T)
            episode_termination = (math.fabs(p.q[1]) > 2) or (math.fabs(p.q[1]) > np.pi/4) or (p.time >10.)
            if (episode_termination):
                # max episode time elapsed
                print(colored("--Episod termination--", "blue"))
                p.evalReward(p.q, p.qd, True)
                break

            #p.tau_ffwd[1] = cart_force
            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)


            # log variables
            if p.DEBUG:
                p.logData()
            p.ros_pub.publishVisual()

            # wait for synconization of the control loop
            rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 4) # to avoid issues of dt 0.0009999
            #p.cost.reset()

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

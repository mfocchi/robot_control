# -*- coding: utf-8 -*-
"""
Created on 3 May  2022

@author: mfocchi
"""

from __future__ import print_function
import os

import numpy as np
import rospy as ros
import threading

# messages for topic subscribers
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyRequest
from termcolor import colored

#gazebo messages
from gazebo_msgs.srv import SetModelState
#gazebo services
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsPropertiesRequest
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest

# ros utils
import roslaunch
import rospkg

#other utils
from base_controllers.utils.ros_publish import RosPub
from base_controllers.utils.math_tools import *
from numpy import nan
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
from termcolor import colored
import matplotlib.pyplot as plt
import distro
import rosgraph
import rosnode
import sys

import  base_controllers.params as conf
# robots can be ur5 and jumpleg to load ur5 you need to set this xacro path in loadModelAndPublishers
robotName = "mir"

from base_controllers.components.inverse_kinematics.inv_kinematics_pinocchio import robotKinematics
from base_controllers.utils.math_tools import Math
from gazebo_msgs.srv import ApplyBodyWrench
from base_controllers.utils.common_functions import plotJoint, plotFrame

from datetime import datetime
import os
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt

from  base_controllers.doretta.utils import constants as constants
from  base_controllers.doretta.environment.trajectory import Trajectory, ModelsList
from  base_controllers.doretta.models.unicycle import Unicycle
from  base_controllers.doretta.controllers.lyapunov import LyapunovController, LyapunovParams
import base_controllers.doretta.velocity_tests as vt

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class BaseControllerMobile(threading.Thread):
    """
        This Class can be used to simulate floating base robots that
        have an under-actuated base (e.g. like quadrupeds, mobile robots)

        ...

        Methods
        -------
        loadModelAndPublishers(xacro_path=None)
           loads publishers for visual features (ros_pub), joint commands and declares subscriber to /ground_truth and /joint_states
        startSimulator(world_name = None)
           Starts gazebo simulator with ros_impedance_controller
        send_des_jstate(self, q_des, qd_des, tau_ffwd)
            publishes /command topic with set-points for joint positions, velocities and feed-forward torques
        startupProcedure():
            initialize PD gains
        initVars()
            initializes class variables
        logData()
            fill in the X_log variables for plotting purposes, it needs to be called at every loop
        _receive_jstate(msg)
            callback associated to the joint state subscriber, fills in q, qd, tau arrays

    """
    def __init__(self, robot_name="mir"):
        threading.Thread.__init__(self)
        self.robot_name = robot_name

        self.u = Utils()
        self.math_utils = Math()

        #send data to param server
        self.verbose = conf.verbose
        K_P = 6.0
        K_THETA = 6.0

        print("Initialized fixed basecontroller---------------------------------------------------------------")

    def startSimulator(self, world_name = None, additional_args=None):
        # needed to be able to load a custom world file
        print(colored('Adding gazebo model path!', 'blue'))
        custom_models_path = rospkg.RosPack().get_path('ros_impedance_controller')+"/worlds/models/"
        if os.getenv("GAZEBO_MODEL_PATH") is not None:
            os.environ["GAZEBO_MODEL_PATH"] +=":"+custom_models_path
        else:
            os.environ["GAZEBO_MODEL_PATH"] = custom_models_path

        launch_file = rospkg.RosPack().get_path('mir_gazebo') + '/launch/mir_empty_world.launch'

        # clean up previous process
        os.system("killall rosmaster rviz gzserver gzclient")
        if (distro.linux_distribution()[1] == "16.04"):
            print(colored("This file only works with distribution from ROS lunar (I.e. ubuntu 17.04 or later) ", "red"))
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [launch_file]
        if additional_args is not None:
            cli_args.extend(additional_args)
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        ros.sleep(1.0)
        print(colored('SIMULATION Started', 'blue'))

    def loadModelAndPublishers(self,  xacro_path = None, additional_urdf_args = None):

        # instantiating objects
        self.ros_pub = RosPub(self.robot_name, only_visual=True)
        self.pub_des_jstate = ros.Publisher("/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)
        # freeze base  and pause simulation service
        self.reset_world = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_physics_client = ros.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.get_physics_client = ros.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        self.pause_physics_client = ros.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = ros.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.reset_joints_client = ros.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        self.u.putIntoGlobalParamServer("verbose", self.verbose)
        # subscribers
        self.sub_jstate = ros.Subscriber("/joint_states", JointState,
                                         callback=self._receive_jstate, queue_size=1,buff_size = 2 ** 24, tcp_nodelay=True)
        self.sub_pose = ros.Subscriber("/odom", Odometry, callback=self._receive_pose,
                                       queue_size=1, tcp_nodelay=True)
    def _receive_jstate(self, msg):
         for msg_idx in range(len(msg.name)):
             for joint_idx in range(len(self.joint_names)):
                 if self.joint_names[joint_idx] == msg.name[msg_idx]:
                     self.q[joint_idx] = msg.position[msg_idx]
                     self.qd[joint_idx] = msg.velocity[msg_idx]
                     self.tau[joint_idx] = msg.effort[msg_idx]

    def _receive_pose(self, msg):

        self.quaternion[0] = msg.pose.pose.orientation.x
        self.quaternion[1] = msg.pose.pose.orientation.y
        self.quaternion[2] = msg.pose.pose.orientation.z
        self.quaternion[3] = msg.pose.pose.orientation.w

        self.euler = np.array(euler_from_quaternion(self.quaternion))

        self.basePoseW[self.u.sp_crd["LX"]] = msg.pose.pose.position.x
        self.basePoseW[self.u.sp_crd["LY"]] = msg.pose.pose.position.y
        self.basePoseW[self.u.sp_crd["LZ"]] = msg.pose.pose.position.z
        self.basePoseW[self.u.sp_crd["AX"]] = self.euler[0]
        self.basePoseW[self.u.sp_crd["AY"]] = self.euler[1]
        self.basePoseW[self.u.sp_crd["AZ"]] = self.euler[2]

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


    def send_des_command(self, vx, vy, omega):
         # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
         msg = Twist()
         msg.linear.x = vx
         msg.linear.y = vy
         msg.angular.z = omega
         self.pub_des_jstate.publish(msg)  
                
    def deregister_node(self):
        print( "deregistering nodes"     )
        self.ros_pub.deregister_node()

    def initVars(self):
        self.n_joints = 2
        self.q = np.zeros(self.n_joints)
        self.qd = np.zeros(self.n_joints)
        self.tau = np.zeros(self.n_joints)
        self.q_des =np.zeros(self.n_joints)
        self.qd_des = np.zeros(self.n_joints)
        self.tau = np.zeros(self.n_joints)
        self.tau_ffwd =np.zeros(self.n_joints)
        self.time  = 0.
        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        self.basePoseW = np.zeros(6)
        self.baseTwistW = np.zeros(6)
        self.euler = np.zeros(3)

        #log vars
        self.q_des_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.q_log = np.empty((self.n_joints,conf.robot_params[self.robot_name]['buffer_size'] )) *nan
        self.qd_des_log = np.empty((self.n_joints,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.qd_log = np.empty((self.n_joints,conf.robot_params[self.robot_name]['buffer_size'] )) *nan
        self.tau_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.tau_ffwd_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.time_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))*0.
        self.basePoseW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)

        self.log_counter = 0
        self.log_counter = 0


    def logData(self):
        if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
            self.q_des_log[:, self.log_counter] = self.q_des
            self.q_log[:,self.log_counter] =  self.q
            self.qd_des_log[:,self.log_counter] =  self.qd_des
            self.qd_log[:,self.log_counter] = self.qd
            self.tau_log[:,self.log_counter] = self.tau
            self.tau_ffwd_log[:, self.log_counter] = self.tau_ffwd
            self.basePoseW_log[:, self.log_counter] = self.basePoseW

            self.time_log[self.log_counter] = self.time
            self.log_counter+=1

    def reset_joints(self, q0, joint_names = None):
        # create the message
        req_reset_joints = SetModelConfigurationRequest()
        req_reset_joints.model_name = self.robot_name
        req_reset_joints.urdf_param_name = 'robot_description'
        if joint_names == None:
            req_reset_joints.joint_names = self.joint_names
        else:
            req_reset_joints.joint_names = joint_names
        req_reset_joints.joint_positions = q0
        self.reset_joints_client(req_reset_joints)
        print(colored(f"---------Resetting Joints to: "+str(q0), "blue"))


def talker(p):
    p.start()
    p.startSimulator()
    p.loadModelAndPublishers()
    p.initVars()
    #p.unpause_physics_client()
    # p.startupProcedure()

    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])
    # define traj
    time_ref = np.linspace(0., 40., int(40./conf.robot_params[p.robot_name]['dt']))
    omegar_ref = 0.5
    x_ref = 1 + np.cos(omegar_ref *time_ref)
    y_ref = np.sin(omegar_ref * time_ref)
    theta_ref = omegar_ref*time_ref
    ros.sleep(1.)

    vel_gen = vt.velocity_test2
    _, _, s_test = vel_gen()

    class robot:
        pass
    robot.x = p.basePoseW[p.u.sp_crd["LX"]]
    robot.y = p.basePoseW[p.u.sp_crd["LY"]]
    robot.theta = p.euler[2]
    print(f"Initial pos X: {robot.x} Y: {robot.y} th: {robot.theta}")
    #robot = Unicycle(x=robot.x, y=robot.y, theta=robot.theta)
    traj = Trajectory(ModelsList.UNICYCLE, robot.x, robot.y, robot.theta, vel_gen)
    # Lyapunov controller parameters
    K_P = 6.0
    K_THETA = 6.0
    params = LyapunovParams(K_P=K_P, K_THETA=K_THETA)
    controller = LyapunovController(params=params)
    controller.config(start_x=robot.x, start_y=robot.y, start_theta=robot.theta, start_time=p.time,
                      velocity_generator=vel_gen)


    #control loop
    while not ros.is_shutdown():
        # controllers
        v, omega = controller.control(robot,p.time)
        # b = 0.5
        # r = .05
        # v_l = v - omega *b/2
        # v_r = v + omega * b / 2

        #
        # # SAFE CHECK -> clipping velocities
        # v = np.clip(v, -constants.MAX_LINEAR_VELOCITY, constants.MAX_LINEAR_VELOCITY)
        # o = np.clip(o, -constants.MAX_ANGULAR_VELOCITY, constants.MAX_ANGULAR_VELOCITY)

        # send commands to gazebo
        p.send_des_command(v, 0, omega)
        # log variables
        p.logData()

        #wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999

if __name__ == '__main__':
    p = BaseControllerMobile(robotName)

    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        if    conf.plotting:
            pass
            plotJoint('position', time_log=p.time_log, q_des_log=p.q_des_log, q_log=p.q_log, joint_names=p.joint_names)
            plotFrame('position', time_log=p.time_log,  Pose_log=p.basePoseW_log,
                      title='CoM', frame='W', sharex=True)

        

# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

from __future__ import print_function
import rospy as ros
from base_controllers.utils.math_tools import *
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
from base_controllers.base_controller import BaseController
from base_controllers.utils.common_functions import plotFrameLinear, plotFrame,  plotJoint, sendStaticTransform, launchFileGeneric
import params as conf
import os
import sys
import rospkg
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest
from numpy import nan
from matplotlib import pyplot as plt
from base_controllers.utils.math_tools import unwrap_angle
from  base_controllers.tracked_robot.utils import limo_constants as constants
from base_controllers.tracked_robot.controllers.lyapunov import LyapunovController, LyapunovParams, Robot
from  base_controllers.tracked_robot.environment.trajectory import Trajectory, ModelsList
from base_controllers.tracked_robot.velocity_generator import VelocityGenerator
from termcolor import colored
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
import catboost as cb
import scipy.io.matlab as mio
from base_controllers.utils.common_functions import spawnModel, launchFileNode
from base_controllers.tracked_robot.regressor.NN.inference_nn import SlipNN
import pandas as pd
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelStateRequest
from geometry_msgs.msg import Twist
from base_controllers.utils.communication_utils import getInitialStateFromOdom, getInitialStateFromJoints
robotName = "limo0" # needs to inherit BaseController

class GenericSimulator(BaseController):
    def __init__(self, robot_name="limo0"):
        super().__init__(robot_name=robot_name, external_conf = conf)
        self.torque_control = False
        print("Initialized limo controller---------------------------------------------------------------")

        self.ControlType = 'CLOSED_LOOP_UNICYCLE' #'OPEN_LOOP' 'CLOSED_LOOP_UNICYCLE' 'CLOSED_LOOP_SLIP_0' 'CLOSED_LOOP_SLIP'
        self.SIDE_SLIP_COMPENSATION = 'MACHINE_LEARNING' # 'MACHINE_LEARNING', 'NONE', 'EXP(not used)'
        self.LONG_SLIP_COMPENSATION = 'MACHINE_LEARNING' # 'MACHINE_LEARNING', 'NONE', 'EXP(not used)'
        self.SLIPPAGE_INFERENCE_TYPE = 'decision_trees'  # 'decision_trees','interpolator' , 'NN'
        self.ESTIMATE_ALPHA_WITH_ACTUAL_VALUES = True # makes difference for v >= 0.4

        self.SENSORS = 'false' #'true',  'false'
        # Parameters for open loop identification
        self.IDENT_TYPE = 'NONE' # 'V_OMEGA(deprecated)', 'WHEELS', 'NONE'
        self.IDENT_LONG_SPEED = 0.3  #used only when IDENT_TYPE = 'V_OMEGA' (deprecated)
        self.IDENT_DIRECTION = 'left' #used only when IDENT_TYPE = 'V_OMEGA' (deprecated)

        self.friction_coefficient = 0.4 # 0.1 (used only in 2d) / 0.4 (2d and 3d) (used for planning in paper)/ 0.6 (only 3d)  with slopes we need high friction otherwise alpha is too high
         # initial pose (sim)
        self.p0 = np.array([0., 0., 0.])

        self.SAVE_BAGS = False
        # to avoid issues with contacts
        self.use_ground_truth_contacts = False
        # to avoid redundant TF on baselink (world -> baselink)
        self.broadcast_world = False

    def initVars(self):
        super().initVars()
        # load model
        try:
            if self.SLIPPAGE_INFERENCE_TYPE=='decision_trees':
                # regressor
                self.regressor_beta_l = cb.CatBoostRegressor()
                self.regressor_beta_r = cb.CatBoostRegressor()
                self.regressor_alpha = cb.CatBoostRegressor()
                self.model_beta_l = self.regressor_beta_l.load_model(os.environ['LOCOSIM_DIR']+'/robot_control/base_controllers/tracked_robot/regressor/limo/model_beta_l'+str(self.friction_coefficient)+'.cb')
                self.model_beta_r = self.regressor_beta_r.load_model(os.environ['LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/regressor/limo/model_beta_r'+str(self.friction_coefficient)+'.cb')
                self.model_alpha = self.regressor_alpha.load_model(os.environ['LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/regressor/limo/model_alpha'+str(self.friction_coefficient)+'.cb')
            elif  self.SLIPPAGE_INFERENCE_TYPE=='NN':
                self.model_beta_l = SlipNN(output='beta_l')
                self.model_beta_r = SlipNN(output='beta_r')
                self.model_alpha = SlipNN(output='alpha')
            elif self.SLIPPAGE_INFERENCE_TYPE=='interpolator':
                from scipy.interpolate import RBFInterpolator
                data = os.environ['LOCOSIM_DIR']+f'/robot_control/base_controllers/tracked_robot/regressor/limo/ident_wheels_sim_2d_'+str(self.friction_coefficient)+'.csv'
                df = pd.read_csv(data, skiprows=1, names=['wheel_l', 'wheel_r', 'beta_l', 'beta_r', 'alpha']) #skiprows skips the first row which are the labels
                x = df[['wheel_l', 'wheel_r']].values
                y = df[['beta_l', 'beta_r', 'alpha']].values
                # upsampling
                # Fit an interpolator for each output dimension
                self.model_beta_l = RBFInterpolator(x, y[:, 0], smoothing=0.1)
                self.model_beta_r = RBFInterpolator(x, y[:, 1], smoothing=0.1)
                self.model_alpha = RBFInterpolator(x, y[:, 2], smoothing=0.1)
        except Exception as e:
            print(colored(f"Error initializing slippage inference model:{e}","red"))
            self.model_beta_l = None
            self.model_beta_r = None
            self.model_alpha = None
            print(colored(f"No Machine Learning  model for need for friction coefficient {self.friction_coefficient}, you need to generate the models by running tracked_robot/regressor/model_slippage_updated.py","red"))
        ## add your variables to initialize here
        self.ctrl_v = 0.
        self.ctrl_omega = 0.0
        self.v_d = 0.
        self.omega_d = 0.
        self.V= 0.
        self.V_dot = 0.

        self.q_des_q0 = np.zeros(self.robot.na)
        self.ctrl_v_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.ctrl_omega_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.v_d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.omega_d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.V_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.V_dot_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))* nan
        self.des_x = 0.
        self.des_y = 0.
        self.des_theta = 0.
        self.beta_l= 0.
        self.beta_r= 0.
        self.alpha= 0.
        self.alpha_control= 0.
        self.radius = 0.
        self.beta_l_control = 0.
        self.beta_r_control = 0.
        self.log_exy = []
        self.log_e_theta = []
        self.euler = np.zeros(3)
        self.basePoseW_des = np.zeros(6) * np.nan
        self.b_base_vel = np.zeros(2)

        self.state_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.des_state_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.basePoseW_des_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)
        self.b_base_vel_log = np.full((2, conf.robot_params[self.robot_name]['buffer_size']),  np.nan)

        self.beta_l_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.beta_r_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.alpha_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.alpha_control_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.radius_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.beta_l_control_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.beta_r_control_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.out_of_frequency_counter=0

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

    def setModelState(self, model_name='', position=np.zeros(3), orientation=np.array([0,0,0,1])):
        # create the message
        set_position = SetModelStateRequest()
        # create model state
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose.position.x = position[0]
        model_state.pose.position.y = position[1]
        model_state.pose.position.z = position[2]
        model_state.pose.orientation.x = orientation[0]
        model_state.pose.orientation.y = orientation[1]
        model_state.pose.orientation.z = orientation[2]
        model_state.pose.orientation.w = orientation[3]
        set_position.model_state = model_state
        # send request and get response (in this case none)
        self.set_state(set_position)

    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                ## add your logs here
                self.ctrl_v_log[self.log_counter] = self.ctrl_v
                self.ctrl_omega_log[self.log_counter] = self.ctrl_omega
                self.v_d_log[self.log_counter] = self.v_d
                self.omega_d_log[self.log_counter] = self.omega_d
                self.V_log[self.log_counter] = self.V
                self.V_dot_log[self.log_counter] = self.V_dot
                self.des_state_log[0, self.log_counter] = self.des_x
                self.des_state_log[1, self.log_counter] = self.des_y
                self.des_state_log[2, self.log_counter] = self.des_theta
                self.state_log[0, self.log_counter] = self.basePoseW[self.u.sp_crd["LX"]]
                self.state_log[1, self.log_counter] = self.basePoseW[self.u.sp_crd["LY"]]
                self.state_log[2, self.log_counter] =  self.basePoseW[self.u.sp_crd["AZ"]]

                self.basePoseW_des_log[:, self.log_counter] = self.basePoseW_des #basepose is logged in base controller
                self.b_base_vel_log[:, self.log_counter] = self.b_base_vel  # basepose is logged in base controller

                self.alpha_log[self.log_counter] = self.alpha
                self.beta_l_log[self.log_counter] = self.beta_l
                self.beta_r_log[self.log_counter] = self.beta_r

                self.alpha_control_log[self.log_counter] = self.alpha_control
                self.beta_l_control_log[self.log_counter] = self.beta_l_control
                self.beta_r_control_log[self.log_counter] = self.beta_r_control
                self.radius_log[self.log_counter] = self.radius
            super().logData()

    def startFramework(self):
        self.decimate_publish = 1
        world_name = None #'ramps.world'
        additional_args = ['spawn_x:=' + str(p.p0[0]),'spawn_y:=' + str(p.p0[1]),'spawn_Y:=' + str(p.p0[2]),'sensors:='+self.SENSORS]
        launch_file = rospkg.RosPack().get_path('limo_description') + '/launch/start_locosim.launch'
        super().startSimulator(world_name=world_name, launch_file=launch_file, additional_args=additional_args)

    def loadModelAndPublishers(self):
        super().loadModelAndPublishers()
        self.reset_joints_client = ros.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.des_vel_pub = ros.Publisher("/des_vel", JointState, queue_size=1, tcp_nodelay=True)
        self.cmd_vel_pub = ros.Publisher("/"+self.robot_name+"/cmd_vel", Twist, queue_size=1, tcp_nodelay=True)

        if self.SAVE_BAGS:
            if p.ControlType=='OPEN_LOOP':
                if p.IDENT_TYPE=='V_OMEGA':
                    bag_name= f"ident_sim_longv_{p.IDENT_LONG_SPEED}_{p.IDENT_DIRECTION}_fr_{p.friction_coefficient}.bag"
                if p.IDENT_TYPE == 'WHEELS':
                   bag_name = f"ident_sim_fr_{p.friction_coefficient}_wheelL_{p.IDENT_WHEEL_L}.bag"
            else:
                bag_name = f"{p.ControlType}_Long_{self.LONG_SLIP_COMPENSATION}_Side_{p.SIDE_SLIP_COMPENSATION}.bag"
            self.recorder = RosbagControlledRecorder(bag_name=bag_name)

    def _receive_pose(self, msg):
        self.quaternion = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        self.euler = np.array(euler_from_quaternion(self.quaternion))
        #unwrap
        self.euler, self.euler_old = unwrap_vector(self.euler, self.euler_old)

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

    def checkLoopFrequency(self):
        # check frequency of publishing
        if hasattr(self, 'check_time'):
            loop_time = ros.Time.now().to_sec() - self.check_time  # actual publishing time interval
            ros_loop_time = self.slow_down_factor * conf.robot_params[p.robot_name]['dt'] * self.decimate_publish  # ideal publishing time interval
            if loop_time > 1.3 * (ros_loop_time):
                loop_real_freq = 1 / loop_time  # actual publishing frequency
                freq_ros = 1 / ros_loop_time  # ideal publishing frequency
                print(colored(f"freq mismatch beyond 30%: loop is running at {loop_real_freq} Hz while it should run at {freq_ros} Hz, freq error is {(freq_ros - loop_real_freq) / freq_ros * 100} %", "red"))
                self.out_of_frequency_counter += 1
                if self.out_of_frequency_counter > 10:
                    original_slow_down_factor = self.slow_down_factor
                    self.slow_down_factor *= 2
                    self.rate = ros.Rate(1 / (self.slow_down_factor * conf.robot_params[p.robot_name]['dt']))
                    print(colored(f"increasing slow_down_factor from {original_slow_down_factor} to {self.slow_down_factor}", "red"))
                    self.out_of_frequency_counter = 0

        self.check_time = ros.Time.now().to_sec()

    def deregister_node(self):
        os.system("killall rosmaster gzserver gzclient rviz coppeliaSim")
        super().deregister_node()

    def startupProcedure(self):
        self.slow_down_factor = 1
        # loop frequency
        self.rate = ros.Rate(1 / (self.slow_down_factor * conf.robot_params[p.robot_name]['dt']))

    def plotData(self):
        if conf.plotting:
            #xy plot
            plt.figure()
            plt.plot(p.des_state_log[0, :], p.des_state_log[1, :], "-r", label="desired")
            plt.plot(p.state_log[0, :], p.state_log[1, :], "-b", label="real")
            plt.legend()
            plt.title(f"Control: {p.ControlType}, Long: {p.LONG_SLIP_COMPENSATION} Side: {p.SIDE_SLIP_COMPENSATION}")
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            # # command plot
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(p.time_log, p.ctrl_v_log, "-b", label="REAL")
            plt.plot(p.time_log, p.v_d_log, "-r", label="desired")
            plt.legend()
            plt.title("v and omega")
            plt.ylabel("linear velocity[m/s]")
            plt.grid(True)
            plt.subplot(2, 1, 2)
            plt.plot(p.time_log, p.ctrl_omega_log, "-b", label="REAL")
            plt.plot(p.time_log, p.omega_d_log, "-r", label="desired")
            plt.legend()
            plt.xlabel("time[sec]")
            plt.ylabel("angular velocity[rad/s]")
            plt.grid(True)

            #plotJoint('position', p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, joint_names=p.joint_names)
            #joint velocities with limits
            fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 8))  # Create all 3 subplots at once

            axs[0].plot(p.time_log, p.qd_log[0, :], "-b", linewidth=3)
            axs[0].plot(p.time_log, p.qd_des_log[0, :], "-r", linewidth=4)
            axs[0].plot(p.time_log, constants.MAXSPEED_RADS_PULLEY * np.ones(len(p.time_log)), "-k", linewidth=4)
            axs[0].plot(p.time_log, -constants.MAXSPEED_RADS_PULLEY * np.ones(len(p.time_log)), "-k", linewidth=4)
            axs[0].set_ylabel("WHEEL_L")
            axs[0].grid(True)

            axs[1].plot(p.time_log, p.qd_log[1, :], "-b", linewidth=3)
            axs[1].plot(p.time_log, p.qd_des_log[1, :], "-r", linewidth=4)
            axs[1].plot(p.time_log, constants.MAXSPEED_RADS_PULLEY * np.ones(len(p.time_log)), "-k", linewidth=4)
            axs[1].plot(p.time_log, -constants.MAXSPEED_RADS_PULLEY * np.ones(len(p.time_log)), "-k", linewidth=4)
            axs[1].set_ylabel("WHEEL_R")
            axs[1].grid(True)

            axs[2].plot(p.time_log, p.alpha_control_log, "-r", linewidth=4)
            axs[2].set_ylabel("alpha")
            axs[2].grid(True)

            plt.xlabel("Time [s]")
            plt.tight_layout()
            plt.show()

            #states plot
            plotFrameLinear(name='position',time_log=p.time_log,des_Pose_log = p.des_state_log, Pose_log=p.state_log, custom_labels=(["X","Y","THETA"]))

            #plot velocities in the base frame
            plt.figure()
            ax1 = plt.subplot(2, 1, 1)
            plt.plot(self.time_log, self.b_base_vel_log[0, :], "-b", label="vx")
            plt.ylabel("b_vx")
            plt.legend()
            plt.grid(True)
            plt.subplot(2, 1, 2, sharex=ax1)
            plt.plot(self.time_log, self.b_base_vel_log[1, :], "-b", label="vy")
            plt.ylabel("b_vy")
            plt.legend()
            plt.grid(True)

            #slippage vars
            plt.figure()
            ax2 = plt.subplot(4, 1, 1)
            plt.plot(self.time_log, self.beta_l_log, "-b", label="real")
            plt.plot(self.time_log, self.beta_l_control_log, "-r", label="control")
            plt.ylabel("beta_l")
            plt.legend()
            plt.grid(True)
            plt.subplot(4, 1, 2,  sharex=ax2)
            plt.plot(self.time_log, self.beta_r_log, "-b", label="real")
            plt.plot(self.time_log, self.beta_r_control_log, "-r", label="control")
            plt.ylabel("beta_r")
            plt.legend()
            plt.grid(True)
            plt.subplot(4, 1, 3,  sharex=ax2)
            plt.plot(self.time_log, self.alpha_log, "-b", label="real")
            plt.plot(self.time_log, self.alpha_control_log, "-r", label="control")
            plt.ylabel("alpha")
            #plt.ylim([-0.4, 0.4])
            plt.grid(True)
            plt.legend()
            plt.subplot(4, 1, 4,  sharex=ax2)
            plt.plot(self.time_log, self.radius_log, "-b")
            plt.ylim([-1,1])
            plt.ylabel("radius")
            plt.grid(True)

            if p.ControlType != 'OPEN_LOOP':
                # tracking errors
                p.log_e_x, p.log_e_y, p.log_e_theta = p.controller.getErrors()
                plt.figure()
                plt.subplot(2, 1, 1)
                plt.plot(np.sqrt(np.power(self.log_e_x,2) +np.power(self.log_e_y,2)), "-b")
                plt.ylabel("exy")
                plt.title("tracking errors")
                plt.grid(True)
                plt.subplot(2, 1, 2)
                plt.plot(self.log_e_theta, "-b")
                plt.ylabel("eth")
                plt.grid(True)

    def mapFromWheels(self, wheel_l, wheel_r):
        if not np.isscalar(wheel_l):
            v = np.zeros_like(wheel_l)
            omega = np.zeros_like(wheel_l)
            for i in range(len(wheel_l)):
                v[i] = constants.SPROCKET_RADIUS*(wheel_l[i] + wheel_r[i])/2
                omega[i] = constants.SPROCKET_RADIUS/constants.TRACK_WIDTH*(wheel_r[i] -wheel_l[i])
            return v, omega


    def publishControlCommand(self, v_des,omega_des):
        qd_des = np.zeros(4)
        qd_des[0] = (v_des - omega_des * constants.TRACK_WIDTH / 2)/constants.SPROCKET_RADIUS  # left front
        qd_des[1] = (v_des + omega_des * constants.TRACK_WIDTH / 2)/constants.SPROCKET_RADIUS  # right front
        qd_des[2] = qd_des[0].copy()
        qd_des[3] = qd_des[1].copy()
        #publish des commands as well
        msg = JointState()
        msg.name = self.joint_names
        msg.header.stamp = ros.Time.from_sec(self.time)
        msg.velocity = np.array([v_des, omega_des])
        self.des_vel_pub.publish(msg)

        msg = Twist()
        msg.linear.x = v_des
        msg.angular.z = omega_des
        self.cmd_vel_pub.publish(msg)
        return qd_des

    #unwrap the joints states
    def unwrap(self):
        for i in range(self.robot.na):
            self.q[i], self.q_old[i] =unwrap_angle(self.q[i], self.q_old[i])

    def generateWheelTraj(self, wheel_l = -4.5):
        ####################################
        # OPEN LOOP wl , wr (from -IDENT_MAX_WHEEL_SPEED to IDENT_MAX_WHEEL_SPEED)
        ####################################
        wheel_l_vec = []
        wheel_r_vec = []
        change_interval = 2.
        if wheel_l <= 0.: #this is to make such that the ID starts always with no rotational speed
            wheel_r = np.linspace(-self.IDENT_MAX_WHEEL_SPEED, self.IDENT_MAX_WHEEL_SPEED, 32) #it if passes from 0 for some reason there is a non linear
                #behaviour in the long slippage
        else:
            wheel_r =np.linspace(self.IDENT_MAX_WHEEL_SPEED, -self.IDENT_MAX_WHEEL_SPEED, 32)
        time = 0
        i = 0
        while True:
            time = np.round(time + conf.robot_params[p.robot_name]['dt'], 4)
            wheel_l_vec.append(wheel_l)
            wheel_r_vec.append(wheel_r[i])
            # detect_switch = not(round(math.fmod(time,change_interval),3) >0)
            if time > ((1 + i) * change_interval):
                i += 1
            if i == len(wheel_r):
                break
        wheel_l_vec.append(0.0)
        wheel_r_vec.append(0.0)
        return wheel_l_vec,wheel_r_vec

    def generateOpenLoopTraj(self, R_initial= 0.05, R_final=0.6, increment=0.025, dt = 0.005, long_v = 0.1, direction="left"):
        # only around 0.3
        change_interval = 3.
        increment = increment
        turning_radius_vec = np.arange(R_final, R_initial, -increment)
        if direction=='left':
            ang_w = np.round(long_v / turning_radius_vec, 3)  # [rad/s]
        else:
            ang_w = -np.round(long_v / turning_radius_vec, 3)  # [rad/s]
        omega_vec = []
        v_vec = []
        time = 0
        i = 0
        while True:
            time = np.round(time + dt, 3)
            omega_vec.append(ang_w[i])
            v_vec.append(long_v)
            # detect_switch = not(round(math.fmod(time,change_interval),3) >0)
            if time > ((1 + i) * change_interval):
                i += 1
            if i == len(turning_radius_vec):
                break
        v_vec.append(0.0)
        omega_vec.append(0.0)
        return v_vec, omega_vec

    def estimateSlippages(self,W_baseTwist, theta, qd):
        wheel_L = qd[0]
        wheel_R = qd[1]
        w_vel_xy = np.zeros(2)
        w_vel_xy[0] = W_baseTwist[self.u.sp_crd["LX"]]
        w_vel_xy[1] = W_baseTwist[self.u.sp_crd["LY"]]
        omega = W_baseTwist[self.u.sp_crd["AZ"]]
        # compute BF velocity
        w_R_b = np.array([[np.cos(theta), -np.sin(theta)],
                          [np.sin(theta), np.cos(theta)]])
        b_vel_xy = (w_R_b.T).dot(w_vel_xy)

        b_vel_x = b_vel_xy[0]
        v = np.linalg.norm(b_vel_xy)

        # compute turning radius for logging
        # in the case radius is infinite, betas are zero (this is to avoid Nans)
        if (abs(omega) < 1e-05) and (abs(v) > 1e-05):
            radius = 1e08 * np.sign(v)
        elif (abs(omega) < 1e-05) and (abs(v) < 1e-05):
            radius = 1e8
        else:
            radius = v / (omega)

        # track velocity  from encoder
        v_enc_l = constants.SPROCKET_RADIUS *  wheel_L
        v_enc_r = constants.SPROCKET_RADIUS *  wheel_R
        B = constants.TRACK_WIDTH

        v_track_l = b_vel_x - omega* B / 2
        v_track_r = b_vel_x + omega* B / 2
        
        # discrepancy bw what it turn out to be (real track) and what it
        # should be (desired) from encoder
        beta_l = v_enc_l-v_track_l
        beta_r = v_enc_r-v_track_r  
        if (abs(b_vel_xy[1])<0.00001) or (abs(b_vel_xy[0])<0.00001):
            side_slip = 0.
        else:
            side_slip = math.atan2(b_vel_xy[1],b_vel_xy[0])

        return beta_l, beta_r, side_slip, radius, b_vel_xy

    def computeLongSlipCompensationExp(self, v, omega, qd_des, constants):
        # in the case radius is infinite, betas are zero (this is to avoid Nans)

        if (abs(omega) < 1e-05) and (abs(v) > 1e-05):
            radius = 1e08 * np.sign(v)
        elif (abs(omega) < 1e-05) and (abs(v) < 1e-05):
            radius = 1e8
        else:
            radius = v / (omega)

        #compute track velocity from encoder
        v_enc_l = constants.SPROCKET_RADIUS*qd_des[0]
        v_enc_r = constants.SPROCKET_RADIUS*qd_des[1]

        #estimate beta_inner, beta_outer from turning radius
        if(radius >= 0.0): # turning left, positive radius, left wheel is inner right wheel is outer
            beta_l = constants.beta_slip_inner_coefficients_left[0]*np.exp(constants.beta_slip_inner_coefficients_left[1]*radius)
            v_enc_l+=beta_l
            beta_r = constants.beta_slip_outer_coefficients_left[0]*np.exp(constants.beta_slip_outer_coefficients_left[1]*radius)
            v_enc_r+=beta_r

        else:# turning right, negative radius, left wheel is outer right is inner
            beta_r = constants.beta_slip_inner_coefficients_right[0]*np.exp(constants.beta_slip_inner_coefficients_right[1]*radius)
            v_enc_r+=beta_r
            beta_l =  constants.beta_slip_outer_coefficients_right[0]*np.exp(constants.beta_slip_outer_coefficients_right[1]*radius)
            v_enc_l+=beta_l

        qd_comp = np.zeros(4)
        qd_comp[0] = 1/constants.SPROCKET_RADIUS * v_enc_l
        qd_comp[1] = 1/constants.SPROCKET_RADIUS * v_enc_r
        qd_comp[2] = qd_comp[0].copy()
        qd_comp[3] = qd_comp[1].copy()

        return qd_comp, beta_l, beta_r

    def computeLongSlipCompensationMachineLearning(self,  qd_des, constants):
        # compute track velocity from encoder
        v_enc_l = constants.SPROCKET_RADIUS * qd_des[0]
        v_enc_r = constants.SPROCKET_RADIUS * qd_des[1]
        if  self.SLIPPAGE_INFERENCE_TYPE == 'decision_trees':
            # predict the betas from NN
            if len(self.model_beta_l.feature_names_)>2:
                beta_l = self.model_beta_l.predict(np.array([qd_des[0], qd_des[1], self.basePoseW[3], self.basePoseW[4], self.basePoseW[5]]))
                beta_r = self.model_beta_r.predict(np.array([qd_des[0], qd_des[1], self.basePoseW[3], self.basePoseW[4], self.basePoseW[5]]))
            else:
                beta_l = self.model_beta_l.predict(qd_des)
                beta_r = self.model_beta_r.predict(qd_des)
        elif self.SLIPPAGE_INFERENCE_TYPE == 'interpolator':
            beta_l = (self.model_beta_l([qd_des])).squeeze()
            beta_r = (self.model_beta_r([qd_des])).squeeze()
        #matlab
        # beta_l = self.eng.feval(self.model_beta_l['predictFcn'], qd_des)
        # beta_r = self.eng.feval(self.model_beta_r['predictFcn'], qd_des)
        v_enc_l += beta_l
        v_enc_r += beta_r


        qd_comp = np.zeros(4)
        qd_comp[0] = 1 / constants.SPROCKET_RADIUS * v_enc_l
        qd_comp[1] = 1 / constants.SPROCKET_RADIUS * v_enc_r
        qd_comp[2] = qd_comp[0].copy()
        qd_comp[3] = qd_comp[1].copy()
        return qd_comp, beta_l, beta_r
    
    def monitor_time(self):
        self.checkLoopFrequency()
        if np.mod(self.time,1) == 0:
            print(colored(f"TIME: {self.time}","red"))

    def pub_odom_msg(self, odom_publisher):
        msg = Odometry()
        msg.header.stamp = ros.Time.from_sec(self.time)
        msg.pose.pose.orientation.x = self.quaternion[0]
        msg.pose.pose.orientation.y = self.quaternion[1]
        msg.pose.pose.orientation.z = self.quaternion[2]
        msg.pose.pose.orientation.w = self.quaternion[3]
        msg.pose.pose.position.x = self.basePoseW[self.u.sp_crd["LX"]]
        msg.pose.pose.position.y = self.basePoseW[self.u.sp_crd["LY"]]
        msg.pose.pose.position.z = self.basePoseW[self.u.sp_crd["LZ"]]
        msg.twist.twist.linear.x = self.baseTwistW[self.u.sp_crd["LX"]]
        msg.twist.twist.linear.y = self.baseTwistW[self.u.sp_crd["LY"]]
        msg.twist.twist.linear.z = self.baseTwistW[self.u.sp_crd["LZ"]]
        msg.twist.twist.angular.x = self.baseTwistW[self.u.sp_crd["AX"]]
        msg.twist.twist.angular.y = self.baseTwistW[self.u.sp_crd["AY"]]
        msg.twist.twist.angular.z = self.baseTwistW[self.u.sp_crd["AZ"]]
        odom_publisher.publish(msg)

    def initSubscribers(self):
        self.sub_jstate = ros.Subscriber("/" + self.robot_name + "/joint_states", JointState,
                                         callback=self._receive_jstate, queue_size=1, tcp_nodelay=True)
        if self.real_robot:
            print(colored("IMPORTANT: Real robot ON,  be sure param use_sim_time = false","red"))
            # for limo the publisher in on limo0/odom not groundtruth
            self.sub_pose_limo = ros.Subscriber("/" + self.robot_name + "/odom", Odometry, callback=self._receive_pose, queue_size=1, tcp_nodelay=True)
            self.p0[0], self.p0[1], self.p0[2] = getInitialStateFromOdom(self.robot_name)
            self.q_des = getInitialStateFromJoints(robot_name=self.robot_name, joint_names=self.joint_names)
        else:
            self.sub_pose_limo = ros.Subscriber("/" + self.robot_name +  "/ground_truth", Odometry, callback=self._receive_pose, queue_size=1, tcp_nodelay=True)

def talker(p):
    p.start()
    p.startFramework()
    if p.ControlType == "OPEN_LOOP" and p.IDENT_TYPE == 'WHEELS':
        wheel_l = np.linspace(-p.IDENT_MAX_WHEEL_SPEED, p.IDENT_MAX_WHEEL_SPEED, 32)
        for speed in range(len(wheel_l)):
            p.IDENT_WHEEL_L = wheel_l[speed]
            main_loop(p)
    else:
        main_loop(p)

def main_loop(p):
    p.loadModelAndPublishers()

    p.initVars()

    p.initSubscribers()
    p.startupProcedure()

    p.q_old = np.zeros(p.robot.na)
    robot_state = Robot()

    if p.SAVE_BAGS:
        p.recorder.start_recording_srv()

    # OPEN loop control
    if p.ControlType == 'OPEN_LOOP':
        counter = 0
        if p.IDENT_TYPE=='NONE':
            # generic open loop test for comparison with matlab
            #vel_gen = VelocityGenerator(simulation_time=100., DT=conf.robot_params[p.robot_name]['dt'])
            #v_ol, omega_ol, _,_,_ = vel_gen.velocity_mir_smooth() #velocity_straight
            v_ol = np.linspace(0.1, 0.1, np.int32(10./conf.robot_params[p.robot_name]['dt']))
            omega_ol = np.linspace(0.2, 0.2, np.int32(10./conf.robot_params[p.robot_name]['dt']))
            traj_length = len(v_ol)
        if p.IDENT_TYPE == 'V_OMEGA':
            #identification repeat long_v = 0.05:0.05:0.4
            v_ol, omega_ol = p.generateOpenLoopTraj(R_initial= 0.1, R_final=0.4, increment=0.05, dt = conf.robot_params[p.robot_name]['dt'], long_v = p.IDENT_LONG_SPEED, direction=p.IDENT_DIRECTION)
            traj_length = len(v_ol)
        if p.IDENT_TYPE == 'WHEELS':
            wheel_l_ol, wheel_r_ol  = p.generateWheelTraj(p.IDENT_WHEEL_L)
            v_ol, omega_ol = p.mapFromWheels(wheel_l_ol, wheel_r_ol)
            traj_length = len(wheel_l_ol)
            # check the buffer size is big enough
            if  traj_length>conf.robot_params[p.robot_name]['buffer_size']:
                print(colored("Buffer size is not big enough for the ID!"))
                sys.exit()
        p.des_x = p.p0[0]  # +0.1
        p.des_y = p.p0[1]  # +0.1
        p.des_theta = p.p0[2]  # +0.1
        p.traj = Trajectory(ModelsList.UNICYCLE, p.des_x, p.des_y, p.des_theta, DT=conf.robot_params[p.robot_name]['dt'], v=v_ol, omega=omega_ol)

        while not ros.is_shutdown(): #TODO
            if p.IDENT_TYPE == 'WHEELS':
                if counter>=traj_length:
                    print(colored("Open loop test accomplished", "red"))
                    break
                p.qd_des = np.array([wheel_l_ol[counter], wheel_r_ol[counter]])
                counter += 1
            else:
                _, _, _, p.v_d, p.omega_d, _, _, traj_finished = p.traj.evalTraj(p.time)
                p.qd_des = p.publishControlCommand(p.v_d, p.omega_d)
                if traj_finished:
                    break

            p.q_des = p.q_des + p.qd_des * conf.robot_params[p.robot_name]['dt']
            p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, p.v_dot_d, p.omega_dot_d, _ = p.traj.evalTraj(p.time)
            #note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            #senting it to be tracked from the impedance loop
            p.monitor_time()
            p.ros_pub.publishVisual(delete_markers=False)
            p.beta_l, p.beta_r, p.alpha, p.radius, p.b_base_vel = p.estimateSlippages(p.baseTwistW, p.basePoseW[p.u.sp_crd["AZ"]], p.qd)

            # log variables
            p.logData()
            # wait for synconization of the control loop
            p.rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),  4)  # to avoid issues of dt 0.0009999
    else:

        # CLOSE loop control
        # generate reference trajectory
        vel_gen = VelocityGenerator(simulation_time=10.,    DT=conf.robot_params[p.robot_name]['dt'])
        p.des_x = p.p0[0]
        p.des_y = p.p0[1]
        p.des_theta = p.p0[2]
        v_ol, omega_ol, v_dot_ol, omega_dot_ol, _ = vel_gen.velocity_mir_smooth(v_max_=0.1, omega_max_=0.2)
        p.traj = Trajectory(ModelsList.UNICYCLE, start_x=p.des_x, start_y=p.des_y, start_theta=p.des_theta, DT=conf.robot_params[p.robot_name]['dt'],
                            v=v_ol, omega=omega_ol, v_dot=v_dot_ol, omega_dot=omega_dot_ol)


        # Lyapunov controller parameters
        params = LyapunovParams(K_P=1., K_THETA=1., DT=conf.robot_params[p.robot_name]['dt'], ESTIMATE_ALPHA_WITH_ACTUAL_VALUES=p.ESTIMATE_ALPHA_WITH_ACTUAL_VALUES) #high gains 15 5 / low gains 10 1 (default)
        p.controller = LyapunovController(params=params)#, matlab_engine = p.eng)
        p.controller.setSideSlipCompensationType(p.SIDE_SLIP_COMPENSATION)
        p.controller.setSlippageInferenceType(p.SLIPPAGE_INFERENCE_TYPE)
        p.traj.set_initial_time(start_time=p.time)
        while not ros.is_shutdown():
            # update kinematics
            robot_state.x = p.basePoseW[p.u.sp_crd["LX"]]
            robot_state.y = p.basePoseW[p.u.sp_crd["LY"]]
            robot_state.theta = p.basePoseW[p.u.sp_crd["AZ"]]
            #print(f"pos X: {robot.x} Y: {robot.y} th: {robot.theta}")
            # controllers

            p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, p.v_dot_d, p.omega_dot_d, traj_finished = p.traj.evalTraj(p.time)
            if traj_finished:
                break

            if p.ControlType=='CLOSED_LOOP_SLIP_0':
                p.ctrl_v, p.ctrl_omega,  p.V, p.V_dot, p.alpha_control = p.controller.control_alpha(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d,  p.v_dot_d, p.omega_dot_d, traj_finished,p.model_alpha,approx=True)
                #p.des_theta -=  p.controller.alpha_exp(p.v_d, p.omega_d, p.model_alpha)  # we track theta_d -alpha_d

            if p.ControlType == 'CLOSED_LOOP_SLIP':
                p.ctrl_v, p.ctrl_omega, p.V, p.V_dot, p.alpha_control = p.controller.control_alpha(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d,  p.v_dot_d, p.omega_dot_d, traj_finished,p.model_alpha, approx=False)
                #p.des_theta -= p.controller.alpha_exp(p.v_d, p.omega_d, p.model_alpha)  # we track theta_d -alpha_d

            if p.ControlType=='CLOSED_LOOP_UNICYCLE':
                p.ctrl_v, p.ctrl_omega, p.V, p.V_dot = p.controller.control_unicycle(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, traj_finished)

            p.qd_des = p.publishControlCommand(p.ctrl_v, p.ctrl_omega)

            if not p.ControlType=='CLOSED_LOOP_UNICYCLE'  and not traj_finished:
                if p.LONG_SLIP_COMPENSATION=='MACHINE_LEARNING':
                    p.qd_des, p.beta_l_control, p.beta_r_control = p.computeLongSlipCompensationMachineLearning(p.qd_des, constants)
                if p.LONG_SLIP_COMPENSATION == 'EXP':
                    p.qd_des, p.beta_l_control, p.beta_r_control = p.computeLongSlipCompensationExp(p.ctrl_v, p.ctrl_omega, p.qd_des, constants)

              # note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            # senting it to be tracked from the impedance loop
            p.q_des = p.q_des + p.qd_des * conf.robot_params[p.robot_name]['dt']
            p.monitor_time()
            p.ros_pub.publishVisual(delete_markers=False)
            p.beta_l, p.beta_r, p.alpha, p.radius, p.b_base_vel = p.estimateSlippages(p.baseTwistW,p.basePoseW[p.u.sp_crd["AZ"]], p.qd)
            # log variables
            p.logData()
            # wait for synconization of the control loop
            p.rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 4) # to avoid issues of dt 0.0009999

    # always save csv when you do ident
    if p.IDENT_TYPE == 'WHEELS':
        not_nans = ~np.isnan(p.time_log)
        data = pd.DataFrame({
            "time": p.time_log[not_nans],
            "wheel_l": p.qd_des_log[0, not_nans],
            "wheel_r": p.qd_des_log[1, not_nans],
            "roll": p.basePoseW_log[3, not_nans],
            "pitch": p.basePoseW_log[4, not_nans],
            "yaw": p.basePoseW_log[5, not_nans],
            "beta_l": p.beta_l_log[not_nans],
            "beta_r": p.beta_r_log[not_nans],
            "alpha": p.alpha_log[not_nans]})

        output_file = os.environ['LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/regressor/data2d/' + str(p.friction_coefficient) + \
                      f"/ident_wheels_fr_{p.friction_coefficient}_wheelL_{p.IDENT_WHEEL_L}.csv"
        data.to_csv(output_file, index=False)
        print(colored(f"Data saved to {output_file}", "red"))

    if p.SAVE_BAGS:
        p.recorder.stop_recording_srv()
        if p.ControlType !='OPEN_LOOP':
            filename = f'{p.ControlType}_Long_{p.LONG_SLIP_COMPENSATION}_Side_{p.SIDE_SLIP_COMPENSATION}.mat'
            p.log_e_x, p.log_e_y, p.log_e_theta = p.controller.getErrors()
            mio.savemat(filename, {'time': p.time_log, 'des_state': p.des_state_log,
                                   'state': p.state_log,
                                   'pose_des':p.basePoseW_des_log,
                                   'pose':p.basePoseW_log, 'ex': p.log_e_x, 'ey': p.log_e_y, 'etheta': p.log_e_theta,
                                   'v': p.ctrl_v_log, 'vd': p.v_d_log, 'omega': p.ctrl_omega_log, 'omega_d': p.omega_d_log,
                                   'wheel_l': p.qd_log[0, :], 'wheel_r': p.qd_log[1, :], 'beta_l': p.beta_l_log,
                                   'beta_r': p.beta_r_log, 'beta_l_pred': p.beta_l_control_log, 'beta_r_pred': p.beta_r_control_log,
                                   'alpha': p.alpha_log, 'alpha_pred': p.alpha_control_log, 'radius': p.radius_log})

if __name__ == '__main__':
    p = GenericSimulator(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        pass
    if p.SAVE_BAGS:
        p.recorder.stop_recording_srv()
    ros.signal_shutdown("killed")
    p.deregister_node()
    print("Plotting")
    p.plotData()



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
from base_controllers.utils.common_functions import plotFrameLinear, plotJoint, sendStaticTransform, launchFileGeneric, launchFileNode
import params as conf
import os
import sys
import rospkg
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest
from numpy import nan
from matplotlib import pyplot as plt
from base_controllers.utils.math_tools import unwrap_angle
from  base_controllers.doretta.utils import constants as constants
from base_controllers.doretta.controllers.lyapunov import LyapunovController, LyapunovParams, Robot
from  base_controllers.doretta.environment.trajectory import Trajectory, ModelsList
from base_controllers.doretta.velocity_generator import VelocityGenerator
from termcolor import colored
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import catboost as cb
import pinocchio as pin
from base_controllers.components.coppelia_manager import CoppeliaManager


robotName = "tractor" # needs to inherit BaseController

class GenericSimulator(BaseController):
    
    def __init__(self, robot_name="tractor"):
        super().__init__(robot_name=robot_name, external_conf = conf)
        self.torque_control = False
        print("Initialized tractor controller---------------------------------------------------------------")
        self.GAZEBO = False
        self.ControlType = 'CLOSED_LOOP_SLIP_0' #'OPEN_LOOP' 'CLOSED_LOOP_UNICYCLE' 'CLOSED_LOOP_SLIP_0' 'CLOSED_LOOP_SLIP'
        self.IDENT_TYPE = 'NONE' # 'V_OMEGA', 'NONE'
        self.IDENT_DIRECTION = 'left' #used only when OPEN_LOOP
        self.IDENT_LONG_SPEED = 0.1 #0.05:0.05:0.4
        self.IDENT_WHEEL_L = 4.5  # -4.5:0.5:4.5

        self.GRAVITY_COMPENSATION = True

        self.SAVE_BAGS = False
        self.LONG_SLIP_COMPENSATION = 'NONE'#'NN', 'EXP', 'NONE'
        self.NAVIGATION = False
        self.USE_GUI = True #false does not work in headless mode
        self.frictionCoeff=0.3 #0.3 0.6
        self.coppeliaModel=f'tractor_ros_{self.frictionCoeff}_slope.ttt'
        #self.coppeliaModel = f'new_track.ttt'

        if self.GAZEBO and not self.ControlType=='CLOSED_LOOP_UNICYCLE' and not self.ControlType=='OPEN_LOOP':
            print(colored("Gazebo Model has no slippage, turn it off","red"))
            sys.exit()

    def initVars(self):
        super().initVars()

        # regressor
        self.model = cb.CatBoostRegressor()
        # laod model
        self.model.load_model(os.environ['LOCOSIM_DIR']+'/robot_control/base_controllers/doretta/controllers/slippage_regressor_wheels.cb')

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

        self.state_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.des_state_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.beta_l_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.beta_r_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.alpha_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.alpha_control_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.radius_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.beta_l_control_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.beta_r_control_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan

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

                self.alpha_log[self.log_counter] = self.alpha
                self.beta_l_log[self.log_counter] = self.beta_l
                self.beta_r_log[self.log_counter] = self.beta_r

                self.alpha_control_log[self.log_counter] = self.alpha_control
                self.beta_l_control_log[self.log_counter] = self.beta_l_control
                self.beta_r_control_log[self.log_counter] = self.beta_r_control
                self.radius_log[self.log_counter] = self.radius
            super().logData()

    def startSimulator(self):
        if self.GAZEBO:
            world_name = 'ramps.world'
            super().startSimulator(world_name=world_name, additional_args=['spawn_Y:=0.'])
        else:
           self.coppeliaManager = CoppeliaManager(self.coppeliaModel, self.USE_GUI)
           self.coppeliaManager.startSimulator()

    def loadModelAndPublishers(self):
        super().loadModelAndPublishers()
        self.reset_joints_client = ros.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.des_vel = ros.Publisher("/des_vel", JointState, queue_size=1, tcp_nodelay=True)

        if self.NAVIGATION:
            self.nav_vel_sub = ros.Subscriber("/cmd_vel", Twist, self.get_command_vel)
            self.odom_pub = ros.Publisher("/odom", Odometry, queue_size=1, tcp_nodelay=True)
            self.broadcast_world = False # this prevents to publish baselink tf from world because we need baselink to odom
            #launch orchard world
            launchFileGeneric(rospkg.RosPack().get_path('cpr_orchard_gazebo')+"/launch/orchard_world.launch")
            launchFileNode(package="wolf_navigation_utils", launch_file="wolf_navigation.launch", additional_args=['launch_controller:=false', 'robot_model:=tractor','lidar_topic:=/lidar_points','base_frame:=base_link','stabilized_frame:=base_link','launch_odometry:=false','cmd_vel_topic:=/cmd_vel','max_vel_yaw:=1','max_vel_x:=0.5'])

        if self.SAVE_BAGS:
            if p.ControlType=='OPEN_LOOP':
                if p.IDENT_TYPE=='V_OMEGA':
                    bag_name= f"ident_sim_friction_{self.frictionCoeff}_longv_{p.IDENT_LONG_SPEED}_{p.IDENT_DIRECTION}.bag"
                else:
                    bag_name = f"ident_sim_wheelL_{p.IDENT_WHEEL_L}_friction_{self.frictionCoeff}.bag"
            else:
                bag_name = f"{p.ControlType}_Long_{self.LONG_SLIP_COMPENSATION}.bag"
            self.recorder = RosbagControlledRecorder(bag_name=bag_name)

        if not self.GAZEBO:
            # I do this only to check frequency
            self.sub_jstate = ros.Subscriber("/" + self.robot_name + "/joint_states", JointState, callback=self._receive_jstate, queue_size=1, tcp_nodelay=True)
            # self.sub_pose = ros.Subscriber("/" + self.robot_name + "/ground_truth", Odometry, callback=self._receive_pose,
            #                                queue_size=1, tcp_nodelay=True)

    def _receive_jstate(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.q[joint_idx] = msg.position[msg_idx]
                    self.qd[joint_idx] = msg.velocity[msg_idx]
                    self.tau[joint_idx] = msg.effort[msg_idx]
        #check frequency
        if hasattr(self, 'check_time'):
            loop_time = ros.Time.now().to_sec() - self.check_time
            if loop_time > 1.1*(self.slow_down_factor * conf.robot_params[p.robot_name]['dt']):
                freq_ros = 1/(self.slow_down_factor * conf.robot_params[p.robot_name]['dt'])
                freq_coppelia = 1/loop_time
                print(colored(f"freq mismatch beyond 10%: coppelia is running at {freq_coppelia} Hz while it should run at {freq_ros} Hz, freq error is {(freq_ros-freq_coppelia)/freq_ros*100} %", "red"))
        self.check_time = ros.Time.now().to_sec()

    # def _receive_pose(self, msg):
    #     self.quaternion[0]=    msg.pose.pose.orientation.x
    #     self.quaternion[1]=    msg.pose.pose.orientation.y
    #     self.quaternion[2]=    msg.pose.pose.orientation.z
    #     self.quaternion[3]=    msg.pose.pose.orientation.w
    #     self.euler = np.array(euler_from_quaternion(self.quaternion))
    #     #unwrap
    #     self.euler, self.euler_old = unwrap_vector(self.euler, self.euler_old)
    #     # compute orientation matrix
    #     self.b_R_w = self.math_utils.rpyToRot(self.euler)
    #
    #     base_offset_wrt_chassis = np.array([-0.07,0,0])
    #     self.basePoseW[:3] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]) + self.b_R_w.T.dot(base_offset_wrt_chassis)
    #     self.basePoseW[self.u.sp_crd["AX"]] = self.euler[0]
    #     self.basePoseW[self.u.sp_crd["AY"]] = self.euler[1]
    #     self.basePoseW[self.u.sp_crd["AZ"]] = self.euler[2]
    #
    #     omega = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y,  msg.twist.twist.angular.z])
    #     self.baseTwistW[:3] = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]) + omega.dot(self.b_R_w.T.dot(base_offset_wrt_chassis))
    #     self.baseTwistW[3:] = omega
    #
    #     if self.broadcast_world:
    #         self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
    #                                    self.quaternion,
    #                                    ros.Time.now(), '/base_link', '/world')

    def get_command_vel(self, msg):
        self.v_d = msg.linear.x
        self.omega_d = msg.angular.z

    def deregister_node(self):
        if not self.GAZEBO:
            self.coppeliaManager.simulationControl('stop')
        os.system("killall rosmaster rviz coppeliaSim")
        super().deregister_node()


    def startupProcedure(self):
        if self.GAZEBO:
            super().startupProcedure()
            if self.torque_control:
                self.pid.setPDs(0.0, 0.0, 0.0)
            # loop frequency
            self.rate = ros.Rate(1 / conf.robot_params[p.robot_name]['dt'])
        else:
            # we need to broadcast the TF world baselink in coppelia not in base controller for synchro issues
            self.broadcast_world = False
            # start coppeliasim
            self.coppeliaManager.simulationControl('start')
            # manage coppelia simulation from locosim
            self.coppeliaManager.simulationControl('enable_sync_mode')
            #Coppelia Runs with increment of 0.01 but is not able to run at 100Hz but at 25 hz so I "slow down" ros, but still update time with dt = 0.01
            self.slow_down_factor = 8
            # loop frequency
            self.rate = ros.Rate(1 / (self.slow_down_factor * conf.robot_params[p.robot_name]['dt']))

    def plotData(self):
        if conf.plotting:
            #xy plot
            plt.figure()
            plt.plot(p.des_state_log[0, :], p.des_state_log[1, :], "-r", label="desired")
            plt.plot(p.state_log[0, :], p.state_log[1, :], "-b", label="real")
            plt.legend()
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
            plt.figure()
            plt.subplot(2, 1, 1)
            plt.plot(p.time_log, p.qd_log[0,:], "-b",  linewidth=3)
            plt.plot(p.time_log, p.qd_des_log[0, :], "-r",  linewidth=4)
            plt.plot(p.time_log, constants.MAXSPEED_RADS_PULLEY*np.ones((len(p.time_log))), "-k",  linewidth=4)
            plt.plot(p.time_log, -constants.MAXSPEED_RADS_PULLEY*np.ones((len(p.time_log))), "-k",  linewidth=4)
            plt.ylabel("WHEEL_L")
            plt.grid(True)
            plt.subplot(2, 1, 2)
            plt.plot(p.time_log, p.qd_log[1, :], "-b",  linewidth=3)
            plt.plot(p.time_log, p.qd_des_log[1, :], "-r",  linewidth=4)                
            plt.plot(p.time_log, constants.MAXSPEED_RADS_PULLEY*np.ones((len(p.time_log))), "-k",  linewidth=4)
            plt.plot(p.time_log, -constants.MAXSPEED_RADS_PULLEY*np.ones((len(p.time_log))), "-k",  linewidth=4)
            plt.ylabel("WHEEL_R")
            plt.grid(True)

            #states plot
            plotFrameLinear(name='position',time_log=p.time_log,des_Pose_log = p.des_state_log, Pose_log=p.state_log)
            plotFrameLinear(name='velocity', time_log=p.time_log, Twist_log=np.vstack((p.baseTwistW_log[:2,:],p.baseTwistW_log[5,:])))

            if not self.GAZEBO and not p.ControlType=='CLOSED_LOOP_UNICYCLE' and not p.ControlType=='OPEN_LOOP':
                #slippage vars
                plt.figure()
                plt.subplot(4, 1, 1)
                plt.plot(self.time_log, self.beta_l_log, "-b", label="real")
                plt.plot(self.time_log, self.beta_l_control_log, "-r", label="control")
                plt.ylabel("beta_l")
                plt.legend()
                plt.grid(True)
                plt.subplot(4, 1, 2)
                plt.plot(self.time_log, self.beta_r_log, "-b", label="real")
                plt.plot(self.time_log, self.beta_r_control_log, "-r", label="control")
                plt.ylabel("beta_r")
                plt.legend()
                plt.grid(True)
                plt.subplot(4, 1, 3)
                plt.plot(self.time_log, self.alpha_log, "-b", label="real")
                plt.plot(self.time_log, self.alpha_control_log, "-r", label="control")
                plt.ylabel("alpha")
                plt.ylim([-1, 1])
                plt.grid(True)
                plt.legend()
                plt.subplot(4, 1, 4)
                plt.plot(self.time_log, self.radius_log, "-b")
                plt.ylim([-1,1])
                plt.ylabel("radius")
                plt.grid(True)

            if p.ControlType == 'CLOSED_LOOP':
                # tracking errors
                self.log_e_x, self.log_e_y, self.log_e_theta = self.controller.getErrors()
                plt.figure()
                plt.subplot(2, 1, 1)
                plt.plot(np.sqrt(np.power(self.log_e_x,2) +np.power(self.log_e_y,2)), "-b")
                plt.ylabel("exy")
                plt.grid(True)
                plt.subplot(2, 1, 2)
                plt.plot(self.log_e_theta, "-b")
                plt.ylabel("eth")
                plt.grid(True)

                #
                # # liapunov V
                # plt.figure()
                # plt.plot(p.time_log, p.V_log, "-b", label="REAL")
                # plt.legend()
                # plt.xlabel("time[sec]")
                # plt.ylabel("V liapunov")
                # # plt.axis("equal")
                # plt.grid(True)
                #
                # #base position
                # plotFrame('position', time_log=p.time_log, Pose_log=p.basePoseW_log,
                #           title='Base', frame='W', sharex=True)

    def mapToWheels(self, v_des,omega_des):
        #
        # # SAFE CHECK -> clipping velocities
        # v = np.clip(v, -constants.MAX_LINEAR_VELOCITY, constants.MAX_LINEAR_VELOCITY)
        # o = np.clip(o, -constants.MAX_ANGULAR_VELOCITY, constants.MAX_ANGULAR_VELOCITY)
        qd_des = np.zeros(2)
        qd_des[0] = (v_des - omega_des * constants.TRACK_WIDTH / 2)/constants.SPROCKET_RADIUS  # left front
        qd_des[1] = (v_des + omega_des * constants.TRACK_WIDTH / 2)/constants.SPROCKET_RADIUS  # right front

        #publish des commands as well
        msg = JointState()
        msg.name = self.joint_names
        msg.header.stamp = ros.Time.from_sec(self.time)
        msg.velocity = np.array([v_des, omega_des])
        self.des_vel.publish(msg)


        return qd_des
    #unwrap the joints states
    def unwrap(self):
        for i in range(self.robot.na):
            self.q[i], self.q_old[i] =unwrap_angle(self.q[i], self.q_old[i])

    def generateWheelTraj(self, wheel_l = -4.5):
        ####################################
        # OPEN LOOP wl , wr (from -4.5 to 4.5)
        ####################################
        wheel_l_vec = []
        wheel_r_vec = []
        change_interval = 3.
        nsamples = 12
        if wheel_l <= 0.: #this is to make such that the ID starts always with no rotational speed
            wheel_r = np.linspace(-constants.MAXSPEED_RADS_PULLEY, constants.MAXSPEED_RADS_PULLEY, nsamples)
        else:
            wheel_r = np.linspace(constants.MAXSPEED_RADS_PULLEY, -constants.MAXSPEED_RADS_PULLEY, nsamples)
        time = 0
        i = 0
        while True:
            time = np.round(time + conf.robot_params[p.robot_name]['dt'], 3)
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
        change_interval = 6.
        increment = increment
        turning_radius_vec = np.arange(R_initial, R_final, increment)
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

        #compute BF velocity
        w_R_b = np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta), np.cos(theta)]])
        b_vel_xy = (w_R_b.T).dot(w_vel_xy)
        b_vel_x = b_vel_xy[0]

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
        if (abs(b_vel_xy[1])<0.001) or (abs(b_vel_xy[0])<0.001):
            side_slip = 0.
        else:
            side_slip = math.atan2(b_vel_xy[1],b_vel_xy[0])

        return beta_l, beta_r, side_slip

    def computeGravityCompensation(self, roll, pitch):
        W = np.array([0., 0., constants.mass*9.81, 0., 0., 0.])
        track_discretization = 4
        grasp_matrix_left = np.zeros((6, 3*track_discretization))
        grasp_matrix_right = np.zeros((6, 3*track_discretization))
        b_pos_track_left_x = np.linspace(constants.b_left_track_start[0], constants.b_left_track_end[0], track_discretization)
        b_pos_track_right_x = np.linspace(constants.b_right_track_start[0], constants.b_right_track_end[0], track_discretization)
        #mapping to horizontal frame
        w_R_b = self.math_utils.eul2Rot(np.array([roll, pitch, self.basePoseW[self.u.sp_crd["AZ"]]]))

        #fill in grasp matrix
        for i in range(track_discretization):
            b_pos_track_left_i = np.array([b_pos_track_left_x[i], constants.b_left_track_start[1], constants.b_left_track_start[2]])
            grasp_matrix_left[:3,i*3:i*3+3] = np.eye(3)
            grasp_matrix_left[3:, i * 3:i * 3 + 3] = pin.skew( w_R_b.dot(b_pos_track_left_i))
            self.ros_pub.add_arrow(p.basePoseW[:3], w_R_b.dot(b_pos_track_left_i), "red")

        for i in range(track_discretization):
            b_pos_track_right_i = np.array([b_pos_track_right_x[i], constants.b_right_track_start[1], constants.b_right_track_start[2]])
            grasp_matrix_right[:3,i*3:i*3+3] = np.eye(3)
            grasp_matrix_right[3:, i * 3:i * 3 + 3] = pin.skew( w_R_b.dot(b_pos_track_right_i))

        #get track element forces distributiong load
        fi = np.linalg.pinv(np.hstack((grasp_matrix_left,grasp_matrix_right))).dot(W)


        #get contact frame
        ty = w_R_b.dot(np.array([0, 1., 0.]))
        n = w_R_b.dot(np.array([0., 0., 1.]))
        #projection on tx
        # fill in projection/sum matrix
        proj_matrix = np.zeros((3, 3*track_discretization))
        sum_matrix = np.zeros((3, 3*track_discretization))
        for i in range(track_discretization):
            #compute projections by outer product I-n*nt, project first on n-x plane then on x
            proj_matrix[:3, i*3:i*3+3] = (np.eye(3) - np.multiply.outer(n.ravel(), n.ravel())).dot((np.eye(3) -  np.multiply.outer(ty.ravel(), ty.ravel())))
            sum_matrix[:3, i*3:i*3+3] = np.eye(3)

        F_l = sum_matrix.dot(fi[:3*track_discretization])
        F_r = sum_matrix.dot(fi[3*track_discretization:])
        F_lx = proj_matrix.dot(fi[:3*track_discretization])
        F_rx = proj_matrix.dot(fi[3*track_discretization:])

        tau_g = np.array([F_lx*constants.SPROCKET_RADIUS, F_rx*constants.SPROCKET_RADIUS])
        return  tau_g, F_l, F_r

    def computeLongSlipCompensation(self, v, omega, qd_des, constants):
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

        qd_comp = np.zeros(2)
        qd_comp[0] = 1/constants.SPROCKET_RADIUS * v_enc_l
        qd_comp[1] = 1/constants.SPROCKET_RADIUS * v_enc_r
        return qd_comp, beta_l, beta_r, radius

    def computeLongSlipCompensationNN(self, v, omega, qd_des, constants):
        # in the case radius is infinite, betas are zero (this is to avoid Nans)
        #
        if (abs(omega) < 1e-05) and (abs(v) > 1e-05):
            radius = 1e08 * np.sign(v)
        elif (abs(omega) < 1e-05) and (abs(v) < 1e-05):
            radius = 1e8
        else:
            radius = v / (omega)

        # compute track velocity from encoder
        v_enc_l = constants.SPROCKET_RADIUS * qd_des[0]
        v_enc_r = constants.SPROCKET_RADIUS * qd_des[1]
        beta_l, beta_r, alpha = self.model.predict(qd_des)

        v_enc_l += beta_l
        v_enc_r += beta_r


        qd_comp = np.zeros(2)
        qd_comp[0] = 1 / constants.SPROCKET_RADIUS * v_enc_l
        qd_comp[1] = 1 / constants.SPROCKET_RADIUS * v_enc_r
        return qd_comp, beta_l, beta_r, radius
    
    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
        # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
        msg = JointState()
        msg.name = self.joint_names
        msg.header.stamp = ros.Time.from_sec(self.time)
        msg.position = q_des
        msg.velocity = qd_des
        msg.effort = tau_ffwd
        self.pub_des_jstate.publish(msg)

        #this is for wolf navigation # gazebo publishes world, base controller publishes base link, we need to add two static transforms from world to odom and map
        if p.NAVIGATION:
            self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
                                           self.quaternion,
                                           ros.Time.now(), '/base_link', '/odom')
            sendStaticTransform("odom", "world", np.zeros(3), np.array([1, 0, 0, 0]))  # this is just to not  brake the locosim rviz that still wants world
            msg = Odometry()
            msg.pose.pose.orientation.x = self.quaternion[0]
            msg.pose.pose.orientation.y = self.quaternion[1]
            msg.pose.pose.orientation.z = self.quaternion[2]
            msg.pose.pose.orientation.w = self.quaternion[3]
            msg.pose.pose.position.x = self.basePoseW[self.u.sp_crd["LX"]]
            msg.pose.pose.position.y = self.basePoseW[self.u.sp_crd["LY"]]
            msg.pose.pose.position.z = self.basePoseW[self.u.sp_crd["LZ"]]
            msg.twist.twist.linear.x= self.baseTwistW[self.u.sp_crd["LX"]]
            msg.twist.twist.linear.y= self.baseTwistW[self.u.sp_crd["LY"]]
            msg.twist.twist.linear.z= self.baseTwistW[self.u.sp_crd["LZ"]]
            msg.twist.twist.angular.x= self.baseTwistW[self.u.sp_crd["AX"]]
            msg.twist.twist.angular.y= self.baseTwistW[self.u.sp_crd["AY"]]
            msg.twist.twist.angular.z= self.baseTwistW[self.u.sp_crd["AZ"]]
            self.odom_pub.publish(msg)

        if np.mod(self.time,1) == 0:
            print(colored(f"TIME: {self.time}","red"))

def talker(p):
    p.start()
    p.startSimulator()
    p.loadModelAndPublishers()
    p.robot.na = 2 #initialize properly vars for only 2 actuators (other 2 are caster wheels)
    p.initVars()
    p.q_old = np.zeros(2)
    p.initSubscribers()
    p.startupProcedure()

    #init joints
    p.q_des = np.copy(p.q_des_q0)
    p.q_old = np.zeros(2)
    robot_state = Robot()
    ros.sleep(1.)
    #
    p.q_des = np.zeros(2)
    p.qd_des = np.zeros(2)
    p.tau_ffwd = np.zeros(2)

    if p.SAVE_BAGS:
        p.recorder.start_recording_srv()
    # OPEN loop control
    if p.ControlType == 'OPEN_LOOP':
        counter = 0
        if p.IDENT_TYPE=='NONE':
        # generic open loop test for comparison with matlab
            vel_gen = VelocityGenerator(simulation_time=100., DT=conf.robot_params[p.robot_name]['dt'])
            v_ol, omega_ol, _,_,_ = vel_gen.velocity_mir_smooth() #velocity_straight
            traj_length = len(v_ol)
        if p.IDENT_TYPE == 'V_OMEGA':
            #identification repeat long_v = 0.05:0.05:0.4
            v_ol, omega_ol = p.generateOpenLoopTraj(R_initial= 0.1, R_final=0.6, increment=0.05, dt = conf.robot_params[p.robot_name]['dt'], long_v = p.IDENT_LONG_SPEED, direction=p.IDENT_DIRECTION)
            traj_length = len(v_ol)
        if p.IDENT_TYPE == 'WHEELS':
            wheel_l_ol, wheel_r_ol  = p.generateWheelTraj(p.IDENT_WHEEL_L)
            traj_length = len(wheel_l_ol)

        while not ros.is_shutdown():
            if counter<traj_length:
                if p.IDENT_TYPE == 'WHEELS':
                    p.qd_des = np.array([wheel_l_ol[counter], wheel_r_ol[counter]])
                else:
                    p.v_d = v_ol[counter]
                    p.omega_d = omega_ol[counter]
                    p.qd_des = p.mapToWheels(p.v_d, p.omega_d )
                counter+=1
            else:
                print(colored("Identification test accomplished", "red"))
                break
            #forward_speed = 1. #max speed is 4.56 rad/s
            #p.qd_des = forward_speed
            if p.torque_control:
                p.tau_ffwd = 30*conf.robot_params[p.robot_name]['kp'] * np.subtract(p.q_des, p.q) -2* conf.robot_params[p.robot_name]['kd'] * p.qd
            else:
                p.tau_ffwd = np.zeros(p.robot.na)

            p.q_des = p.q_des + p.qd_des * conf.robot_params[p.robot_name]['dt']

            if p.GRAVITY_COMPENSATION:
                p.tau_g, p.F_l, p.F_r = p.computeGravityCompensation(p.basePoseW[p.u.sp_crd["AX"]], p.basePoseW[p.u.sp_crd["AY"]])
                w_center_track_left = p.b_R_w.T.dot(0.5 * (constants.b_left_track_start + constants.b_left_track_end))
                w_center_track_right = p.b_R_w.T.dot(0.5 * (constants.b_right_track_start + constants.b_right_track_end))
                p.ros_pub.add_arrow(p.basePoseW[:3] + w_center_track_left, p.F_l / np.linalg.norm(p.F_l), "red")
                p.ros_pub.add_arrow(p.basePoseW[:3] + w_center_track_right, p.F_r / np.linalg.norm(p.F_r), "red")

            #note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            #senting it to be tracked from the impedance loop
            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
            p.ros_pub.publishVisual(delete_markers=False)
            if not p.GAZEBO:
                p.coppeliaManager.simulationControl('trigger_next_step')
                p.unwrap()
                # log variables
            p.logData()
            # wait for synconization of the control loop
            p.rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),  3)  # to avoid issues of dt 0.0009999
    else:

        # CLOSE loop control
        # generate reference trajectory
        vel_gen = VelocityGenerator(simulation_time=40.,    DT=conf.robot_params[p.robot_name]['dt'])
        # initial_des_x = 0.1
        # initial_des_y = 0.1
        # initial_des_theta = 0.3
        initial_des_x = 0.0
        initial_des_y = 0.0
        initial_des_theta = 0.0
        p.traj = Trajectory(ModelsList.UNICYCLE, initial_des_x, initial_des_y, initial_des_theta, vel_gen.velocity_mir_smooth, conf.robot_params[p.robot_name]['dt'])


        # Lyapunov controller parameters
        params = LyapunovParams(K_P=5., K_THETA=1., DT=conf.robot_params[p.robot_name]['dt'])
        p.controller = LyapunovController(params=params)
        p.traj.set_initial_time(start_time=p.time)
        while not ros.is_shutdown():
            # update kinematics
            robot_state.x = p.basePoseW[p.u.sp_crd["LX"]]
            robot_state.y = p.basePoseW[p.u.sp_crd["LY"]]
            robot_state.theta = p.basePoseW[p.u.sp_crd["AZ"]]
            #print(f"pos X: {robot.x} Y: {robot.y} th: {robot.theta}")

            # controllers
            if p.NAVIGATION:
                p.des_x, p.des_y, p.des_theta = p.traj.getSingleUpdate(p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d)
                traj_finished = None
            else:
                p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, p.v_dot_d, p.omega_dot_d, traj_finished = p.traj.evalTraj(p.time)
                if traj_finished:
                    break
            if p.ControlType=='CLOSED_LOOP_SLIP_0':
                p.ctrl_v, p.ctrl_omega,  p.V, p.V_dot, p.alpha_control = p.controller.control_alpha(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d,  p.v_dot_d, p.omega_dot_d, traj_finished, approx=True)
                p.des_theta -=  p.controller.alpha_exp(p.v_d, p.omega_d)  # we track theta_d -alpha_d

            if p.ControlType == 'CLOSED_LOOP_SLIP':
                p.ctrl_v, p.ctrl_omega, p.V, p.V_dot, p.alpha_control = p.controller.control_alpha(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d,  p.v_dot_d, p.omega_dot_d, traj_finished,approx=False)
                p.des_theta -= p.controller.alpha_exp(p.v_d, p.omega_d)  # we track theta_d -alpha_d

            if p.ControlType=='CLOSED_LOOP_UNICYCLE':
                p.ctrl_v, p.ctrl_omega, p.V, p.V_dot = p.controller.control_unicycle(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, traj_finished)
            p.qd_des = p.mapToWheels(p.ctrl_v, p.ctrl_omega)

            if not p.ControlType=='CLOSED_LOOP_UNICYCLE' and p.LONG_SLIP_COMPENSATION is not 'NONE' and not traj_finished:
                if p.LONG_SLIP_COMPENSATION=='NN':
                    p.qd_des, p.beta_l_control, p.beta_r_control, p.radius = p.computeLongSlipCompensationNN(p.ctrl_v, p.ctrl_omega,p.qd_des, constants)
                else:#exponential
                    p.qd_des, p.beta_l_control, p.beta_r_control, p.radius = p.computeLongSlipCompensation(p.ctrl_v, p.ctrl_omega, p.qd_des, constants)
    
            # note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            # senting it to be tracked from the impedance loop
            p.q_des = p.q_des + p.qd_des * conf.robot_params[p.robot_name]['dt']
            if p.GRAVITY_COMPENSATION:
                p.tau_g, p.F_l, p.F_r = p.computeGravityCompensation(p.basePoseW[p.u.sp_crd["AX"]], p.basePoseW[p.u.sp_crd["AY"]])
                w_center_track_left = p.b_R_w.T.dot(0.5*(constants.b_left_track_start+constants.b_left_track_end))
                w_center_track_right = p.b_R_w.T.dot(0.5 * (constants.b_right_track_start + constants.b_right_track_end))
                p.ros_pub.add_arrow(p.basePoseW[:3] + w_center_track_left, p.F_l/np.linalg.norm(p.F_l), "red")
                p.ros_pub.add_arrow(p.basePoseW[:3] + w_center_track_right, p.F_r/np.linalg.norm(p.F_r), "red")

            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
            p.ros_pub.publishVisual(delete_markers=False)

            if not p.GAZEBO:
                p.coppeliaManager.simulationControl('trigger_next_step')
                p.unwrap()

            p.beta_l, p.beta_r, p.alpha = p.estimateSlippages(p.baseTwistW,p.basePoseW[p.u.sp_crd["AZ"]], p.qd)
            # log variables
            p.logData()
            # wait for synconization of the control loop
            p.rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999
    if p.SAVE_BAGS:
        p.recorder.stop_recording_srv()

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



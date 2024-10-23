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
from  base_controllers.tracked_robot.utils import constants as constants
from base_controllers.tracked_robot.controllers.lyapunov import LyapunovController, LyapunovParams, Robot
from  base_controllers.tracked_robot.environment.trajectory import Trajectory, ModelsList
from base_controllers.tracked_robot.velocity_generator import VelocityGenerator
from termcolor import colored
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import catboost as cb
import pinocchio as pin
from base_controllers.components.coppelia_manager import CoppeliaManager
from optim_interfaces.srv import Optim, OptimRequest
import scipy.io.matlab as mio
from base_controllers.tracked_robot.simulator.tracked_vehicle_simulator import TrackedVehicleSimulator, Ground
from base_controllers.tracked_robot.simulator.terrain_manager import TerrainManager
from base_controllers.utils.common_functions import getRobotModelFloating
from base_controllers.utils.common_functions import checkRosMaster
from base_controllers.utils.common_functions import spawnModel
import pandas as pd

robotName = "tractor" # needs to inherit BaseController

class GenericSimulator(BaseController):
    
    def __init__(self, robot_name="tractor"):
        super().__init__(robot_name=robot_name, external_conf = conf)
        self.torque_control = False
        print("Initialized tractor controller---------------------------------------------------------------")
        self.SIMULATOR = 'gazebo'#, 'gazebo', 'coppelia'(deprecated), 'biral'
        self.NAVIGATION = 'none'  # 'none', '2d' , '3d'
        self.TERRAIN = False

        self.STATISTICAL_ANALYSIS = True
        self.ControlType = 'CLOSED_LOOP_UNICYCLE' #'OPEN_LOOP' 'CLOSED_LOOP_UNICYCLE' 'CLOSED_LOOP_SLIP_0' 'CLOSED_LOOP_SLIP'
        self.SIDE_SLIP_COMPENSATION = 'NN'#'NN', 'EXP', 'NONE'
        self.LONG_SLIP_COMPENSATION = 'NN'#'NN', 'EXP(not used)', 'NONE'
        self.ESTIMATE_ALPHA_WITH_ACTUAL_VALUES = False # makes difference for v >= 0.4

        # Parameters for open loop identification
        self.IDENT_TYPE = 'WHEELS' # 'V_OMEGA', 'WHEELS', 'NONE'
        self.IDENT_MAX_WHEEL_SPEED = 12 #used only when IDENT_TYPE = 'WHEELS' 7/12
        self.IDENT_LONG_SPEED = 0.2  #used only when IDENT_TYPE = 'V_OMEGA' 0.2, 0.55 (riccardo)
        self.IDENT_DIRECTION = 'left' #used only when IDENT_TYPE = 'V_OMEGA'

        #biral friction coeff
        self.friction_coefficient = 0.1 # 0.1/ 0.09041/ 0.13349 / 0.1568 /

        # initial pose
        self.p0 = np.array([0., 0., 0.]) #FOR PAPER np.array([-0.05, 0.03, 0.01])

        self.MATLAB_PLANNING = 'none' # 'none', 'dubins' , 'optim'
        # target used only for matlab trajectory generation (dubins/optimization) #need to run dubins_optimization/ros/ros_node.m
        self.pf = np.array([2., 2.5, -0.4])

        self.GRAVITY_COMPENSATION = False
        self.SAVE_BAGS = False

        self.USE_GUI = False
        self.ADD_NOISE = False #FOR PAPER
        self.coppeliaModel=f'tractor_ros_0.3_slope.ttt'

        if self.SIMULATOR == 'gazebo' and not self.ControlType=='CLOSED_LOOP_UNICYCLE' and not self.ControlType=='OPEN_LOOP':
            print(colored("Gazebo Model has no slippage, use self.SIMULATOR:=biral","red"))
            sys.exit()
        if self.NAVIGATION !='none':
            #add custom models from wolf, need to clone git@github.com:graiola/wolf_gazebo_resources.git
            custom_models_path = rospkg.RosPack().get_path('wolf_gazebo_resources') + "/models/"
            os.environ["GAZEBO_MODEL_PATH"] += ":" + custom_models_path

    def initVars(self):
        super().initVars()

        # regressor
        self.regressor_beta_l = cb.CatBoostRegressor()
        self.regressor_beta_r = cb.CatBoostRegressor()
        self.regressor_alpha = cb.CatBoostRegressor()
        # laod model
        try:
            self.model_beta_l = self.regressor_beta_l.load_model(os.environ['LOCOSIM_DIR']+'/robot_control/base_controllers/tracked_robot/regressor/model_beta_l.cb')
            self.model_beta_r = self.regressor_beta_r.load_model(os.environ['LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/regressor/model_beta_r.cb')
            self.model_alpha = self.regressor_alpha.load_model(os.environ['LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/regressor/model_alpha.cb')

        except:
            print(colored("need to generate the models with running tracked_robot/regressor/model_slippage_updated.py","red"))
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
        if self.SIMULATOR == 'gazebo':
            world_name = None #'ramps.world'
            additional_args = ['spawn_x:=' + str(p.p0[0]),'spawn_y:=' + str(p.p0[1]),'spawn_Y:=' + str(p.p0[2]), 'rviz_conf:=$(find tractor_description)/rviz/conf.rviz']
            super().startSimulator(world_name=world_name, additional_args=additional_args)
            if self.TERRAIN:
                #spawn the terrain model in gazebo
                spawnModel("tractor_description", "terrain", spawn_pos=np.array([0.,0.,0.]))
        elif self.SIMULATOR == 'coppelia':
           self.coppeliaManager = CoppeliaManager(self.coppeliaModel, self.USE_GUI)
           self.coppeliaManager.startSimulator()
        else: # Biral simulator
            os.system("killall rosmaster rviz gzserver coppeliaSim")
            # launch roscore
            checkRosMaster()
            ros.sleep(1.5)
            # run robot state publisher + load robot description + rviz
            launchFileGeneric(rospkg.RosPack().get_path('tractor_description') + "/launch/rviz_nojoints.launch")
            groundParams = Ground(friction_coefficient=self.friction_coefficient)
            self.tracked_vehicle_simulator = TrackedVehicleSimulator(dt=conf.robot_params[p.robot_name]['dt'], ground=groundParams)

            self.robot = getRobotModelFloating(self.robot_name)
            # instantiating additional publishers
            self.joint_pub = ros.Publisher("/" + self.robot_name + "/joint_states", JointState, queue_size=1)
            self.groundtruth_pub = ros.Publisher("/" + self.robot_name + "/ground_truth", Odometry, queue_size=1, tcp_nodelay=True)

    def loadModelAndPublishers(self):
        super().loadModelAndPublishers()
        self.reset_joints_client = ros.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.des_vel = ros.Publisher("/des_vel", JointState, queue_size=1, tcp_nodelay=True)
        if self.TERRAIN:
            self.ros_pub.add_mesh("tractor_description", "/meshes/terrain.stl", position=np.array([0.,0.,0.0]), color="red")
            self.terrainManager = TerrainManager(rospkg.RosPack().get_path('tractor_description') + "/meshes/terrain.stl")
        if self.NAVIGATION != 'none':
            print(colored("IMPORTANT: be sure that you are running the image mfocchi/trento_lab_framework:introrob_upgrade", "red"))
            self.nav_vel_sub = ros.Subscriber("/cmd_vel", Twist, self.get_command_vel)
            self.odom_pub = ros.Publisher("/odom", Odometry, queue_size=1, tcp_nodelay=True)
            self.broadcast_world = False # this prevents to publish baselink tf from world because we need baselink to odom
            #launch orchard world
            #launchFileGeneric(rospkg.RosPack().get_path('cpr_orchard_gazebo') + "/launch/orchard_world.launch")

            # type: indoor: based on slam 2d (default: gmapping)
            # type: outdoor: based on gps
            # type: hybrid: EKF fusing slam with gps
            # type: 3d: slam 3d (on pointcloud)
            # slam:  default = "gmapping", gmapping/slam_toolbox/hector_mapping
            print(colored("Starting wolf navigation", "red"))
            if self.NAVIGATION=='2d':
                launchFileNode(package="wolf_navigation_utils", launch_file="wolf_navigation.launch", additional_args=['map_file:=/tmp/embty.db',
                                                                                                                       'type:=indoor',
                                                                                                                       'launch_controller:=false',
                                                                                                                       'robot_model:=tractor',
                                                                                                                       'lidar_topic:=/lidar_points',
                                                                                                                       'base_frame:=base_link',
                                                                                                                       'stabilized_frame:=horizontal_frame',
                                                                                                                       'launch_odometry:=false',
                                                                                                                       'world_name:=inspection',
                                                                                                                       'cmd_vel_topic:=/cmd_vel',
                                                                                                                       'max_vel_yaw:=1', 'max_vel_x:=0.5'])
            if self.NAVIGATION=='3d':
                #to set 3dNav goal: sx click + drag +right click to move up
                launchFileNode(package="wolf_navigation_utils", launch_file="wolf_navigation.launch", additional_args=['map_file:=/tmp/embty.db',
                                                                                                                       'type:=3d',
                                                                                                                       'launch_controller:=false',
                                                                                                                       'robot_model:=tractor', #loads planner params inside tractor description
                                                                                                                       'lidar_topic:=/lidar_points',
                                                                                                                       'slope_filter:= true',
                                                                                                                       'global_planner:=wolf_3dnav_planner/Wolf3DNavPlanner',
                                                                                                                       'base_frame:=base_link',
                                                                                                                       'stabilized_frame:=horizontal_frame',
                                                                                                                       'launch_odometry:=false',
                                                                                                                       'world_name:=inspection',
                                                                                                                       'cmd_vel_topic:=/cmd_vel',
                                                                                                                       'max_vel_yaw:=1', 'max_vel_x:=0.5'])

        if self.SAVE_BAGS:
            if p.ControlType=='OPEN_LOOP':
                if p.IDENT_TYPE=='V_OMEGA':
                    bag_name= f"ident_sim_longv_{p.IDENT_LONG_SPEED}_{p.IDENT_DIRECTION}_fr_{p.friction_coefficient}.bag"
                else:
                    bag_name = f"ident_sim_wheelL_{p.IDENT_WHEEL_L}.bag"
            else:
                bag_name = f"{p.ControlType}_Long_{self.LONG_SLIP_COMPENSATION}_Side_{p.SIDE_SLIP_COMPENSATION}.bag"
            self.recorder = RosbagControlledRecorder(bag_name=bag_name)


    # This will be used instead of the basecontroller one, I do it just to check frequency
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


    def getTrajFromMatlab(self):
        try:
            ros.wait_for_service('/optim', timeout=30)
            self.optim_client = ros.ServiceProxy('/optim', Optim)
            request_optim = OptimRequest()
            request_optim.x0 = self.p0[0]
            request_optim.y0 = self.p0[1]
            request_optim.theta0 = self.p0[2]
            request_optim.xf = self.pf[0]
            request_optim.yf = self.pf[1]
            request_optim.thetaf = self.pf[2]
            request_optim.plan_type = self.MATLAB_PLANNING
            response = self.optim_client(request_optim)
            print(colored(f"Planning with {request_optim.plan_type}", "red"))
            #print(response.des_x[-10:])
            # print(response.des_y[-10:0])
            # print(response.des_theta[-10:])
            # print(response.des_v[-10:])
            # print(response.des_omega[-10:])
            return response.des_x,response.des_y,response.des_theta,response.des_v, response.des_omega, response.dt

        except:
            print(colored("Matlab service call /optim not available"), "red")

    def get_command_vel(self, msg):
        self.v_d = msg.linear.x
        self.omega_d = msg.angular.z

    def deregister_node(self):
        if self.SIMULATOR == 'coppelia':
            self.coppeliaManager.simulationControl('stop')
        os.system("killall rosmaster rviz coppeliaSim")
        super().deregister_node()


    def startupProcedure(self):
        if self.SIMULATOR == 'gazebo':
            super().startupProcedure()
            if self.torque_control:
                self.pid.setPDs(0.0, 0.0, 0.0)
            self.slow_down_factor = 1
        elif self.SIMULATOR == 'coppelia':
            # we need to broadcast the TF world baselink in coppelia not in base controller for synchro issues
            self.broadcast_world = False
            # start coppeliasim
            self.coppeliaManager.simulationControl('start')
            # manage coppelia simulation from locosim
            self.coppeliaManager.simulationControl('enable_sync_mode')
            #Coppelia Runs with increment of 0.01 but is not able to run at 100Hz but at 25 hz so I "slow down" ros, but still update time with dt = 0.01
            self.slow_down_factor = 8

        else:#Biral
            self.tracked_vehicle_simulator.initSimulation(vbody_init=np.array([0, 0, 0.0]), pose_init=self.p0)
            self.broadcast_world = False
            self.slow_down_factor = 1
            # important, you need to reset also baseState otherwise robot_state the first time will be set to 0,0,0!
            self.basePoseW[self.u.sp_crd["LX"]] = self.p0[0]  # fixed height TODO change this when on slopes
            self.basePoseW[self.u.sp_crd["LY"]] = self.p0[1]  # fixed height TODO change this when on slopes
            self.basePoseW[self.u.sp_crd["LZ"]] = self.tracked_vehicle_simulator.tracked_robot.vehicle_param.height  # fixed height TODO change this when on slopes
            self.basePoseW[self.u.sp_crd["AZ"]] = self.p0[2]  # fixed height TODO change this when on slopes

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
            #plotFrameLinear(name='velocity', time_log=p.time_log, Twist_log=np.vstack((p.baseTwistW_log[:2,:],p.baseTwistW_log[5,:])))

            if self.SIMULATOR != 'gazebo':
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
                plt.ylim([-0.4, 0.4])
                plt.grid(True)
                plt.legend()
                plt.subplot(4, 1, 4)
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

    def mapFromWheels(self, wheel_l, wheel_r):
        if not np.isscalar(wheel_l):
            v = np.zeros_like(wheel_l)
            omega = np.zeros_like(wheel_l)
            for i in range(len(wheel_l)):
                v[i] = constants.SPROCKET_RADIUS*(wheel_l[i] + wheel_r[i])/2
                omega[i] = constants.SPROCKET_RADIUS/constants.TRACK_WIDTH*(wheel_r[i] -wheel_l[i])
            return v, omega


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
        if wheel_l <= 0.: #this is to make such that the ID starts always with no rotational speed
            wheel_r = np.linspace(-self.IDENT_MAX_WHEEL_SPEED, self.IDENT_MAX_WHEEL_SPEED, 24) #it if passes from 0 for some reason there is a non linear
                #behaviour in the long slippage
        else:
            wheel_r =np.linspace(self.IDENT_MAX_WHEEL_SPEED, -self.IDENT_MAX_WHEEL_SPEED, 24)
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

        #compute BF velocity
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

        return beta_l, beta_r, side_slip, radius

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
            #debug
            # self.ros_pub.add_arrow(p.basePoseW[:3], w_R_b.dot(b_pos_track_left_i), "red")

        for i in range(track_discretization):
            b_pos_track_right_i = np.array([b_pos_track_right_x[i], constants.b_right_track_start[1], constants.b_right_track_start[2]])
            grasp_matrix_right[:3,i*3:i*3+3] = np.eye(3)
            grasp_matrix_right[3:, i * 3:i * 3 + 3] = pin.skew( w_R_b.dot(b_pos_track_right_i))

        #get track element forces distributiong load
        fi = np.linalg.pinv(np.hstack((grasp_matrix_left,grasp_matrix_right))).dot(W)


        #get contact frame
        ty = w_R_b.dot(np.array([0, 1., 0.]))
        n = w_R_b.dot(np.array([0., 0., 1.]))
        tx = np.cross(ty,n)
        #projection on tx
        # fill in projection/sum matrix
        proj_matrix = np.zeros((3, 3*track_discretization))
        sum_matrix = np.zeros((3, 3*track_discretization))
        for i in range(track_discretization):
            #compute projections by outer product I-n*nt, project first on plane n-x  then on x
            proj_matrix[:3, i*3:i*3+3] = tx.dot((np.eye(3) -  np.multiply.outer(ty.ravel(), ty.ravel())))
            sum_matrix[:3, i*3:i*3+3] = np.eye(3)

        F_l = sum_matrix.dot(fi[:3*track_discretization])
        F_r = sum_matrix.dot(fi[3*track_discretization:])

        #project each little force separately
        # F_lx = proj_matrix.dot(fi[:3*track_discretization])
        # F_rx = proj_matrix.dot(fi[3*track_discretization:])

        # as an alternative directly project F_l F_r
        # project first on plane n-x then onto nx
        P_xn = np.eye(3) -  np.multiply.outer(ty.ravel(), ty.ravel())
        F_lx = tx.dot(P_xn.dot(F_l))
        F_rx = tx.dot(P_xn.dot(F_r))

        tau_g = np.array([F_lx * constants.SPROCKET_RADIUS, F_rx * constants.SPROCKET_RADIUS])

        return  tau_g, F_l, F_r

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

        qd_comp = np.zeros(2)
        qd_comp[0] = 1/constants.SPROCKET_RADIUS * v_enc_l
        qd_comp[1] = 1/constants.SPROCKET_RADIUS * v_enc_r


        return qd_comp, beta_l, beta_r

    def computeLongSlipCompensationNN(self,  qd_des, constants):
        # compute track velocity from encoder
        v_enc_l = constants.SPROCKET_RADIUS * qd_des[0]
        v_enc_r = constants.SPROCKET_RADIUS * qd_des[1]
        # predict the betas from NN
        beta_l = self.model_beta_l.predict(qd_des)
        beta_r = self.model_beta_r.predict(qd_des)

        v_enc_l += beta_l
        v_enc_r += beta_r


        qd_comp = np.zeros(2)
        qd_comp[0] = 1 / constants.SPROCKET_RADIUS * v_enc_l
        qd_comp[1] = 1 / constants.SPROCKET_RADIUS * v_enc_r
        return qd_comp, beta_l, beta_r
    
    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
        # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
        msg = JointState()
        msg.name = self.joint_names
        msg.header.stamp = ros.Time.from_sec(self.time)
        msg.position = q_des
        msg.velocity = qd_des
        msg.effort = tau_ffwd
        self.pub_des_jstate.publish(msg) #publish in /commands

        #trigger simulators
        if self.SIMULATOR == 'biral': #TODO implement torque control
            if self.ControlType != 'OPEN_LOOP' and self.LONG_SLIP_COMPENSATION  != 'NONE':
                if np.any(qd_des > np.array([constants.MAXSPEED_RADS_PULLEY, constants.MAXSPEED_RADS_PULLEY])) or np.any(qd_des < -np.array([constants.MAXSPEED_RADS_PULLEY, constants.MAXSPEED_RADS_PULLEY])):
                    print(colored("wheel speed beyond limits, NN might do wrong predictions", "red"))
            self.tracked_vehicle_simulator.simulateOneStep(qd_des[0], qd_des[1])
            pose, pose_der =  self.tracked_vehicle_simulator.getRobotState()
            #fill in base state
            self.basePoseW[:2] = pose[:2]
            self.basePoseW[self.u.sp_crd["AZ"]] = pose[2]
            self.euler = self.u.angPart(self.basePoseW)
            self.baseTwistW[:2] = pose_der[:2]
            self.baseTwistW[self.u.sp_crd["AZ"]] = pose_der[2]

            self.quaternion = pin.Quaternion(pin.rpy.rpyToMatrix(self.euler))
            self.b_R_w = self.math_utils.eul2Rot(self.euler).T
            #publish TF for rviz
            self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
                                           self.quaternion,
                                           ros.Time.now(), '/base_link', '/world')
            self.pub_odom_msg(self.groundtruth_pub) #this is to publish on the topic groundtruth if somebody needs it
            self.q = q_des.copy()
            self.qd = qd_des.copy()
            self.joint_pub.publish(msg)  # this publishes q = q_des, it is just for rviz

        if self.SIMULATOR == 'coppelia':
            self.coppeliaManager.simulationControl('trigger_next_step')
            self.unwrap()


        #this is for wolf navigation # gazebo publishes world, base controller publishes base link, we need to add two static transforms from world to odom and map
        if p.NAVIGATION != 'none':
            self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
                                           self.quaternion,
                                           ros.Time.now(), '/base_link', '/odom')
            #publish horizontal frame for stabilizer
            self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
                                           pin.Quaternion(pin.rpy.rpyToMatrix(np.array([0., 0., self.euler[2]]))),
                                           ros.Time.now(), '/horizontal_frame', '/world')

            sendStaticTransform("odom", "world", np.zeros(3), np.array([1, 0, 0, 0]))  # this is just to not  brake the locosim rviz that still wants world
            self.pub_odom_msg(self.odom_pub)

        if self.TERRAIN:
            eval_point, roll, pitch = self.terrainManager.project_on_mesh(point=self.basePoseW[:2], direction=np.array([0.,0.,1.]))
            w_R_terr = self.math_utils.eul2Rot(np.array([roll, pitch, self.euler[2]]))
            w_normal = w_R_terr.dot(np.array([0,0,1]))

            self.ros_pub.add_arrow(eval_point, w_normal, color="blue")
            self.ros_pub.add_marker(eval_point, color="blue")

        if np.mod(self.time,1) == 0:
            if not self.STATISTICAL_ANALYSIS:
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

    def genRandomTarget(self, radius_min, radius_max):
        radius = radius_min + radius_max * np.random.uniform(low=0, high=1, size=1)
        phi = np.random.uniform(low=0, high=2*np.pi, size=1)
        theta = np.random.uniform(low=0, high=2*np.pi, size=1)
        return np.array([radius.item()*np.cos(phi).item(), radius.item()*np.sin(phi).item(), theta.item()])

def talker(p):
    p.start()
    p.startSimulator()
    if p.ControlType == "OPEN_LOOP" and p.IDENT_TYPE == 'WHEELS':
        wheel_l = np.linspace(-p.IDENT_MAX_WHEEL_SPEED, p.IDENT_MAX_WHEEL_SPEED, 24)
        for speed in range(len(wheel_l)):
            p.IDENT_WHEEL_L = wheel_l[speed]
            main_loop(p)
    elif p.STATISTICAL_ANALYSIS:
        print(colored('CREATING NEW CSV TO STORE  TESTS', 'blue'))
        columns = [ 'test', 'target_x','target_y','target_z', 'exy', 'etheta']
        p.df = pd.DataFrame(columns=columns)
        for p.test in range(100):
            p.p0 = np.zeros(3)
            p.pf = p.genRandomTarget(1., 4.)
            main_loop(p)
    else:
        main_loop(p)

def main_loop(p):
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
            #vel_gen = VelocityGenerator(simulation_time=100., DT=conf.robot_params[p.robot_name]['dt'])
            #v_ol, omega_ol, _,_,_ = vel_gen.velocity_mir_smooth() #velocity_straight
            v_ol = np.linspace(0.4, 0.4, np.int32(20./conf.robot_params[p.robot_name]['dt']))
            omega_ol = np.linspace(0.2, 0.2, np.int32(20./conf.robot_params[p.robot_name]['dt']))
            traj_length = len(v_ol)
        if p.IDENT_TYPE == 'V_OMEGA':
            #identification repeat long_v = 0.05:0.05:0.4
            v_ol, omega_ol = p.generateOpenLoopTraj(R_initial= 0.1, R_final=0.6, increment=0.05, dt = conf.robot_params[p.robot_name]['dt'], long_v = p.IDENT_LONG_SPEED, direction=p.IDENT_DIRECTION)
            traj_length = len(v_ol)
        if p.IDENT_TYPE == 'WHEELS':
            wheel_l_ol, wheel_r_ol  = p.generateWheelTraj(p.IDENT_WHEEL_L)
            v_ol, omega_ol = p.mapFromWheels(wheel_l_ol, wheel_r_ol)
            traj_length = len(wheel_l_ol)

        if p.MATLAB_PLANNING == 'none':
            p.traj = Trajectory(ModelsList.UNICYCLE, p.p0[0], p.p0[1], p.p0[2], DT=conf.robot_params[p.robot_name]['dt'], v=v_ol, omega=omega_ol)
        else:#matlab planning
            des_x_vec, des_y_vec,des_theta_vec, v_ol, omega_ol, matlab_dt=  p.getTrajFromMatlab()
            p.traj = Trajectory(None, des_x_vec, des_y_vec,des_theta_vec, None, DT=matlab_dt, v=v_ol, omega=omega_ol)
            traj_length = len(v_ol)

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
                print(colored("Open loop test accomplished", "red"))
                break
            #forward_speed = 1. #max speed is 4.56 rad/s
            #p.qd_des = forward_speed
            if p.torque_control:
                p.tau_ffwd = 30*conf.robot_params[p.robot_name]['kp'] * np.subtract(p.q_des, p.q) -2* conf.robot_params[p.robot_name]['kd'] * p.qd
            else:
                p.tau_ffwd = np.zeros(p.robot.na)

            p.q_des = p.q_des + p.qd_des * conf.robot_params[p.robot_name]['dt']

            if p.GRAVITY_COMPENSATION:
                p.tau_ffwd, p.F_l, p.F_r = p.computeGravityCompensation(p.basePoseW[p.u.sp_crd["AX"]], p.basePoseW[p.u.sp_crd["AY"]])
                #w_center_track_left = p.b_R_w.T.dot(0.5 * (constants.b_left_track_start + constants.b_left_track_end))
                #w_center_track_right = p.b_R_w.T.dot(0.5 * (constants.b_right_track_start + constants.b_right_track_end))
                #p.ros_pub.add_arrow(p.basePoseW[:3] + w_center_track_left, p.F_l / np.linalg.norm(p.F_l), "red")
                #p.ros_pub.add_arrow(p.basePoseW[:3] + w_center_track_right, p.F_r / np.linalg.norm(p.F_r), "red")

            p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, p.v_dot_d, p.omega_dot_d, _ = p.traj.evalTraj(p.time)
            #note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            #senting it to be tracked from the impedance loop
            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
            p.ros_pub.publishVisual(delete_markers=False)

            p.beta_l, p.beta_r, p.alpha, p.radius = p.estimateSlippages(p.baseTwistW, p.basePoseW[p.u.sp_crd["AZ"]], p.qd)

            # log variables
            p.logData()
            # wait for synconization of the control loop
            p.rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),  3)  # to avoid issues of dt 0.0009999
    else:

        # CLOSE loop control
        # generate reference trajectory
        vel_gen = VelocityGenerator(simulation_time=20.,    DT=conf.robot_params[p.robot_name]['dt'])

        if p.MATLAB_PLANNING == 'none':
            # initial_des_x = p.p0[0]+0.1
            # initial_des_y = p.p0[1]+0.1
            # initial_des_theta = p.p0[2]+0.3
            initial_des_x = p.p0[0]
            initial_des_y = p.p0[1]
            initial_des_theta = p.p0[2]
            v_ol, omega_ol, v_dot_ol, omega_dot_ol, _ = vel_gen.velocity_mir_smooth(v_max_=0.2, omega_max_=0.3) #slow 0.2 0.3 / fast 0.25 0.4 (for higher linear speed alpha is not predicted properly)
            p.traj = Trajectory(ModelsList.UNICYCLE, initial_des_x, initial_des_y, initial_des_theta, DT=conf.robot_params[p.robot_name]['dt'],
                                v=v_ol, omega=omega_ol, v_dot=v_dot_ol, omega_dot=omega_dot_ol)
        else:
            des_x_vec, des_y_vec, des_theta_vec, v_ol, omega_ol, matlab_dt = p.getTrajFromMatlab()
            p.traj = Trajectory(None, des_x_vec, des_y_vec, des_theta_vec, None, DT=matlab_dt, v=v_ol, omega=omega_ol)


        # Lyapunov controller parameters
        params = LyapunovParams(K_P=10., K_THETA=1., DT=conf.robot_params[p.robot_name]['dt'], ESTIMATE_ALPHA_WITH_ACTUAL_VALUES=p.ESTIMATE_ALPHA_WITH_ACTUAL_VALUES) #high gains 15 5 / low gains 10 1 (default)
        p.controller = LyapunovController(params=params)
        p.controller.setSideSlipCompensationType(p.SIDE_SLIP_COMPENSATION)
        p.traj.set_initial_time(start_time=p.time)
        while not ros.is_shutdown():
            # update kinematics
            robot_state.x = p.basePoseW[p.u.sp_crd["LX"]]
            robot_state.y = p.basePoseW[p.u.sp_crd["LY"]]
            robot_state.theta = p.basePoseW[p.u.sp_crd["AZ"]]
            #print(f"pos X: {robot.x} Y: {robot.y} th: {robot.theta}")

            # controllers
            if p.NAVIGATION !='none':
                p.des_x, p.des_y, p.des_theta = p.traj.getSingleUpdate(p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d)
                traj_finished = None
            else:
                p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, p.v_dot_d, p.omega_dot_d, traj_finished = p.traj.evalTraj(p.time)
                if traj_finished:
                    break
            if p.ControlType=='CLOSED_LOOP_SLIP_0':
                p.ctrl_v, p.ctrl_omega,  p.V, p.V_dot, p.alpha_control = p.controller.control_alpha(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d,  p.v_dot_d, p.omega_dot_d, traj_finished,p.model_alpha,approx=True)
                p.des_theta -=  p.controller.alpha_exp(p.v_d, p.omega_d, p.model_alpha)  # we track theta_d -alpha_d

            if p.ControlType == 'CLOSED_LOOP_SLIP':
                p.ctrl_v, p.ctrl_omega, p.V, p.V_dot, p.alpha_control = p.controller.control_alpha(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d,  p.v_dot_d, p.omega_dot_d, traj_finished,p.model_alpha, approx=False)
                p.des_theta -= p.controller.alpha_exp(p.v_d, p.omega_d, p.model_alpha)  # we track theta_d -alpha_d

            if p.ControlType=='CLOSED_LOOP_UNICYCLE':
                p.ctrl_v, p.ctrl_omega, p.V, p.V_dot = p.controller.control_unicycle(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, traj_finished)

            p.qd_des = p.mapToWheels(p.ctrl_v, p.ctrl_omega)

            if not p.ControlType=='CLOSED_LOOP_UNICYCLE'  and not traj_finished:
                if p.LONG_SLIP_COMPENSATION=='NN':
                    p.qd_des, p.beta_l_control, p.beta_r_control = p.computeLongSlipCompensationNN(p.qd_des, constants)
                if p.LONG_SLIP_COMPENSATION == 'EXP':
                    p.qd_des, p.beta_l_control, p.beta_r_control = p.computeLongSlipCompensationExp(p.ctrl_v, p.ctrl_omega, p.qd_des, constants)

            if p.ADD_NOISE:
                p.qd_des += np.random.normal(0, 0.02)

            # note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            # senting it to be tracked from the impedance loop
            p.q_des = p.q_des + p.qd_des * conf.robot_params[p.robot_name]['dt']
            if p.GRAVITY_COMPENSATION:
                p.tau_g, p.F_l, p.F_r = p.computeGravityCompensation(p.basePoseW[p.u.sp_crd["AX"]], p.basePoseW[p.u.sp_crd["AY"]])
                # w_center_track_left = p.b_R_w.T.dot(0.5*(constants.b_left_track_start+constants.b_left_track_end))
                # w_center_track_right = p.b_R_w.T.dot(0.5 * (constants.b_right_track_start + constants.b_right_track_end))
                # p.ros_pub.add_arrow(p.basePoseW[:3] + w_center_track_left, p.F_l/np.linalg.norm(p.F_l), "red")
                # p.ros_pub.add_arrow(p.basePoseW[:3] + w_center_track_right, p.F_r/np.linalg.norm(p.F_r), "red")


            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
            p.ros_pub.publishVisual(delete_markers=False)

            p.beta_l, p.beta_r, p.alpha, p.radius = p.estimateSlippages(p.baseTwistW,p.basePoseW[p.u.sp_crd["AZ"]], p.qd)
            # log variables
            p.logData()
            # wait for synconization of the control loop
            p.rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999

    if p.SAVE_BAGS:
        p.recorder.stop_recording_srv()
        filename = f'{p.ControlType}_Long_{p.LONG_SLIP_COMPENSATION}_Side_{p.SIDE_SLIP_COMPENSATION}.mat'
        if p.ControlType !='OPEN_LOOP':
            p.log_e_x, p.log_e_y, p.log_e_theta = p.controller.getErrors()
            mio.savemat(filename, {'time': p.time_log, 'des_state': p.des_state_log,
                                   'state': p.state_log, 'ex': p.log_e_x, 'ey': p.log_e_y, 'etheta': p.log_e_theta,
                                   'v': p.ctrl_v_log, 'vd': p.v_d_log, 'omega': p.ctrl_omega_log, 'omega_d': p.omega_d_log,
                                   'wheel_l': p.qd_log[0, :], 'wheel_r': p.qd_log[1, :], 'beta_l': p.beta_l_log,
                                   'beta_r': p.beta_r_log, 'beta_l_pred': p.beta_l_control_log, 'beta_r_pred': p.beta_r_control_log,
                                   'alpha': p.alpha_log, 'alpha_pred': p.alpha_control_log, 'radius': p.radius_log})

    if p.STATISTICAL_ANALYSIS:
        p.log_e_x, p.log_e_y, p.log_e_theta = p.controller.getErrors()
        e_xy = np.sqrt(np.power(p.log_e_x, 2) + np.power(p.log_e_y, 2))
        rmse_xy = np.sqrt(np.mean(e_xy ** 2))
        rmse_theta = np.sqrt(np.mean(np.array(p.log_e_theta) ** 2))
        print(colored(f"Target: {p.pf.reshape(1,3)}, e_xy: {rmse_xy} e_theta {rmse_theta}","red"))
        dict = {'test':p.test, 'target_x': p.pf[0],'target_y': p.pf[1],'target_z': p.pf[2], 'exy': rmse_xy, 'etheta': rmse_theta}
        df_dict = pd.DataFrame([dict])
        p.df = pd.concat([p.df, df_dict], ignore_index=True)
        p.df.to_csv(f'statistic_{p.ControlType}_Matlab_{p.MATLAB_PLANNING}.csv', index=None)

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



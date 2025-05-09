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
from base_controllers.tracked_robot.simulator.tracked_vehicle_simulator3d import  TrackedVehicleSimulator3D, Ground3D
from base_controllers.utils.common_functions import getRobotModelFloating
from base_controllers.utils.common_functions import checkRosMaster
from base_controllers.utils.common_functions import spawnModel, launchFileNode
from base_controllers.tracked_robot.regressor.NN.inference_nn import SlipNN
import pandas as pd
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from rosgraph_msgs.msg import Clock

robotName = "tractor" # needs to inherit BaseController

class GenericSimulator(BaseController):
    
    def __init__(self, robot_name="tractor"):
        super().__init__(robot_name=robot_name, external_conf = conf)
        self.torque_control = False
        print("Initialized tractor controller---------------------------------------------------------------")
        self.SIMULATOR = 'distributed2d'#, 'gazebo(unicycle)', 'coppelia'(deprecated), 'distributed2d'(2d) 'distributed3d'
        self.NAVIGATION = 'none'  # 'none', '2d' , '3d'
        self.TERRAIN = False #True: Slopes False: Flat terrain

        self.STATISTICAL_ANALYSIS = False #samples targets and orientations in a given space around the robot and compute average tracking error
        self.ControlType = 'CLOSED_LOOP_SLIP_0' #'OPEN_LOOP' 'CLOSED_LOOP_UNICYCLE' 'CLOSED_LOOP_SLIP_0' 'CLOSED_LOOP_SLIP'
        self.SIDE_SLIP_COMPENSATION = 'MACHINE_LEARNING' # 'MACHINE_LEARNING', 'NONE', 'EXP(not used)'
        self.LONG_SLIP_COMPENSATION = 'MACHINE_LEARNING' # 'MACHINE_LEARNING', 'NONE', 'EXP(not used)'
        self.SLIPPAGE_INFERENCE_TYPE = 'decision_trees'  # 'decision_trees','interpolator' , 'NN'
        self.ESTIMATE_ALPHA_WITH_ACTUAL_VALUES = True # makes difference for v >= 0.4

        # Parameters for open loop identification
        self.IDENT_TYPE = 'NONE' # 'V_OMEGA(deprecated)', 'WHEELS', 'NONE'
        self.IDENT_LONG_SPEED = 0.2  #used only when IDENT_TYPE = 'V_OMEGA' (deprecated)
        self.IDENT_DIRECTION = 'left' #used only when IDENT_TYPE = 'V_OMEGA' (deprecated)

        self.friction_coefficient = 0.4 # 0.1 (used only in 2d) / 0.4 (2d and 3d) (used for planning in paper)/ 0.6 (only 3d)  with slopes we need high friction otherwise alpha is too high
        #distributed friction coeff

        # initial pose
        self.p0 = np.array([0., 0., 0.])
        #self.p0 = np.array([15., 2., 0.]) #FOR PAPER for closed loop slope test 3d
        #self.p0 = np.array([0.05, 0.03, 0.01]) #FOR PAPER user_defined_reference

        # target used only for matlab trajectory generation (dubins/optimization) #need to run dubins_optimization/ros/ros_node.m
        self.pf = np.array([2., 2.5, -0.4])
        self.PLANNING = 'none' # 'none', 'dubins' , 'optim', 'clothoids'
        self.PLANNING_SPEED = 0.4

        self.GRAVITY_COMPENSATION = False
        self.SAVE_BAGS = False

        self.USE_GUI = False
        self.ADD_NOISE = False #FOR PAPER user_defined_reference
        self.coppeliaModel=f'tractor_ros_0.3_slope.ttt'

        if self.SIMULATOR == 'gazebo' and not self.ControlType=='CLOSED_LOOP_UNICYCLE' and not self.ControlType=='OPEN_LOOP':
            print(colored("Gazebo Model has no slippage, use self.SIMULATOR:=distributed2d","red"))
            sys.exit()
        if self.NAVIGATION !='none':
            #add custom models from wolf, need to clone git@github.com:graiola/wolf_gazebo_resources.git
            custom_models_path = rospkg.RosPack().get_path('wolf_gazebo_resources') + "/models/"
            os.environ["GAZEBO_MODEL_PATH"] += ":" + custom_models_path

    def initVars(self):
        super().initVars()
        # load model
        try:
            if self.SLIPPAGE_INFERENCE_TYPE=='decision_trees':
                # regressor
                self.regressor_beta_l = cb.CatBoostRegressor()
                self.regressor_beta_r = cb.CatBoostRegressor()
                self.regressor_alpha = cb.CatBoostRegressor()
                self.model_beta_l = self.regressor_beta_l.load_model(os.environ['LOCOSIM_DIR']+'/robot_control/base_controllers/tracked_robot/regressor/model_beta_l'+self.flag3D+str(self.friction_coefficient)+'.cb')
                self.model_beta_r = self.regressor_beta_r.load_model(os.environ['LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/regressor/model_beta_r'+self.flag3D+str(self.friction_coefficient)+'.cb')
                self.model_alpha = self.regressor_alpha.load_model(os.environ['LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/regressor/model_alpha'+self.flag3D+str(self.friction_coefficient)+'.cb')
                # loading with matlab
                # import matlab.engine
                # self.eng = matlab.engine.start_matlab()
                # self.model_alpha = self.eng.load(os.environ['LOCOSIM_DIR']+'/robot_control/base_controllers/tracked_robot/regressor/training.mat')['alpha_model_18']
                # self.model_beta_l = self.eng.load(os.environ['LOCOSIM_DIR']+'/robot_control/base_controllers/tracked_robot/regressor/training.mat')['beta_l_model_18']
                # self.model_beta_r = self.eng.load(os.environ['LOCOSIM_DIR']+'/robot_control/base_controllers/tracked_robot/regressor/training.mat')['beta_r_model_18']
            elif  self.SLIPPAGE_INFERENCE_TYPE=='NN':
                self.model_beta_l = SlipNN(output='beta_l')
                self.model_beta_r = SlipNN(output='beta_r')
                self.model_alpha = SlipNN(output='alpha')
            elif self.SLIPPAGE_INFERENCE_TYPE=='interpolator':
                from scipy.interpolate import RBFInterpolator
                data = os.environ['LOCOSIM_DIR']+f'/robot_control/base_controllers/tracked_robot/regressor/ident_wheels_sim_2d_'+str(self.friction_coefficient)+'.csv'
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
        self.pub_counter = 0
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

    def startSimulator(self):
        self.decimate_publish = 1
        if self.SIMULATOR == 'gazebo':
            world_name = None #'ramps.world'
            additional_args = ['spawn_x:=' + str(p.p0[0]),'spawn_y:=' + str(p.p0[1]),'spawn_Y:=' + str(p.p0[2]), 'rviz_conf:=$(find tractor_description)/rviz/conf.rviz']
            super().startSimulator(world_name=world_name, additional_args=additional_args)
        elif self.SIMULATOR == 'coppelia':
           self.coppeliaManager = CoppeliaManager(self.coppeliaModel, self.USE_GUI)
           self.coppeliaManager.startSimulator()
        else: # Biral simulator distributed2d/distributed3d
            os.system("killall rosmaster rviz gzserver coppeliaSim")
            # launch roscore
            checkRosMaster()
            ros.sleep(1.5)
            if self.friction_coefficient == 0.1: #should match with training region (very slippery only on flat)
                constants.MAXSPEED_RADS_PULLEY = 10.
                self.IDENT_MAX_WHEEL_SPEED = 10.  # used only when IDENT_TYPE = 'WHEELS' 7/12
            if self.friction_coefficient == 0.4 or self.friction_coefficient == 0.6: #should match with training region
                constants.MAXSPEED_RADS_PULLEY = 18.
                self.IDENT_MAX_WHEEL_SPEED = 18.
                # run robot state publisher + load robot description + rviz
            launchFileGeneric(rospkg.RosPack().get_path('tractor_description') + "/launch/rviz_nojoints.launch")
            if self.SIMULATOR == 'distributed3d':
                print(colored("SIMULATION 3D is unstable for dt > 0.001, resetting dt=0.001 and increased 5x buffer_size", "red"))
                conf.robot_params[self.robot_name]['buffer_size'] *= 5
                conf.robot_params[p.robot_name]['dt'] = 0.001
                groundParams = Ground3D(friction_coefficient=self.friction_coefficient, terrain_stiffness=1e05, terrain_damping=0.5e04)
                self.tracked_vehicle_simulator = TrackedVehicleSimulator3D(dt=conf.robot_params[p.robot_name]['dt'],  ground=groundParams, USE_MESH=self.TERRAIN, enable_visuals=False, contact_distribution=False)
                self.flag3D='_3d_'
            else: #'distributed2d':
                if (self.friction_coefficient != 0.4) or (self.friction_coefficient != 0.1):
                    print(colored("wrong friction coeff, can be 0.1 or 0.4"))
                groundParams = Ground(friction_coefficient=self.friction_coefficient)
                self.tracked_vehicle_simulator = TrackedVehicleSimulator(dt=conf.robot_params[p.robot_name]['dt'], ground=groundParams)
                self.flag3D=''
            self.robot = getRobotModelFloating(self.robot_name)
            # instantiating additional publishers
            self.joint_pub = ros.Publisher("/" + self.robot_name + "/joint_states", JointState, queue_size=1)
            if self.IDENT_TYPE!='NONE':
                self.PLANNING = 'none'
                self.groundtruth_pub = ros.Publisher("/" + self.robot_name + "/ground_truth", Odometry, queue_size=1, tcp_nodelay=True)
                if self.IDENT_TYPE == 'WHEELS' and self.SIMULATOR == 'distributed3d':
                    self.TERRAIN = True
            if self.NAVIGATION!='none' and self.TERRAIN: #need to launch empty gazebo to emulate the lidar
                launchFileNode(package='gazebo_ros', launch_file='empty_world.launch', additional_args = ['use_sim_time:=true','gui:=false','paused:=false'])
                spawnModel(package_name='tractor_description', model_name='lidar')
                self.set_state = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)

        if self.TERRAIN and (self.NAVIGATION!='none' or self.SIMULATOR=='gazebo'):
            # spawn the terrain model in gazebo in case of navigation
            spawnModel("tractor_description", "terrain", spawn_pos=np.array([0., 0., 0.]))

    def loadModelAndPublishers(self):
        super().loadModelAndPublishers()
        self.reset_joints_client = ros.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.des_vel = ros.Publisher("/des_vel", JointState, queue_size=1, tcp_nodelay=True)

        #self.clock_pub = ros.Publisher('/clock', Clock, queue_size=10)


        if self.TERRAIN and self.SIMULATOR=='distributed3d': #terrain is only available in distributed3d
            from base_controllers.tracked_robot.simulator.terrain_manager import TerrainManager
            self.terrainManager = TerrainManager(rospkg.RosPack().get_path('tractor_description') + "/meshes/terrain.stl")
            self.tracked_vehicle_simulator.setTerrainManager(self.terrainManager)
            if self.IDENT_TYPE=='WHEELS' :
                from base_controllers.tracked_robot.simulator.terrain_manager import create_ramp_mesh
                ramp_mesh = create_ramp_mesh(length=350., width=350., inclination=p.RAMP_INCLINATION, origin=np.array([0, 0, 0]))
                self.terrainManager.set_mesh(ramp_mesh)
        if self.NAVIGATION != 'none':
            print(colored("IMPORTANT: be sure that you are running the image mfocchi/trento_lab_framework:introrob_upgrade", "red"))
            self.nav_vel_sub = ros.Subscriber("/cmd_vel", Twist, self.get_command_vel)
            self.odom_pub = ros.Publisher("/odom", Odometry, queue_size=1, tcp_nodelay=True)
            self.broadcast_world = False # this prevents to publish baselink tf from world because we need baselink to odom

            # type: indoor: based on slam 2d (default: gmapping)
            # type: outdoor: based on gps
            # type: hybrid: EKF fusing slam with gps
            # type: 3d: slam 3d (on pointcloud)
            # slam:  default = "gmapping", gmapping/slam_toolbox/hector_mapping
            print(colored("Starting wolf navigation", "red"))
            if self.NAVIGATION=='2d':
                # launch orchard world
                #launchFileGeneric(rospkg.RosPack().get_path('cpr_orchard_gazebo') + "/launch/orchard_world.launch")
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
                                                                                                                       'max_vel_yaw:=1', 'max_vel_x:=0.8'])
            if self.NAVIGATION=='3d':
                #to set 3dNav goal: sx click + drag +right click to move up
                self.p0 = np.array([5, -3.8, 0])
                print(colored(f"Navigation 3d: adjusting initial position to {self.p0} because to be able to generate a path the nav should have the map of the hill"))
                self.friction_coefficient = 0.6
                print(colored(f"Navigation 3d: Setting friction coeff to {self.friction_coefficient} otherwise it slips too much"))
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
                                                                                                                       'max_vel_yaw:=1', 'max_vel_x:=1.2'])


        if self.SAVE_BAGS:
            if p.ControlType=='OPEN_LOOP':
                if p.IDENT_TYPE=='V_OMEGA':
                    bag_name= f"ident_sim_longv_{p.IDENT_LONG_SPEED}_{p.IDENT_DIRECTION}_fr_{p.friction_coefficient}.bag"
                if p.IDENT_TYPE == 'WHEELS':
                    if p.SIMULATOR=='distributed3d':
                        bag_name = f"ident_sim_fr_{p.friction_coefficient}_ramp_{p.RAMP_INCLINATION}_wheelL_{p.IDENT_WHEEL_L}.bag"
                    else:
                        bag_name = f"ident_sim_wheelL_{p.IDENT_WHEEL_L}.bag"

            else:
                bag_name = f"{p.ControlType}_Long_{self.LONG_SLIP_COMPENSATION}_Side_{p.SIDE_SLIP_COMPENSATION}.bag"
            self.recorder = RosbagControlledRecorder(bag_name=bag_name)

    # This will be used instead of the basecontroller one, I do it just to check frequency!
    def _receive_jstate(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.q[joint_idx] = msg.position[msg_idx]
                    self.qd[joint_idx] = msg.velocity[msg_idx]
                    self.tau[joint_idx] = msg.effort[msg_idx]

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

    def getClothoids(self, long_vel, dt = 0.001):
        import Clothoids
        curve = Clothoids.ClothoidCurve("curve")
        curve.build_G1(self.p0[0], self.p0[1], self.p0[2],self.pf[0], self.pf[1], self.pf[2])
        self.PLANNING_DURATION = curve.length() / long_vel
        number_of_samples = int(np.floor(self.PLANNING_DURATION / dt))
        values = np.arange(0, curve.length(), curve.length() / number_of_samples, dtype=np.float64)
        print(colored(f"Planning with {self.PLANNING}, pf: {p.pf},  duration {self.PLANNING_DURATION} s", "red"))
        xy = np.zeros((values.size, 2))
        dxdy = np.zeros((values.size, 2))
        theta = np.zeros((values.size))
        dtheta = np.zeros((values.size))
        for i in range(values.size):
            xy[i, :] = curve.eval(values[i])
            theta[i] = curve.theta(values[i])
            dxdy[i, :] = curve.eval_D(values[i])
            dtheta[i] = curve.theta_D(values[i])
        # map to time (only velocities are affected)
        dxdy_t = dxdy * long_vel
        omega_vec = dtheta * long_vel
        long_v_vec = np.ones((values.size))*long_vel
        return xy[:,0] , xy[:,1], theta, long_v_vec, omega_vec , dt

    def getTrajFromMatlab(self):
        try:
            print(colored("Calling Matlab service /optim: remember to  set correct  friction_coeff in robot_params.m", "red"))
            ros.wait_for_service('/optim', timeout=120.)
            self.optim_client = ros.ServiceProxy('/optim', Optim)
            request_optim = OptimRequest()
            request_optim.x0 = self.p0[0]
            request_optim.y0 = self.p0[1]
            request_optim.theta0 = self.p0[2]
            request_optim.xf = self.pf[0]
            request_optim.yf = self.pf[1]
            request_optim.thetaf = self.pf[2]
            request_optim.vmax = self.PLANNING_SPEED
            request_optim.plan_type = self.PLANNING
            response = self.optim_client(request_optim)
            self.PLANNING_DURATION = response.tf
            print(colored(f"Planning with {request_optim.plan_type} to pf:{self.pf} and duration {self.PLANNING_DURATION}", "red"))
            #print(response.des_x[-10:])
            # print(response.des_y[-10:0])
            # print(response.des_theta[-10:])
            # print(response.des_v[-10:])
            # print(response.des_omega[-10:])
            return response.des_x,response.des_y,response.des_theta, response.des_v, response.des_omega, response.dt

        except:
            print(colored("Matlab service call /optim not available, run ros_node in dubins_optimization", "red"))
            sys.exit()

    def get_command_vel(self, msg):
        self.v_d = msg.linear.x
        self.omega_d = msg.angular.z

    def deregister_node(self):
        if self.SIMULATOR == 'coppelia':
            self.coppeliaManager.simulationControl('stop')
        os.system("killall rosmaster gzserver gzclient rviz coppeliaSim")
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
            if self.SIMULATOR=='distributed3d':
                self.terrain_consistent_pose_init=np.array([self.p0[0], self.p0[1], 0, 0, 0, 0])
                if self.TERRAIN: #ramp and mesh
                    start_position, start_roll, start_pitch, start_yaw = p.terrainManager.project_on_mesh(point=self.terrain_consistent_pose_init[:2], direction=np.array([0., 0., 1.]))
                    self.terrain_consistent_pose_init[:3] = start_position.copy()
                    self.terrain_consistent_pose_init[3] = start_roll
                    self.terrain_consistent_pose_init[4] = start_pitch
                    self.terrain_consistent_pose_init[5] = start_yaw
                    # init com self.vehicle_param.height above ground
                    w_R_terr = p.math_utils.eul2Rot(np.array([start_roll, start_pitch, start_yaw]))
                    self.terrain_consistent_pose_init[:3] += self.tracked_vehicle_simulator.consider_robot_height * (w_R_terr[:, 2] * self.tracked_vehicle_simulator.vehicle_param.height)
                else:
                    self.terrain_consistent_pose_init[:3] += self.tracked_vehicle_simulator.consider_robot_height * (np.array([0.,0.,1.])* self.tracked_vehicle_simulator.vehicle_param.height)
                self.tracked_vehicle_simulator.initSimulation(pose_init=self.terrain_consistent_pose_init, twist_init=np.zeros(6), ros_pub = self.ros_pub)
                # important, you need to reset also baseState otherwise robot_state the first time will be set to 0,0,0!
                self.basePoseW = self.terrain_consistent_pose_init.copy()
            else:
                self.tracked_vehicle_simulator.initSimulation(pose_init=self.p0, vbody_init=np.array([0, 0, 0.0]))
                # important, you need to reset also baseState otherwise robot_state the first time will be set to 0,0,0!
                self.basePoseW[self.u.sp_crd["LX"]] = self.p0[0]
                self.basePoseW[self.u.sp_crd["LY"]] = self.p0[1]
                self.basePoseW[self.u.sp_crd["LZ"]] = self.tracked_vehicle_simulator.tracked_robot.vehicle_param.height  # fixed height TODO change this when on slopes
                self.basePoseW[self.u.sp_crd["AZ"]] = self.p0[2]

            self.broadcast_world = False
            self.slow_down_factor = 2

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
            if self.SIMULATOR == 'distributed3d': #not the roll and pitch are not meaningful because we are not tracking the yaw of the terrain so they are assosiated to a different yaw
                plotFrame('position', time_log=p.time_log, des_Pose_log=p.basePoseW_des_log, Pose_log=p.basePoseW_log, title='states', frame='W')
            else:
                plotFrameLinear(name='position',time_log=p.time_log,des_Pose_log = p.des_state_log, Pose_log=p.state_log, custom_labels=(["X","Y","THETA"]))
                #plotFrameLinear(name='velocity', time_log=p.time_log, Twist_log=np.vstack((p.baseTwistW_log[:2,:],p.baseTwistW_log[5,:])))

            if self.SIMULATOR != 'gazebo':
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


    def   mapToWheels(self, v_des,omega_des):
        #
        # # SAFE CHECK -> clipping velocities
        # v = np.clip(v, -constants.MAX_LINEAR_VELOCITY, constants.MAX_LINEAR_VELOCITY)
        # o = np.clip(o, -constants.MAX_ANGULAR_VELOCITY, constants.MAX_ANGULAR_VELOCITY)
        #no longer needed
        # if self.SIMULATOR=='distributed3d':
        #     self.w_R_b = self.math_utils.eul2Rot(self.euler)
        #     self.hf_R_b = self.math_utils.eul2Rot(np.array([self.euler[0],self.euler[1], 0.]))
        #     # project v_des which is in Horizontal frame onto hf_x_b
        #     v_des = self.hf_R_b[0].dot(np.array([v_des, 0., 0.]))
        #     # project omega_des which is in WF  onto w_z_b
        #     omega_des = self.w_R_b[2].dot(np.array([0., 0.,omega_des]))

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
        # OPEN LOOP wl , wr (from -IDENT_MAX_WHEEL_SPEED to IDENT_MAX_WHEEL_SPEED)
        ####################################

        if self.SIMULATOR=='distributed3d':
            number_of_samples = int(np.floor(10./conf.robot_params[p.robot_name]['dt']))
            wheel_l_vec = np.linspace(wheel_l, wheel_l, 3*number_of_samples)
            wheel_r_vec = np.linspace(0, self.IDENT_MAX_WHEEL_SPEED, number_of_samples)  # it if passes from 0 for some reason there is a non linear
            wheel_r_vec = np.append(wheel_r_vec, np.linspace(self.IDENT_MAX_WHEEL_SPEED, -self.IDENT_MAX_WHEEL_SPEED, 2*number_of_samples))
        else:
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

        if self.SIMULATOR=='distributed3d':
            # project twist from wf to bf
            w_R_b = self.math_utils.eul2Rot(self.u.angPart(self.basePoseW))
            b_lin_vel = w_R_b.T.dot(self.u.linPart(W_baseTwist))
            b_ang_vel = w_R_b.T.dot(self.u.angPart(W_baseTwist))
            b_vel_xy = b_lin_vel[:2]
            omega = b_ang_vel[2]
        else:
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

        return  tau_g, F_l, F_r, F_lx, F_rx

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


        qd_comp = np.zeros(2)
        qd_comp[0] = 1 / constants.SPROCKET_RADIUS * v_enc_l
        qd_comp[1] = 1 / constants.SPROCKET_RADIUS * v_enc_r
        return qd_comp, beta_l, beta_r
    
    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
        if self.NAVIGATION=='none':
            self.checkLoopFrequency()

        # Publish clock to have ros.Time.now sync with self.time
        #self.clock_pub.publish(Clock(clock=ros.Time.from_sec(self.time)))

        # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
        msg = JointState()
        msg.name = self.joint_names
        msg.header.stamp = ros.Time.from_sec(self.time)
        msg.position = q_des
        msg.velocity = qd_des
        msg.effort = tau_ffwd
        if np.mod(self.pub_counter, self.decimate_publish) == 0:
            self.pub_des_jstate.publish(msg) #publish in /commands

        # I comment because it slows loop down TODO
        if np.mod(self.pub_counter, self.decimate_publish) == 0:
            self.joint_pub.publish(msg)  # this publishes in tractor/joint_state q = q_des, it is just for rviz to see the joints of the wheels moving

        #trigger simulators
        if self.SIMULATOR == 'distributed2d' or self.SIMULATOR == 'distributed3d': #TODO implement torque control
            if self.ControlType != 'OPEN_LOOP' and self.LONG_SLIP_COMPENSATION  != 'NONE':
                if np.any(qd_des > np.array([constants.MAXSPEED_RADS_PULLEY, constants.MAXSPEED_RADS_PULLEY])) or np.any(qd_des < -np.array([constants.MAXSPEED_RADS_PULLEY, constants.MAXSPEED_RADS_PULLEY])):
                    print(colored("wheel speed beyond limits, NN might do wrong predictions", "red"))


            if self.SIMULATOR=='distributed3d':
                if self.TERRAIN:
                    pg, terrain_roll, terrain_pitch, terrain_yaw = self.terrainManager.project_on_mesh(point=self.basePoseW[:2], direction=np.array([0., 0., 1.]))
                    pose_des, terrain_roll_des, terrain_pitch_des, terrain_yaw_des = self.terrainManager.project_on_mesh(point=np.array([self.des_x, self.des_y]), direction=np.array([0., 0., 1.]))
                    w_R_terr = self.math_utils.eul2Rot(np.array([terrain_roll, terrain_pitch, terrain_yaw]))
                    w_normal = w_R_terr.dot(np.array([0, 0, 1]))

                    # self.ros_pub.add_arrow(pg, w_normal * 0.5, color="white")
                    # self.ros_pub.add_marker(pg, radius=0.1, color="white", alpha=1.)

                else:
                    terrain_roll = terrain_roll_des = 0.
                    terrain_pitch = terrain_pitch_des =  0.
                    terrain_yaw = 0.
                    pg = np.array([self.basePoseW[0], self.basePoseW[1], 0.])
                    pose_des = np.array([p.des_x, p.des_y, pg[2]])

                self.b_eox, self.b_eoy =self.tracked_vehicle_simulator.simulateOneStep(pg,  terrain_roll,  terrain_pitch, terrain_yaw, qd_des[0], qd_des[1], self.F_lx, self.F_rx)
                self.basePoseW, self.baseTwistW = self.tracked_vehicle_simulator.getRobotState()
                # shift up  of robot height along Zb component
                pose_des += self.tracked_vehicle_simulator.consider_robot_height * self.tracked_vehicle_simulator.w_com_height_vector
                self.basePoseW_des = np.concatenate((pose_des, np.array([terrain_roll_des, terrain_pitch_des, self.des_theta])))


            else:
                self.tracked_vehicle_simulator.simulateOneStep(qd_des[0], qd_des[1])
                pose, pose_der =  self.tracked_vehicle_simulator.getRobotState()
                #fill in base state
                self.basePoseW[:2] = pose[:2]
                self.basePoseW[self.u.sp_crd["AZ"]] = pose[2]
                self.baseTwistW[:2] = pose_der[:2]
                self.baseTwistW[self.u.sp_crd["AZ"]] = pose_der[2]

            self.euler = self.u.angPart(self.basePoseW)
            self.quaternion = pin.Quaternion(pin.rpy.rpyToMatrix(self.euler))
            self.b_R_w = self.math_utils.eul2Rot(self.euler).T
            #publish TF for rviz
            self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
                                           self.quaternion,
                                           ros.Time.now(), '/base_link', '/world')
            if self.IDENT_TYPE!='NONE':
                if np.mod(self.pub_counter, self.decimate_publish) == 0:
                    self.pub_odom_msg(self.groundtruth_pub) #this is to publish on the topic groundtruth if somebody needs it

            self.q = q_des.copy()
            self.qd = qd_des.copy()


            if self.NAVIGATION!='none' and self.SIMULATOR!='gazebo': #for distributed models set the lidar position in gazebo consistent with the robot motion
                self.setModelState('lidar', self.u.linPart(self.basePoseW), self.quaternion)

        if self.TERRAIN: #this is published to show mesh in rviz
            if self.IDENT_TYPE=='WHEELS' and self.SIMULATOR=='distributed3d':
                self.ros_pub.add_plane(pos=np.array([0,0,-0.]), orient=np.array([0., self.RAMP_INCLINATION, 0]), color="white", alpha=0.5)
            else:
                self.ros_pub.add_mesh("tractor_description", "/meshes/terrain.stl", position=np.array([0., 0., 0.0]), color="red", alpha=1.0)

        if self.SIMULATOR == 'coppelia':
            self.coppeliaManager.simulationControl('trigger_next_step')
            self.unwrap() #coppelia joints requires unwrap



        #this is for wolf navigation # gazebo publishes world, base controller publishes base link, we need to add two static transforms from world to odom and map
        if p.NAVIGATION != 'none':
            self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
                                           self.quaternion,
                                           ros.Time.now(), '/base_link', '/odom')
            #publish horizontal frame for stabilizer
            self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
                                           pin.Quaternion(pin.rpy.rpyToMatrix(np.array([0., 0., self.euler[2]]))),
                                           ros.Time.now(), '/horizontal_frame', '/world') #for 3d nav
            self.broadcaster.sendTransform(np.zeros(3),
                                           pin.Quaternion(pin.rpy.rpyToMatrix(np.array([0., 0., 0]))),
                                           ros.Time.now(), '/base_footprint', '/horizontal_frame')  # for 3d nav


            sendStaticTransform("odom", "world", np.zeros(3), np.array([1, 0, 0, 0]))  # this is just to not  brake the locosim rviz that still wants world
            self.pub_odom_msg(self.odom_pub)

        if np.mod(self.time,1) == 0:
            if not self.STATISTICAL_ANALYSIS:
                print(colored(f"TIME: {self.time}","red"))
        self.pub_counter+=1

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

    def genUniformRandomTarget(self, radius_min, radius_max):
        radius = radius_min + radius_max * np.random.uniform(low=0, high=1, size=1)
        phi = np.random.uniform(low=0, high=2*np.pi, size=1)
        theta = np.random.uniform(low=0, high=2*np.pi, size=1)
        return np.array([radius.item()*np.cos(phi).item(), radius.item()*np.sin(phi).item(), theta.item()])

    def genQuasiMontecarloRandomTarget(self, radius_min, radius_max):
        if not hasattr(self, 'sampler'):
            from scipy.stats import qmc
            self.sampler = qmc.Halton(d=3, scramble=True, seed=0)
            # since we are going to remove the samples out of the circle I need to generate a bit more samples
            n_samples = int(np.floor(100 * math.sqrt(2)))
            self.sample = self.sampler.random(n=n_samples)
            xy = radius_max * (2 * self.sample[:, :2] - 1)
            self.sample_in_circ = []
            self.phi = []
            # remove sample out of the circle
            for i in range(self.sample.shape[0]):
                radius = np.linalg.norm(xy[i, :])
                if (radius > radius_min) and (radius < radius_max):
                    self.sample_in_circ.append(xy[i, :])
                    self.phi.append(self.sample[i, 2] * 2 * np.pi)
        xy_sample = self.sample_in_circ.pop(0)
        phi_sample = self.phi.pop(0)
        return np.array([xy_sample[0], xy_sample[1], phi_sample])

def talker(p):
    p.start()
    p.startSimulator()

    if p.ControlType == "OPEN_LOOP" and p.IDENT_TYPE == 'WHEELS':
        wheel_l = np.linspace(-p.IDENT_MAX_WHEEL_SPEED, p.IDENT_MAX_WHEEL_SPEED, 32)
        ramps = np.linspace(0.0, -0.3, 5) #I use negative ramp inclination otherwise the terrain consistent startyaw is PI and not 0
        if p.SIMULATOR == 'distributed3d':
            # p.IDENT_WHEEL_L =  3
            # p.RAMP_INCLINATION = 0.2
            # main_loop(p)
            for inclination in range(len(ramps)):
                p.RAMP_INCLINATION = ramps[inclination]
                print(colored(f"Ident with inclination {p.RAMP_INCLINATION}", "red"))
                for speed in range(len(wheel_l)):
                    p.IDENT_WHEEL_L = wheel_l[speed]
                    main_loop(p)
        else:
            for speed in range(len(wheel_l)):
                p.IDENT_WHEEL_L = wheel_l[speed]
                main_loop(p)
    elif p.STATISTICAL_ANALYSIS:
        print(colored('CREATING NEW CSV TO STORE  TESTS', 'blue'))
        columns = [ 'test', 'target_x','target_y','target_theta', 'exy', 'etheta', 'tf']
        p.df = pd.DataFrame(columns=columns)
        np.random.seed(0) #create always the same random sequence
        for p.test in range(100):

            p.p0 = np.zeros(3)
            # p.pf = p.genUniformRandomTarget(1., 4.) #no longer used
            p.pf = p.genQuasiMontecarloRandomTarget(1., 4.)
            main_loop(p)
            # xy plot
            plt.figure()
            plt.plot(p.des_state_log[0, :], p.des_state_log[1, :], "-r", label="desired")
            plt.plot(p.state_log[0, :], p.state_log[1, :], "-b", label="real")
            plt.legend()
            plt.title(f"Control: {p.ControlType}, Long: {p.LONG_SLIP_COMPENSATION} Side: {p.SIDE_SLIP_COMPENSATION}")
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

    else:
        main_loop(p)

def main_loop(p):
    p.loadModelAndPublishers()
    p.u.putIntoGlobalParamServer("use_sim_time", True)
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
            # check the buffer size is big enough
            if  traj_length>conf.robot_params[p.robot_name]['buffer_size']:
                print(colored("Buffer size is not big enough for the ID!"))
                sys.exit()
        if p.PLANNING == 'none':
            if p.SIMULATOR == 'distributed3d':
                p.des_x = p.terrain_consistent_pose_init[0]  # +0.1
                p.des_y = p.terrain_consistent_pose_init[1]  # +0.1
                p.des_theta = p.terrain_consistent_pose_init[5]  # +0.1
            else:
                p.des_x = p.p0[0]  # +0.1
                p.des_y = p.p0[1]  # +0.1
                p.des_theta = p.p0[2]  # +0.1
            p.traj = Trajectory(ModelsList.UNICYCLE, p.des_x, p.des_y, p.des_theta, DT=conf.robot_params[p.robot_name]['dt'], v=v_ol, omega=omega_ol)
        else: #matlab or clothoids
            if p.PLANNING=='clothoids':
                des_x_vec, des_y_vec,des_theta_vec, v_ol, omega_ol, plan_dt= p.getClothoids(long_vel=0.4, dt = 0.001)
            else: #matlab planning
                des_x_vec, des_y_vec,des_theta_vec, v_ol, omega_ol, plan_dt=  p.getTrajFromMatlab()
            #
            p.traj = Trajectory(None, des_x_vec, des_y_vec,des_theta_vec, None, DT=plan_dt, v=v_ol, omega=omega_ol)
            traj_length = len(v_ol)

        while not ros.is_shutdown():
            if p.IDENT_TYPE == 'WHEELS':
                if counter>=traj_length:
                    print(colored("Open loop test accomplished", "red"))
                    break
                p.qd_des = np.array([wheel_l_ol[counter], wheel_r_ol[counter]])
                counter += 1
            else:
                _, _, _, p.v_d, p.omega_d, _, _, traj_finished = p.traj.evalTraj(p.time)
                p.qd_des = p.mapToWheels(p.v_d, p.omega_d)
                if traj_finished:
                    break
            #forward_speed = 1. #max speed is 4.56 rad/s
            #p.qd_des = forward_speed
            if p.torque_control:
                p.tau_ffwd = 30*conf.robot_params[p.robot_name]['kp'] * np.subtract(p.q_des, p.q) -2* conf.robot_params[p.robot_name]['kd'] * p.qd
            else:
                p.tau_ffwd = np.zeros(p.robot.na)

            p.q_des = p.q_des + p.qd_des * conf.robot_params[p.robot_name]['dt']

            if p.GRAVITY_COMPENSATION:
                p.tau_ffwd, p.F_l, p.F_r,p.F_lx,p.F_rx = p.computeGravityCompensation(p.basePoseW[p.u.sp_crd["AX"]], p.basePoseW[p.u.sp_crd["AY"]])
                #w_center_track_left = p.b_R_w.T.dot(0.5 * (constants.b_left_track_start + constants.b_left_track_end))
                #w_center_track_right = p.b_R_w.T.dot(0.5 * (constants.b_right_track_start + constants.b_right_track_end))
                #p.ros_pub.add_arrow(p.basePoseW[:3] + w_center_track_left, p.F_l / np.linalg.norm(p.F_l), "red")
                #p.ros_pub.add_arrow(p.basePoseW[:3] + w_center_track_right, p.F_r / np.linalg.norm(p.F_r), "red")
            else:
                p.F_lx = 0
                p.F_rx = 0
            p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, p.v_dot_d, p.omega_dot_d, _ = p.traj.evalTraj(p.time)
            #note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            #senting it to be tracked from the impedance loop
            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
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
        vel_gen = VelocityGenerator(simulation_time=20.,    DT=conf.robot_params[p.robot_name]['dt'])
        if p.PLANNING == 'none':
            if p.SIMULATOR=='distributed3d':
                p.des_x = p.terrain_consistent_pose_init[0]  # +0.1
                p.des_y = p.terrain_consistent_pose_init[1]  # +0.1
                p.des_theta = p.terrain_consistent_pose_init[5]  # +0.1
            else:
                p.des_x = p.p0[0]
                p.des_y = p.p0[1]
                p.des_theta = p.p0[2]
                # for PAPER user_defined_reference
                # p.des_x = 0
                # p.des_y = 0
                # p.des_theta = 0
            if p.friction_coefficient == 0.1:
                v_ol, omega_ol, v_dot_ol, omega_dot_ol, _ = vel_gen.velocity_mir_smooth(v_max_=0.2, omega_max_=0.3)
            if p.friction_coefficient == 0.4:
                v_ol, omega_ol, v_dot_ol, omega_dot_ol, _ = vel_gen.velocity_mir_smooth(v_max_=0.4, omega_max_=0.2)
            if p.friction_coefficient == 0.6:
                v_ol, omega_ol, v_dot_ol, omega_dot_ol, _ = vel_gen.velocity_mir_smooth(v_max_=0.6, omega_max_=0.4)
            p.traj = Trajectory(ModelsList.UNICYCLE, start_x=p.des_x, start_y=p.des_y, start_theta=p.des_theta, DT=conf.robot_params[p.robot_name]['dt'],
                                v=v_ol, omega=omega_ol, v_dot=v_dot_ol, omega_dot=omega_dot_ol)
        else:
            if p.PLANNING == 'clothoids':
                # from base_controllers.utils.profiler import Profiler
                # profiler = Profiler(function_name=p.getClothoids)
                des_x_vec, des_y_vec, des_theta_vec, v_ol, omega_ol, plan_dt = p.getClothoids(long_vel=0.4, dt = 0.001)
                #print(colored(f"Computation time per Clothoid call: {profiler.get_total_time()} seconds", "red"))
            else:  # matlab planning
                des_x_vec, des_y_vec, des_theta_vec, v_ol, omega_ol, plan_dt = p.getTrajFromMatlab()
            p.traj = Trajectory(None, des_x_vec, des_y_vec, des_theta_vec, None, DT=plan_dt, v=v_ol, omega=omega_ol)

        # Lyapunov controller parameters
        params = LyapunovParams(K_P=10., K_THETA=1., DT=conf.robot_params[p.robot_name]['dt'], ESTIMATE_ALPHA_WITH_ACTUAL_VALUES=p.ESTIMATE_ALPHA_WITH_ACTUAL_VALUES) #high gains 15 5 / low gains 10 1 (default)
        p.controller = LyapunovController(params=params)#, matlab_engine = p.eng)
        p.controller.setSideSlipCompensationType(p.SIDE_SLIP_COMPENSATION)
        p.controller.setSlippageInferenceType(p.SLIPPAGE_INFERENCE_TYPE)
        p.traj.set_initial_time(start_time=p.time)
        while not ros.is_shutdown():
            # update kinematics
            if p.SIMULATOR == 'distributed3d':
                robot_state.x = p.basePoseW[p.u.sp_crd["LX"]]
                robot_state.y = p.basePoseW[p.u.sp_crd["LY"]]
                robot_state.z = p.basePoseW[p.u.sp_crd["LY"]]
                robot_state.roll = p.basePoseW[p.u.sp_crd["AX"]]
                robot_state.pitch = p.basePoseW[p.u.sp_crd["AY"]]
                robot_state.theta = p.basePoseW[p.u.sp_crd["AZ"]]
            else:
                robot_state.x = p.basePoseW[p.u.sp_crd["LX"]]
                robot_state.y = p.basePoseW[p.u.sp_crd["LY"]]
                robot_state.theta = p.basePoseW[p.u.sp_crd["AZ"]]
            #print(f"pos X: {robot.x} Y: {robot.y} th: {robot.theta}")
            # controllers
            if p.NAVIGATION !='none':
                p.des_x, p.des_y, p.des_theta = p.traj.getSingleUpdate(p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d)
                p.v_dot_d = 0.
                p.omega_dot_d = 0.
                traj_finished = None
            else:

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

            p.qd_des = p.mapToWheels(p.ctrl_v, p.ctrl_omega)

            if not p.ControlType=='CLOSED_LOOP_UNICYCLE'  and not traj_finished:
                if p.LONG_SLIP_COMPENSATION=='MACHINE_LEARNING':
                    p.qd_des, p.beta_l_control, p.beta_r_control = p.computeLongSlipCompensationMachineLearning(p.qd_des, constants)
                if p.LONG_SLIP_COMPENSATION == 'EXP':
                    p.qd_des, p.beta_l_control, p.beta_r_control = p.computeLongSlipCompensationExp(p.ctrl_v, p.ctrl_omega, p.qd_des, constants)

            if p.ADD_NOISE:
                p.qd_des += np.random.normal(0, 0.02)

            # note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            # senting it to be tracked from the impedance loop
            p.q_des = p.q_des + p.qd_des * conf.robot_params[p.robot_name]['dt']
            if p.GRAVITY_COMPENSATION:
                p.tau_g, p.F_l, p.F_r ,p.F_lx, p.F_rx= p.computeGravityCompensation(p.basePoseW[p.u.sp_crd["AX"]], p.basePoseW[p.u.sp_crd["AY"]])
                # w_center_track_left = p.b_R_w.T.dot(0.5*(constants.b_left_track_start+constants.b_left_track_end))
                # w_center_track_right = p.b_R_w.T.dot(0.5 * (constants.b_right_track_start + constants.b_right_track_end))
                # p.ros_pub.add_arrow(p.basePoseW[:3] + w_center_track_left, p.F_l/np.linalg.norm(p.F_l), "red")
                # p.ros_pub.add_arrow(p.basePoseW[:3] + w_center_track_right, p.F_r/np.linalg.norm(p.F_r), "red")
            else:
                p.F_lx=0
                p.F_rx = 0

            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
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
        if p.SIMULATOR == 'distributed3d':
            # Save to CSV
            output_file = os.environ['LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/regressor/data3d/' + \
                          f"ident_wheels_fr_{p.friction_coefficient}_ramp_{p.RAMP_INCLINATION}_wheelL_{p.IDENT_WHEEL_L}.csv"
        else:
            output_file = os.environ['LOCOSIM_DIR'] + '/robot_control/base_controllers/tracked_robot/regressor/data2d/' + \
                          f"ident_wheels_fr_{p.friction_coefficient}_wheelL_{p.IDENT_WHEEL_L}.csv"
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

    if  p.PLANNING!='none' and p.ControlType!='OPEN_LOOP':
        p.log_e_x, p.log_e_y, p.log_e_theta = p.controller.getErrors()
        e_xy = np.sqrt(np.power(p.log_e_x, 2) + np.power(p.log_e_y, 2))
        rmse_xy = np.sqrt(np.mean(e_xy ** 2))
        rmse_theta = np.sqrt(np.mean(np.array(p.log_e_theta) ** 2))
        print(colored(f"Target: {p.pf.reshape(1,3)}, e_xy: {rmse_xy} e_theta {rmse_theta}","red"))
        if p.STATISTICAL_ANALYSIS:
            dict = {'test':p.test, 'target_x': p.pf[0],'target_y': p.pf[1],'target_theta': p.pf[2], 'exy': rmse_xy, 'etheta': rmse_theta, 'tf':p.PLANNING_DURATION}
            df_dict = pd.DataFrame([dict])
            p.df = pd.concat([p.df, df_dict], ignore_index=True)
            p.df.to_csv(f'statistic_{p.ControlType}_{p.PLANNING}.csv', index=None)


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



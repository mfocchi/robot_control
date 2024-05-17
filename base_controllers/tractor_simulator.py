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
from base_controllers.utils.common_functions import checkRosMaster, plotFrameLinear, plotJoint
import params as conf
import os
from std_msgs.msg import Bool, Int32, Float32
import rospkg
import roslaunch
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest
from numpy import nan
from matplotlib import pyplot as plt
from base_controllers.utils.math_tools import unwrap_angle
from  base_controllers.doretta.utils import constants as constants
from  base_controllers.doretta.models.unicycle import Unicycle
from base_controllers.doretta.controllers.lyapunov import LyapunovController, LyapunovParams, Robot
from  base_controllers.doretta.environment.trajectory import Trajectory, ModelsList
from base_controllers.doretta.velocity_generator import VelocityGenerator
from termcolor import colored
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
from sensor_msgs.msg import JointState

robotName = "tractor" # needs to inherit BaseController

class GenericSimulator(BaseController):
    
    def __init__(self, robot_name="tractor"):
        super().__init__(robot_name=robot_name, external_conf = conf)
        self.torque_control = False
        print("Initialized tractor controller---------------------------------------------------------------")
        self.GAZEBO = False
        self.ControlType = 'CLOSED_LOOP' #'OPEN_LOOP'
        self.SAVE_BAGS = False
        self.SLIPPAGE_CONTROL = False

    def initVars(self):
        super().initVars()
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

        self.state_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.des_state_log = np.full((3, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.beta_l_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.beta_r_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.alpha_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.alpha_control_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.radius_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan

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

                self.beta_l_log[self.log_counter] = self.beta_l
                self.beta_r_log[self.log_counter] = self.beta_r
                self.alpha_log[self.log_counter] = self.alpha
                self.alpha_control_log[self.log_counter] = self.alpha_control
                self.radius_log[self.log_counter] = self.radius
            super().logData()

    def startSimulator(self):
        if self.GAZEBO:
            #world_name = tractor.world
            super().startSimulator(world_name=None, additional_args=['spawn_Y:=0.'])
        else:
            print(colored(
                f"---------To run this you need to clone https://github.com/mfocchi/CoppeliaSim inside locosim folder",
                "red"))
            # clean up previous process
            os.system("killall rosmaster rviz coppeliaSim")
            # launch roscore
            checkRosMaster()
            ros.sleep(1.5)

            # launch coppeliasim
            scene = rospkg.RosPack().get_path('tractor_description') + '/CoppeliaSimModels/tractor_ros.ttt'
            file = os.getenv("LOCOSIM_DIR")+"/CoppeliaSim/coppeliaSim.sh"+" "+scene+" &"
            os.system(file)


            # run robot state publisher + load robot description + rviz
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('tractor_description')+"/launch/rviz_nojoints.launch"])
            launch.start()

            # launch separately
            # package = 'robot_state_publisher'
            # executable = 'robot_state_publisher'
            # node = roslaunch.core.Node(package, executable)
            # launch = roslaunch.scriptapi.ROSLaunch()
            # launch.start()
            # process = launch.launch(node)
            #
            # # run rviz
            # package = 'rviz'
            # executable = 'rviz'
            # args = '-d ' + rospkg.RosPack().get_path('ros_impedance_controller') + '/config/operator.rviz'
            # node = roslaunch.core.Node(package, executable, args=args)
            # launch = roslaunch.scriptapi.ROSLaunch()
            # launch.start()
            # process = launch.launch(node)

            # wait for coppelia to start
            ros.sleep(1.5)



    def loadModelAndPublishers(self):
        super().loadModelAndPublishers()
        self.reset_joints_client = ros.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.des_vel = ros.Publisher("/des_vel", JointState, queue_size=1, tcp_nodelay=True)
        if self.SAVE_BAGS:
            self.recorder = RosbagControlledRecorder('rosbag record -a', False)

        if not self.GAZEBO:
            self.startPub=ros.Publisher("/startSimulation", Bool, queue_size=1, tcp_nodelay=True)
            self.pausePub=ros.Publisher("/pauseSimulation", Bool, queue_size=1, tcp_nodelay=True)
            self.stopPub=ros.Publisher("/stopSimulation", Bool, queue_size=1, tcp_nodelay=True)
            self.enableSyncModePub=ros.Publisher("/enableSyncMode", Bool, queue_size=1, tcp_nodelay=True)
            self.triggerNextStepPub=ros.Publisher("/triggerNextStep", Bool, queue_size=1, tcp_nodelay=True)
            self.renderSimulationPub = ros.Publisher("/renderSimulation", Bool, queue_size=1, tcp_nodelay=True)
            # simStepDoneSub=ros.Subscriber("/simulationStepDone", Bool, sim_done_)
            self.simStateSub=ros.Subscriber("/simulationState", Int32, self.sim_state_)
            # simTimeSub=simROS.Subscriber('/simulationTime',Float32, time_sim_)


    def sim_state_(self, msg):
        pass

    def deregister_node(self):
        if not self.GAZEBO:
            self.simulationControl('stop')
        os.system("killall rosmaster rviz coppeliaSim")
        super().deregister_node()

    def simulationControl(self,input):
        if input == 'start':
            print("starting simulation")
            ros.wait_for_message('/simulationState', Bool, timeout=5.)
            ros.sleep(1.5)
            msg = Bool()
            msg.data = True
            for i in range(30):
                self.startPub.publish(msg)
        if input == 'stop':
            self.stopPub.publish(True)
        if input == 'enable_sync_mode':
            self.enableSyncModePub.publish(True)
        if input=='trigger_next_step':
            self.triggerNextStepPub.publish(True)
        if input=='gui_off':
            self.renderSimulationPub.publish(True)

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

            self.simulationControl('start')
            # start coppeliasim
            self.simulationControl('enable_sync_mode')
            #Coppelia Runs with increment of 0.01 but is not able to run at 100Hz but at 25 hz so I "slow down" ros, but still update time with dt = 0.01
            slow_down_factor = 8
            # loop frequency
            self.rate = ros.Rate(1 / (slow_down_factor * conf.robot_params[p.robot_name]['dt']))


    def plotData(self):
        if conf.plotting:
            plotJoint('position', p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, joint_names=p.joint_names)
            plotJoint('velocity', p.time_log, qd_log=p.qd_log, qd_des_log=p.qd_des_log, joint_names=p.joint_names)
            plotFrameLinear(name='position',time_log=p.time_log,des_Pose_log = p.des_state_log, Pose_log=p.state_log)
            plotFrameLinear(name='velocity', time_log=p.time_log, Twist_log=p.baseTwistW_log[:3,:])

            #tracking errors
            self.log_e_x, self.log_e_y,  self.log_e_theta = self.controller.getErrors()
            plt.figure()
            plt.subplot(3, 1, 1)
            plt.plot(self.log_e_x, "-b")
            plt.ylabel("ex")
            plt.grid(True)
            plt.subplot(3, 1, 2)
            plt.plot(self.log_e_y, "-b")
            plt.ylabel("ey")
            plt.grid(True)
            plt.subplot(3, 1, 3)
            plt.plot(self.log_e_y, "-b")
            plt.ylabel("eth")
            plt.grid(True)
            #slippage vars
            plt.figure()
            plt.subplot(4, 1, 1)
            plt.plot(self.time_log, self.beta_l_log, "-b")
            plt.ylabel("beta_l")
            plt.grid(True)
            plt.subplot(4, 1, 2)
            plt.plot(self.time_log, self.beta_r_log, "-b")
            plt.ylabel("beta_r")
            plt.grid(True)
            plt.subplot(4, 1, 3)
            plt.plot(self.time_log, self.alpha_log, "-b", label="real")
            plt.plot(self.time_log, self.alpha_control_log, "-r", label="control")
            plt.ylabel("alpha")
            plt.legend()
            plt.subplot(4, 1, 4)
            plt.plot(self.time_log, self.radius_log, "-b", label="des")
            plt.ylabel("alpha")
            plt.grid(True)

            if p.ControlType == 'CLOSED_LOOP':
                plt.figure()
                plt.plot(p.traj.x, p.traj.y, "-r", label="desired")
                plt.plot(p.basePoseW_log[0, :], p.basePoseW_log[1, :], "-b", label="real")
                plt.legend()
                plt.xlabel("x[m]")
                plt.ylabel("y[m]")
                plt.axis("equal")
                plt.grid(True)
                # # VELOCITY
                plt.figure()
                plt.subplot(2, 1, 1)
                plt.plot(p.time_log, p.ctrl_v_log, "-b", label="REAL")
                plt.plot(p.time_log, p.v_d_log, "-r", label="desired")
                plt.legend()
                plt.xlabel("time[sec]")
                plt.ylabel("linear velocity[m/s]")
                # plt.axis("equal")
                plt.grid(True)
                plt.subplot(2, 1, 2)
                plt.plot(p.time_log, p.ctrl_omega_log, "-b", label="REAL")
                plt.plot(p.time_log, p.omega_d_log, "-r", label="desired")
                plt.legend()
                plt.xlabel("time[sec]")
                plt.ylabel("angular velocity[rad/s]")
                # plt.axis("equal")
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

        #publish des commands
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

    def generateOpenLoopTraj(self, R_initial= 0.05, R_final=0.6, increment=0.05, dt = 0.005, long_v = 0.1):
        # only around 0.3
        change_interval = 4.
        increment = increment
        turning_radius_vec = np.arange(R_initial, R_final, increment)

        ang_w = np.round(long_v / turning_radius_vec, 3)  # [rad/s]
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
        b_vel_xy = w_R_b.T.dot(w_vel_xy)

        # track velocity  from encoder
        v_enc_l = constants.SPROCKET_RADIUS *  wheel_L
        v_enc_r = constants.SPROCKET_RADIUS *  wheel_R
        B = constants.TRACK_WIDTH

        v_track_l = b_vel_xy[0] - omega* B / 2
        v_track_r = b_vel_xy[0] + omega* B / 2

        beta_l = np.abs(v_track_l) - np.abs(v_enc_l)
        beta_r = np.abs(v_track_r) - np.abs(v_enc_r)
        side_slip = math.atan2(b_vel_xy[1],b_vel_xy[0])
        return beta_l, beta_r, side_slip
    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
         # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
         msg = JointState()
         msg.name = self.joint_names
         msg.header.stamp = ros.Time.from_sec(self.time)
         msg.position = q_des
         msg.velocity = qd_des
         msg.effort = tau_ffwd
         self.pub_des_jstate.publish(msg)

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

    if p.ControlType == 'OPEN_LOOP':
        counter = 0
        v_ol, omega_ol = p.generateOpenLoopTraj(R_initial= 0.05, R_final=0.6, increment=0.1, dt = conf.robot_params[p.robot_name]['dt'], long_v = 0.1)
        # OPEN loop control
        while not ros.is_shutdown():
            if counter<len(v_ol):
                p.qd_des = p.mapToWheels(v_ol[counter], omega_ol[counter])
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
            #note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            #senting it to be tracked from the impedance loop
            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

            if not p.GAZEBO:
                p.simulationControl('trigger_next_step')
                p.unwrap()
                # log variables
            p.logData()
            # wait for synconization of the control loop
            p.rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),  3)  # to avoid issues of dt 0.0009999
    else:
        # CLOSE loop control
        # generate reference trajectory
        vel_gen = VelocityGenerator(simulation_time=10.,    DT=conf.robot_params[p.robot_name]['dt'])
        # initial_des_x = 0.1
        # initial_des_y = 0.1
        # initial_des_theta = 0.3
        initial_des_x = 0.0
        initial_des_y = 0.0
        initial_des_theta = 0.0
        p.traj = Trajectory(ModelsList.UNICYCLE, initial_des_x, initial_des_y, initial_des_theta, vel_gen.velocity_mir_smooth, conf.robot_params[p.robot_name]['dt'])

        # Lyapunov controller parameters
        params = LyapunovParams(K_P=10., K_THETA=1.5, DT=conf.robot_params[p.robot_name]['dt'])
        p.controller = LyapunovController(params=params)
        p.traj.set_initial_time(start_time=p.time)
        #v_des, omega_des, _ = vel_gen.velocity_mir_smooth()
        #count = 0
        while not ros.is_shutdown():
            # update kinematics
            robot_state.x = p.basePoseW[p.u.sp_crd["LX"]]
            robot_state.y = p.basePoseW[p.u.sp_crd["LY"]]
            robot_state.theta = p.basePoseW[p.u.sp_crd["AZ"]]
            #print(f"pos X: {robot.x} Y: {robot.y} th: {robot.theta}")

            # controllers
            #getSingleUpdate()
            p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, traj_finished = p.traj.evalTraj(p.time)
            if p.SLIPPAGE_CONTROL:
                p.ctrl_v, p.ctrl_omega,  p.V, p.V_dot, p.alpha_control = p.controller.control_alpha0(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, traj_finished)
            else:
                p.ctrl_v, p.ctrl_omega, p.V, p.V_dot = p.controller.control_unicycle(robot_state, p.time, p.des_x, p.des_y, p.des_theta, p.v_d, p.omega_d, traj_finished)
            p.qd_des = p.mapToWheels(p.ctrl_v, p.ctrl_omega)

            # debug send openloop vels
            # p.ctrl_v = v_des[count]
            # p.ctrl_omega = omega_des[count]
            # if count < (len(v_des)-1):
            #     count+=1

            # note there is only a ros_impedance controller, not a joint_group_vel controller, so I can only set velocity by integrating the wheel speed and
            # senting it to be tracked from the impedance loop
            p.q_des = p.q_des + p.qd_des * conf.robot_params[p.robot_name]['dt']
            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

            if not p.GAZEBO:
                p.simulationControl('trigger_next_step')
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

    ros.signal_shutdown("killed")
    p.deregister_node()
    print("Plotting")
    p.plotData()



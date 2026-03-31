# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

import rospy as ros
from base_controllers.utils.math_tools import *
import pinocchio as pin
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
import matplotlib.pyplot as plt
from numpy import nan
from base_controllers.utils.common_functions import plotJoint, plotFrameLinear, spawnMesh
from termcolor import colored
import os
import tf
from base_controllers.base_controller_fixed import BaseControllerFixed
from geometry_msgs.msg import Wrench, Point
from gazebo_msgs.msg import ContactsState
import scipy.io.matlab as mio
import rospkg
from base_controllers.utils.matlab_conversions import mat_vector2python, mat_matrix2python
import matlab.engine
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
import sys
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
import  base_controllers.params as conf
robotName = "climbingrobot2"
from base_controllers.utils.common_functions import SafeTFBroadcaster
import json
from base_controllers.components.terrain_manager import TerrainManager
import pandas as pd
from base_controllers.utils.halton_walker import get_halton_samples
from orientation_controller import OrientationController
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.msg import ModelState
import subprocess

class ClimbingrobotController(BaseControllerFixed):
    def __init__(self, robot_name="ur5"):
        self.EXTERNAL_FORCE = False
        self.landing = True #do landing
        self.MPC_control = True
        self.PLOT_MPC = False
        self.PROPELLERS = True
        self.USE_PROPELLERS_FOR_LEG_REORIENT = False # true use propeller to reorient the leg
        self.SAVE_BAG = False # does not show rope vectors
        self.rope_index = np.array([2, 8]) #'wire_base_prismatic_r', 'wire_base_prismatic_l',
        self.leg_index = np.array([12, 13, 14])
        self.wheel_index = np.array([16, 18]) #'wheel_joint_l',  'wheel_joint_r'
        self.hip_pitch_joint = 12
        self.hip_roll_joint = 13
        self.base_passive_joints = np.array([3,4,5, 9,10,11])
        self.anchor_passive_joints = np.array([0,1, 6,7])
        self.OBSTACLE_AVOIDANCE = 'mesh' #'none', 'mesh'
        self.use_gui = False
        self.PAPER = False
        self.SAMPLE_FOR_VALUE_FUNCTION = False
        self.USE_ORIENTATION_CONTROL = False

        if self.MPC_control:
            sys.path.insert(0, './codegen_mpc')

        if self.OBSTACLE_AVOIDANCE=='mesh':
            sys.path.insert(0, './codegen_mesh')
        else:
            sys.path.insert(0, './codegen')

        self.force_scale = 60.
        self.mountain_thickness = 0.1 # TODO call the launch file passing this parameter
        self.r_leg = 0.3

        super().__init__(robot_name=robot_name)
        print("Initialized climbingrobot controller---------------------------------------------------------------")

    def apply_propeller_moment(self, Mz):
        # create force per to ropes plane
        arm = np.linalg.norm(self.hoist_l_pos-self.base_pos)
        force = self.w_R_b[:,0]*Mz/(2*arm)
        self.ros_pub.add_arrow(self.hoist_l_pos, force/(10*self.force_scale), "green", scale=1.5)  #left should be positive
        self.ros_pub.add_arrow(self.hoist_r_pos, -force/(10*self.force_scale), "green", scale=1.5) #right should be negative
        wrench = Wrench()
        wrench.force.x = 0.
        wrench.force.y = 0.
        wrench.force.z = 0.
        wrench.torque.x = 0.
        wrench.torque.y = 0.
        wrench.torque.z = Mz
        self.pub_prop_force.publish(wrench)

    def apply_propeller_force(self, ext_force):
        # create force per to ropes plane
        self.prop_forceW  = self.n_bar * ext_force
        self.ros_pub.add_arrow(self.base_pos, self.prop_forceW/self.force_scale , "blue", scale=1.5)
        wrench = Wrench()
        wrench.force.x = self.prop_forceW [0]
        wrench.force.y = self.prop_forceW [1]
        wrench.force.z = self.prop_forceW [2]
        wrench.torque.x = 0.
        wrench.torque.y = 0.
        wrench.torque.z = 0.
        self.pub_prop_force.publish(wrench)

    def apply_propeller_orient(self, w_wrench, prop_thrusts):
        self.ros_pub.add_arrow(self.base_pos + self.w_R_b @ self.orientControl.b_propeller_pos[0],
                               self.orientControl.b_propeller_axes[0] * prop_thrusts[0]/self.force_scale , "blue", scale=1.5)
        self.ros_pub.add_arrow(self.base_pos + self.w_R_b @ self.orientControl.b_propeller_pos[1],
                               self.orientControl.b_propeller_axes[1] * prop_thrusts[1] / self.force_scale, "blue", scale=1.5)
        self.ros_pub.add_arrow(self.base_pos + self.w_R_b @ self.orientControl.b_propeller_pos[2],
                               self.orientControl.b_propeller_axes[2] * prop_thrusts[2] / self.force_scale, "blue", scale=1.5)
        self.ros_pub.add_arrow(self.base_pos + self.w_R_b @ self.orientControl.b_propeller_pos[3],
                               self.orientControl.b_propeller_axes[3] * prop_thrusts[3] / self.force_scale, "blue", scale=1.5)
        wrench = Wrench()
        wrench.force.x =  w_wrench[0]
        wrench.force.y =  w_wrench[1]
        wrench.force.z =  w_wrench[2]
        wrench.torque.x = w_wrench[3]
        wrench.torque.y = w_wrench[4]
        wrench.torque.z = w_wrench[5]
        self.pub_prop_force.publish(wrench)

    def loadModelAndPublishers(self, xacro_path=None):
        xacro_path = rospkg.RosPack().get_path('climbingrobot_description') + '/urdf/' + self.robot_name + '.xacro'
        additional_urdf_args = ' anchorX:=' + str(conf.robot_params[self.robot_name]['spawn_x'])
        additional_urdf_args += ' anchorY:=' + str(conf.robot_params[self.robot_name]['spawn_y'])
        additional_urdf_args += ' anchorZ:=' + str(conf.robot_params[self.robot_name]['spawn_z'])
        additional_urdf_args += ' anchor2X:=' + str(conf.robot_params[self.robot_name]['spawn_2x'])
        additional_urdf_args += ' anchor2Y:=' + str(conf.robot_params[self.robot_name]['spawn_2y'])
        additional_urdf_args += ' anchor2Z:=' + str(conf.robot_params[self.robot_name]['spawn_2z'])
        super().loadModelAndPublishers(xacro_path=xacro_path, additional_urdf_args=additional_urdf_args,  markers_time_to_live=conf.robot_params[self.robot_name]['dt'])

        self.broadcaster = SafeTFBroadcaster()
        self.sub_contact= ros.Subscriber("/" + self.robot_name + "/foot_bumper", ContactsState,
                                             callback=self._receive_contact, queue_size=1, buff_size=2 ** 24,
                                             tcp_nodelay=True)
        # this is for the matlab optim
        self.eng = matlab.engine.start_matlab()

        if self.OBSTACLE_AVOIDANCE=='mesh':
            self.eng.addpath('./codegen_mesh', nargout=0)
        else:
            self.eng.addpath('./codegen', nargout=0)

        if self.MPC_control:
            self.eng.addpath('./codegen_mpc', nargout=0)

        if self.PROPELLERS:
            self.pub_prop_force = ros.Publisher("/base_force", Wrench, queue_size=1, tcp_nodelay=True)
        if self.SAVE_BAG:
            if self.PAPER:
                self.recorder = RosbagControlledRecorder(bag_name = "climbing_robot_rocky_terrain_paper.bag", record_from_startup_=False)
            else:
                self.recorder = RosbagControlledRecorder(bag_name="climbing_robot_rocky_terrain_single_jumps.bag", record_from_startup_=False)

        self.reset_base = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def getRobotMass(self):
        robot_link_masses = []
        #get link masses supported by joints
        for idx in self.robot.model.inertias:
            robot_link_masses.append(idx.mass)
        # the robot is supported after this joint
        total_robot_mass = sum(robot_link_masses[self.robot.model.getJointId('wire_base_yaw_l'):])
        return total_robot_mass

    def updateKinematicsDynamics(self):
        # q is continuously updated
        self.robot.computeAllTerms(self.q, self.qd )
        # joint space inertia matrix
        self.M = self.robot.mass(self.q )
        # bias terms
        self.h = self.robot.nle(self.q  , self.qd )
        #gravity terms
        self.g = self.robot.gravity(self.q )

        # this is expressed in a workdframe with the origin attached to the base frame origin
        self.anchor_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId('anchor')).translation
        self.anchor_pos2 = self.robot.framePlacement(self.q, self.robot.model.getFrameId('anchor_2')).translation
        self.anchor_distance_y = (self.anchor_pos2 - self.anchor_pos)[1]

        #base variables
        self.base_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).translation
        self.w_R_b = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).rotation
        self.base_rpy = self.math_utils.rot2eul(self.w_R_b)
        self.Jb = self.robot.frameJacobian(self.q, self.robot.model.getFrameId('base_link'), True,  pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        self.base_vel = self.Jb[:3, :].dot(self.qd)
        self.omega_b =  self.Jb[3:, :].dot(self.qd) #in WF

        # compute com in base frame
        robotComB = pin.centerOfMass(self.robot.model, self.robot.data, self.q)
        # from com in WF
        self.com = self.robot.robotComW(self.q)

        #compute ee position  in the world frame
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        self.x_ee =  self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation

        self.hoist_l_pos = self.base_pos +  self.w_R_b.dot(np.array([0.0, -0.05, 0.05]))
        # print("sanitycheck", self.hoist_l_pos - self.robot.framePlacement(self.q, self.robot.model.getFrameId('pre-base3')).translation)
        self.hoist_r_pos = self.base_pos + self.w_R_b.dot(np.array([0.0, 0.05, 0.05]))
        # print("sanitycheck", self.hoist_r_pos - self.robot.framePlacement(self.q, self.robot.model.getFrameId('fake_link')).translation)

        self.rope_direction = (self.hoist_l_pos - self.anchor_pos) / np.linalg.norm(self.hoist_l_pos  - self.anchor_pos)
        self.rope_direction2 = (self.hoist_r_pos - self.anchor_pos2) / np.linalg.norm(self.hoist_r_pos - self.anchor_pos2)

        # feet variable
        # compute jacobian of the end effector in the world frame (take only the linear part and the actuated joints part)
        self.J = self.robot.frameJacobian(self.q , self.robot.model.getFrameId(frame_name), True, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
        self.Jleg = self.J[:, self.leg_index]
        self.dJdq = self.robot.frameClassicAcceleration(self.q , self.qd , None,  self.robot.model.getFrameId(frame_name), False).linear

        w_R_wire = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wire')).rotation
        w_R_wire2 = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wire_2')).rotation

        self.mat2Gazebo = self.anchor_pos
        self.base_pos_mat = self.base_pos - self.mat2Gazebo
        # offset between hoists
        hoist_distance = np.linalg.norm(self.hoist_l_pos - self.hoist_r_pos)

        #compute matlab states
        self.psi = math.atan2(self.base_pos_mat[0], -self.base_pos_mat[2])
        # to get the matlab state from the gazebo prismatic joints we need to consider that the gazebo joints is in zero config
        # when the rope is 2.5 m half of anchor distance (startup at the point in the middle of the anchors)
        #print("sanitycheck", self.l_1 - np.linalg.norm((self.hoist_l_pos - self.anchor_pos)))
        # print("sanitycheck", self.l_2 - np.linalg.norm((self.hoist_r_pos - self.anchor_pos2)))
        self.l_1 = self.q[self.rope_index[1]] - hoist_distance/2 + self.anchor_distance_y/2
        self.l_2 = self.q[self.rope_index[0]] - hoist_distance/2 + self.anchor_distance_y/2

        # use geometric intuition for psid
        n_par = (self.anchor_pos - self.anchor_pos2) / np.linalg.norm(self.anchor_pos - self.anchor_pos2)
        rope2_axis = (self.base_pos - self.anchor_pos2) / np.linalg.norm(self.base_pos - self.anchor_pos2)
        self.n_bar = np.cross(n_par, rope2_axis) / np.linalg.norm(np.cross(n_par, rope2_axis))

        #compute matlab state derivatives
        self.psid = (self.n_bar.dot(self.base_vel)) / np.linalg.norm(np.cross(n_par, self.base_pos - self.anchor_pos2))
        self.l_1d = self.qd[self.rope_index[1]]
        self.l_2d = self.qd[self.rope_index[0]]

        # the mountain is always wrt to world
        mountain_pos = np.array([-self.mountain_thickness/2, conf.robot_params[self.robot_name]['spawn_y'], 0.0])
        if hasattr(self, "broadcaster"):
            self.broadcaster.sendTransform(mountain_pos, (0.0, 0.0, 0.0, 1.0), ros.Time.now(), '/wall', '/world')

    def _receive_contact(self, msg):
        self.contactForceW = np.zeros(3)
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.contactForceW = self.robot.framePlacement(self.q,  self.robot.model.getFrameId("lower_link")).rotation.dot(grf)

    def initVars(self):
        super().initVars()
        self.contactForceW_l = np.zeros(3)
        self.contactForceW_r = np.zeros(3)
        self.qdd_des =  np.zeros(self.robot.na)
        self.base_accel = np.zeros(3)
        self.base_rpy = np.zeros(3)
        self.Fr_l_fbk = 0
        self.Fr_r_fbk = 0
        self.Fr_l = 0
        self.Fr_r = 0
        self.prop_force = 0
        self.prop_thrusts = [0]*4
        self.MPC_tracking_error = []

        w_R_wall = self.math_utils.eul2Rot(np.array([0, -conf.robot_params[self.robot_name]['wall_inclination'], 0]))
        self.wall_normal = w_R_wall[:, 0].copy()  # take X axis, I need to use copy otherwise matlab complains is not contiguous

        self.touch_down_detected_prismleg = False
        self.touch_down_detected_l = False #no longer used
        self.touch_down_detected_r = False #no longer used
        self.optimal_control_traj_finished = False
        self.mpc_index = 0
        self.mpc_index_old = 0
        self.mpc_index_ffwd = 0  # updated only when we stop recomputing mpc

        # init new logged vars here
        self.com_log =  np.empty((3, conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.simp_model_state_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        #self.ldot_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.base_pos_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.base_rpy_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']
        self.time_jump_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_l_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_r_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_l_fbk_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_r_fbk_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.l_1d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.l_2d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.psid_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.base_vel_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.prop_force_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.prop_thrusts_log = np.empty((4, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        if self.USE_ORIENTATION_CONTROL:
            propeller_orient = np.array([0.25 * np.pi, 0.75 * np.pi, np.pi + 0.25 * np.pi, np.pi + 0.75 * np.pi])
            self.orientControl = OrientationController(base_line_x = 0.1, base_line_y = 0.2, propeller_orient=propeller_orient)

    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                self.simp_model_state_log[:, self.log_counter] = np.array([self.psi, self.l_1, self.l_2])
                # self.ldot_log[self.log_counter] = self.ldot
                self.base_pos_log[:, self.log_counter] = self.base_pos
                self.base_rpy_log[:, self.log_counter] = self.base_rpy
                self.Fr_l_log[self.log_counter] = self.Fr_l
                self.Fr_r_log[self.log_counter] = self.Fr_r
                self.Fr_l_fbk_log[self.log_counter] = self.Fr_l_fbk
                self.Fr_r_fbk_log[self.log_counter] = self.Fr_r_fbk
                self.l_1d_log[self.log_counter] =  self.l_1d
                self.l_2d_log[self.log_counter] =  self.l_2d
                self.psid_log[self.log_counter] = self.psid
                self.base_vel_log[:,self.log_counter] = self.base_vel
                if self.PROPELLERS:
                    self.prop_force_log[self.log_counter] = self.prop_force
                    self.prop_thrusts_log[:, self.log_counter] = self.prop_thrusts

                #self.time_jump_log[self.log_counter] = self.time - self.end_thrusting

            super().logData()

    def deregister_node(self):
        super().deregister_node()
        os.system(" rosnode kill -a")
        os.system(" rosnode kill /gzserver /gzclient")
        os.system(" pkill rosmaster")

    def startupProcedure(self):
        #set PD gains
        super().startupProcedure()

    def plotStuff(self):
        print("PLOTTING")
        print(colored("The initial p0_x and mountain_pitch can be different by the desired ones computed by optim, even if we started optim from actual p0, "
                      "because the robot sags a bit due to leg reorientation","red"))
        # plotFrameLinear('com position', 1, p.time_log, None, p.com_log)
        # plotFrameLinear('contact force', 2, p.time_log, None, p.contactForceW_log)
        actual_com= p.base_pos_log - p.mat2Gazebo.reshape(3, 1) # mat2Gazebo is WF in matlab
        plotJoint('position', p.time_log, p.q_log, p.q_des_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
        plot3D('basePos', 2,  ['X', 'Y', 'Z'], p.time_log, actual_com, p.ref_time + (p.startJump+p.orientTime), p.ref_com)
        plot3D('matlab states', 3, ['psi', 'l1', 'l2'], p.time_log, p.simp_model_state_log, p.ref_time + (p.startJump+p.orientTime), np.vstack((p.ref_psi, p.ref_l_1, p.ref_l_2)) )



        # plot rope forces
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.ylabel("Fr_l")
        plt.plot(p.ref_time + (p.startJump+p.orientTime), p.Fr_l0, color='red')
        plt.plot(p.time_log, p.Fr_l_log, color='blue')
        plt.grid()
        plt.subplot(2, 1, 2)
        plt.ylabel("Fr_r")
        plt.plot(p.ref_time + (p.startJump+p.orientTime), p.Fr_r0, label="ref", color='red')
        plt.plot(p.time_log, p.Fr_r_log, label="ref+MPC", color='blue')
        plt.legend()
        plt.grid()

        if p.USE_ORIENTATION_CONTROL:
            # plot propeller thrusts
            plt.figure()
            plt.ylabel("prop_thrusts")
            plt.subplot(4, 1, 1)
            plt.grid()
            plt.plot(p.time_log, p.prop_thrusts_log[0,:], label="prop1", color='blue')
            plt.subplot(4, 1, 2)
            plt.grid()
            plt.plot(p.time_log, p.prop_thrusts_log[1,:], label="prop2", color='blue')
            plt.subplot(4, 1, 3)
            plt.grid()
            plt.plot(p.time_log, p.prop_thrusts_log[2,:], label="prop3", color='blue')
            plt.subplot(4, 1, 4)
            plt.plot(p.time_log, p.prop_thrusts_log[3,:], label="prop4", color='blue')
            plt.legend()
            plt.grid()
            plotFrameLinear('position', time_log=p.time_log, Pose_log=p.base_rpy_log)

        #save data
        time_jump = p.time_log - (p.startJump + p.orientTime)
        filename = f'test_gazebo_MPC_{p.MPC_control}.mat'
        mio.savemat(filename, {'ref_time': p.ref_time, 'ref_com': p.ref_com,
                                'time_gazebo': time_jump, 'actual_com': actual_com,
                                'ref_psi':p.ref_psi,'ref_l_1':p.ref_l_1, 'ref_l_2':p.ref_l_2,
                                'psi': p.simp_model_state_log[0,:], 'l_1': p.simp_model_state_log[1,:], 'l_2': p.simp_model_state_log[2,:],
                                'psid': p.psid_log, 'l_1d': p.l_1d_log,'l_2d': p.l_2d_log,
                                'mu': p.mu , 'Fleg': p.Fleg,'Fr_max': p.Fr_max,
                                'Fr_l0': p.Fr_l0, 'Fr_r0': p.Fr_r0,
                                'Fr_l': p.Fr_l_log, 'Fr_r': p.Fr_r_log })

    def getIndex(self,t):
        try:
            # get index
            a_bool = self.jump_data["time"] >= t
            idx = min([i for (i, val) in enumerate(a_bool) if val])-1
            if idx == -1:
                return 0
            else:
                return idx
        except:
            return  -1

    def getImpulseAngle(self):
        angle_hip_roll =  math.atan2(self.jump_data["Fleg"][1], self.jump_data["Fleg"][0])
        angle_hip_pitch =  math.atan2(self.jump_data["Fleg"][2], self.jump_data["Fleg"][0])
        print(colored(f"Start orienting leg to (pitch, roll)  {p.time}: {angle_hip_pitch, angle_hip_roll}", "blue"))
        angle_hip_pitch +=-1.57
        return angle_hip_pitch, angle_hip_roll

    # compute the passive and rope joints reference from the matlab position referred to a world frame located in between anchors
    def computeJointVariables(self, p):
        # mountain_wire_pitch_l = math.atan2(p[0]-conf.robot_params[self.robot_name]['spawn_x'], -p[2])
        # mountain_wire_pitch_r = math.atan2(p[0]-conf.robot_params[self.robot_name]['spawn_2x'], -p[2])

        mountain_wire_pitch_l = math.atan2(p[0] , -p[2])
        mountain_wire_pitch_r = math.atan2(p[0] , -p[2])

        mountain_wire_roll_l = -math.atan2(-p[2], p[1])
        mountain_wire_roll_r = math.atan2(-p[2], self.anchor_distance_y-p[1])
        # this is an approximation cause I shuould compute the real rope lenght considering the hoist distance so this function is only useful for init but it is inaccurate!
        wire_base_prismatic_l = np.linalg.norm(p) -self.anchor_distance_y*0.5
        wire_base_prismatic_r = math.sqrt(p[0]*p[0] +(self.anchor_distance_y - p[1])*(self.anchor_distance_y - p[1]) + p[2] * p[2])-self.anchor_distance_y*0.5

        wire_base_roll_l = -mountain_wire_roll_l
        wire_base_roll_r = -mountain_wire_roll_r
        return [mountain_wire_pitch_r, mountain_wire_roll_r,  wire_base_prismatic_r, 0., wire_base_roll_r, 0.,
                mountain_wire_pitch_l, mountain_wire_roll_l,  wire_base_prismatic_l, 0., wire_base_roll_l, 0.]

    def detectTouchDown(self):
        force_th = 15.
        #old way with two landing legs
        # if not self.touch_down_detected_l and (self.wall_normal.dot(self.contactForceW_l) > force_th):
        #     self.touch_down_detected_l = True
        # if not self.touch_down_detected_r  and (self.wall_normal.dot(self.contactForceW_r) > force_th):
        #     self.touch_down_detected_r = True
        # if self.touch_down_detected_l and self.touch_down_detected_r:
        #     print(colored("TouchDown Detected", "blue"))
        #     # sample com pos
        #     self.x_tilde0 =  self.wall_normal.reshape(1, 3) @ (self.com)# - self.x_p)
        #     return True
        # else:
        #     return False

        #new way prismatic leg
        if not self.touch_down_detected_prismleg and np.linalg.norm(self.contactForceW) > force_th:
            self.touch_down_detected_prismleg = True
        if self.touch_down_detected_prismleg:
            #print(colored("TouchDown Detected", "blue"))
            return True
        else:
            return False

    def resetRobot(self, p0 = None):

        p0_adj = p0.copy()
        self.updateKinematicsDynamics()
        p0_adj[0] = self.terrainManager.wall_surface_eval(p0[2], p0[1], self.mesh_x, self.mesh_y, self.mesh_z) + 0.2  # account for leg

        print(colored(f"---------Computing Consistent Joints:", "red"))
        # self.q_des = np.copy(self.q_des_q0)
        # self.q_des[:12] = self.computeJointVariables(p0_adj)
        from closed_loop_inverse_kinematics import ClosedLoopKinSolver
        solver = ClosedLoopKinSolver(robot_name=self.robot_name)
        self.q_des = solver.computeJointVariables(p0_adj + self.mat2Gazebo, np.eye(3), self.q_des_q0, debug=False)
        for joint, name in zip(self.q_des, self.joint_names):
            print(colored(f"{name}: {joint}", "red"))




        # create model state
        reset_base_req = SetModelStateRequest()
        quaternion = pin.Quaternion(np.eye(3))
        model_state = ModelState()
        model_state.model_name = self.robot_name
        new_base_pos =    + np.array([4,0,-5]) #use this to spawn the robot out of the wall, there is a bug, so it is a relative offset not an absolute one
        print(colored(f"---------Resetting Robot to {p0_adj}, wait for convergence!", "red"))
        model_state.pose.position.x = new_base_pos[0]
        model_state.pose.position.y = new_base_pos[1]
        model_state.pose.position.z = new_base_pos[2]
        model_state.reference_frame = "world"
        model_state.pose.orientation.x = quaternion.x
        model_state.pose.orientation.y = quaternion.y
        model_state.pose.orientation.z = quaternion.z
        model_state.pose.orientation.w = quaternion.w
        model_state.twist.linear.x = 0.
        model_state.twist.linear.y = 0.
        model_state.twist.linear.z = 0.
        model_state.twist.angular.x = 0.
        model_state.twist.angular.y = 0.
        model_state.twist.angular.z = 0.
        reset_base_req.model_state = model_state
        #send request and get response (in this case none)
        self.reset_base(reset_base_req)

        # # create the message (THIS DOES NOT WORK WITH KINEMATIC LOOPS)
        # from gazebo_msgs.srv import SetModelConfiguration
        # from gazebo_msgs.srv import SetModelConfigurationRequest
        # self.reset_joints = ros.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        # req_reset_joints = SetModelConfigurationRequest()
        # req_reset_joints.model_name = self.robot_name
        # req_reset_joints.urdf_param_name = 'robot_description'
        # req_reset_joints.joint_names = self.joint_names
        # req_reset_joints.joint_positions = self.q_des.tolist()
        # # send request and get response (in this case none)
        # resp = self.reset_joints(req_reset_joints)



        kp = np.array([500, 500, #mountain_wire_passive_joints
                       1000, #wire_base_prismatic_r
                       100, 100, 100, #wire_base_poassive_joints
                        500, 500, #mountain_wire_passive_joints
                       1000, #wire_base_prismatic_l
                       300, 300, 300,#mountain_wire_passive_joints
                        150, 130, 120])
        kd=np.array([300, 300,
                     400,
                     80, 80, 80,
                      300, 300,
                     400,
                     80, 80, 80,
                        10, 10, 10])
        self.pid.setPDjoints(kp, kd, np.zeros(self.robot.na))
        #not ok sets only rope PDs
        #self.pid.setPDjoint(p.rope_index, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'], 0.)

        self.start_reset = self.time
        while self.time < (self.start_reset + 6.):

            if np.linalg.norm(self.contactForceW) > 100.:
                self.startupProcedure()
                break

            self.updateKinematicsDynamics()
            self.ros_pub.add_arrow(self.anchor_pos, (self.hoist_l_pos - self.anchor_pos), "green", scale=2.5)  # arope, already in gazebo
            self.ros_pub.add_arrow(self.anchor_pos2, (self.hoist_r_pos - self.anchor_pos2), "green", scale=2.5)  # arope, already in gazebo
            # plot contact force on retractable leg
            self.ros_pub.add_arrow(self.x_ee, self.contactForceW / p.force_scale, "blue", scale=2.5)
            # plot target position (whenever is available)
            self.ros_pub.add_marker(p.x_ee, radius=0.05)
            self.ros_pub.add_mesh(mesh_path=os.environ['LOCOSIM_DIR'] + '/robot_descriptions/climbingrobot_description/meshes/runtime_mesh.obj', position=p.mat2Gazebo, color=None, alpha=1.0)
            self.ros_pub.publishVisual(delete_markers=False)
            # send commands to gazebo
            self.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
            self.time = np.round(self.time + np.array([conf.robot_params[self.robot_name]['dt']]), 4)  # to avoid issues of dt 0.0009999
            self.logData()
            self.rate.sleep()

    def resetRope(self):
        print(colored(f"RESTORING ROPE PD", "red"))
        # enable PD for rope and reset the PD reference to the new estension
        # sample the new elongation
        self.q_des[p.rope_index[0]] = np.copy(p.q[p.rope_index[0]])
        self.q_des[p.rope_index[1]] = np.copy(p.q[p.rope_index[1]])
        #print("resetting rope joints qdes : ", self.q_des[p.rope_index])
        self.Fr_r = 0.
        self.Fr_l = 0.
        self.tau_ffwd[p.rope_index] = np.zeros(2)
        self.pid.setPDjoint(p.rope_index, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'], 0.)

    def printParams(self, p0, pf):
        print(colored(f"p0: {p0}","red"))
        print(colored(f"pf: {pf}","red"))
        print(colored(f"Fleg_max: {self.Fleg_max}","red"))
        print(colored(f"Fr_max: {self.Fr_max}", "red"))
        print(colored(f"mu: {self.mu}", "red"))
        print(colored(f"jump_clearance: {self.optim_params['jump_clearance']}", "red"))
        print(colored(f"mass: {self.optim_params['m']}", "red"))
        print(colored(f"num_params: {self.optim_params['num_params'] }", "red"))
        print(colored(f"int_method: {self.optim_params['int_method']}", "red"))
        print(colored(f"N_dyn: {self.optim_params['N_dyn']}", "red"))
        print(colored(f"FRICTION_CONE: {self.optim_params['FRICTION_CONE']}", "red"))
        print(colored(f"int_steps: {self.optim_params['int_steps']}", "red"))
        print(colored(f"contact_normal: {self.optim_params['contact_normal']}", "red"))
        print(colored(f"b: {self.optim_params['b']}", "red"))
        print(colored(f"p_a1: {self.optim_params['p_a1']}", "red"))
        print(colored(f"p_a2: {self.optim_params['p_a2']}", "red"))
        print(colored(f"g: {self.optim_params['g'] }", "red"))
        print(colored(f"w1: {self.optim_params['w1']}", "red"))
        print(colored(f"w2: {self.optim_params['w2']}", "red"))
        print(colored(f"w3: {self.optim_params['w3']}", "red"))
        print(colored(f"w4: {self.optim_params['w4']}", "red"))
        print(colored(f"w5: {self.optim_params['w5']}", "red"))
        print(colored(f"w6: {self.optim_params['w6']}", "red"))
        print(colored(f"T_th: {self.optim_params['T_th']}", "red"))

    def initOptim(self, p0, pf, Fleg_max = None, Fr_max = None, Fr_min = None):
        if Fleg_max is None:
            self.Fleg_max = 150.
        if Fr_max is None:
            self.Fr_max = 190.  # had to increas because of slopes  it used tp be 90
        if Fr_min is None:
            self.Fr_min = 15.  # had to increas because of downward jumps it used tp be 0
        # down ward jumps
        #self.Fr_min = 15.  # had to increas because of slopes downward jumps it used tp be 0
        self.mu = 0.8
        self.optim_params = {}

        if self.OBSTACLE_AVOIDANCE=="mesh":
            self.optim_params['m'] = self.getRobotMass()
            self.optim_params['num_params'] = 4.
            self.optim_params['int_method'] = 'rk4'
            self.optim_params['N_dyn'] = 30.
            self.optim_params['FRICTION_CONE'] = 1.
            self.optim_params['int_steps'] = 5.
            self.optim_params['b'] = self.anchor_distance_y
            self.optim_params['p_a1'] = matlab.double([0., 0., 0.]).reshape(3, 1)
            self.optim_params['p_a2'] = matlab.double([0., self.optim_params['b'], 0.]).reshape(3, 1)
            self.optim_params['g'] = 9.81
            self.optim_params['w1'] = 1.  # smooth
            self.optim_params['w2'] = 1.  # hoist work 100.  # hoist work use this for multiple jumps for energetic comparison (test are for 0 or 100)
            self.optim_params['w3'] = 0.
            self.optim_params['w4'] = 0.
            self.optim_params['w5'] = 0.
            self.optim_params['w6'] = 0.
            self.optim_params['T_th'] = 0.05
            self.optim_params['obstacle_avoidance'] = 'mesh'
            self.optim_params['jump_clearance'] = 1.5
            # Interpolator (note: z must be increasing — here from -10 to 0)
            #correct initial and final positions
            # in theory you should not correct initial position which is already ok
            #p0[0] = self.terrainManager.wall_surface_eval(p0[2], p0[1],  self.mesh_x,  self.mesh_y,  self.mesh_z)
            pf[0] =  self.terrainManager.wall_surface_eval(pf[2], pf[1],  self.mesh_x,  self.mesh_y,  self.mesh_z) + 0.2 # shift the point to account for leg length!
            #does not work non matching with test_mex TODO
            # p0= np.array([0.99103, 2.5, -6.])
            # pf= np.array([0.40632, 4., -4.])

            # compute consistent normal
            normal = self.terrainManager.wall_normal_eval(p0[2], p0[1], self.mesh_x, self.mesh_y, self.mesh_z)
            self.optim_params['mesh_x'] = self.mesh_x
            self.optim_params['mesh_y'] = self.mesh_y
            self.optim_params['mesh_z'] = self.mesh_z
            self.optim_params['contact_normal'] = matlab.double(normal)
        else:
            self.optim_params['jump_clearance'] = 1.
            self.optim_params['m'] = self.getRobotMass()
            #if terrain is inclined we consider only the Y,Z component of the pf and we need to compute a target point consistent with the wall!
            if conf.robot_params[p.robot_name]['wall_inclination']>0.: #TODO missing normal in matlab wall_constraint!
                pf[0] = (-pf[2]) * math.tan(conf.robot_params[p.robot_name]['wall_inclination']) +  conf.robot_params[p.robot_name]['spawn_x'] #spawn_x is for the anchor point which is shifted wrt the wall
                print(f"adjusting landing target to be consistent with wall: {pf}")
            #no longer used
            self.optim_params['obstacle_avoidance'] = False
            self.optim_params['obstacle_location'] = matlab.double(np.zeros(3)).reshape(3, 1)
            self.optim_params['obstacle_size'] = matlab.double(np.zeros(3)).reshape(3, 1)
            self.optim_params['num_params'] = 4.
            self.optim_params['int_method'] = 'rk4'
            self.optim_params['N_dyn'] = 30.
            self.optim_params['FRICTION_CONE'] = 1.
            self.optim_params['int_steps'] = 5.
            self.optim_params['contact_normal'] = matlab.double([1,0,0]).reshape(3, 1)
            self.optim_params['b'] = self.anchor_distance_y
            self.optim_params['p_a1'] = matlab.double([0., 0., 0.]).reshape(3, 1)
            self.optim_params['p_a2'] = matlab.double([0., self.optim_params['b'], 0.]).reshape(3, 1)
            self.optim_params['g'] = 9.81
            self.optim_params['w1'] = 1. # smooth
            self.optim_params['w2'] = 0. # hoist work 100.  # hoist work use this for multiple jumps for energetic comparison (test are for 0 or 100)
            self.optim_params['w3'] = 0.
            self.optim_params['w4'] = 0.
            self.optim_params['w5'] = 0.
            self.optim_params['w6'] = 0.
            self.optim_params['T_th'] = 0.05
        #matlab.double() doesn’t accept arbitrary non-contiguous numpy arrays. Even if the shape is right, the memory layout matters,
        #so we force them to be contiguous
        p0 = np.ascontiguousarray(p0, dtype=float)  # handles lists, arrays, views
        pf = np.ascontiguousarray(pf, dtype=float)  # handles lists, arrays, views
        try:
            self.matvars = self.eng.optimize_cpp_mex(matlab.double(p0), matlab.double(pf), self.Fleg_max, self.Fr_max,  self.Fr_min, self.mu, self.optim_params)
        except:
            print(colored("there are issues in calling optimize_cpp_mex, maybe regenerate matlab code","red"))
            sys.exit()
        # extract variables
        self.ref_com  = mat_matrix2python(self.matvars['p'])
        self.ref_psi = mat_vector2python(self.matvars['psi'])
        self.ref_l_1 = mat_vector2python(self.matvars['l1'])
        self.ref_l_2 = mat_vector2python(self.matvars['l2'])
        self.ref_time = mat_vector2python(self.matvars['time'])
        self.Fr_l0 = mat_vector2python(self.matvars['Fr_l'])
        self.Fr_r0 = mat_vector2python(self.matvars['Fr_r'])
        self.Fleg = mat_vector2python(self.matvars['Fleg'])
        #this is computed integrating the dynamics with dt and can be different from the reference, we should use the reference at the end of the horizon
        #self.targetPos = mat_vector2python(self.matvars['achieved_target'])
        self.targetPos = self.ref_com[:,-1] #output of optumization
        self.targetPosIdeal = pf
        print(colored(f"offline optimization accomplished, p0:{p0}, pf(ideal):{self.targetPosIdeal}, real tg(r.int.):{self.targetPos}", "blue"))
        #print(colored(f"target to be compared with text_mex_x.py (fine integr. ) is:{self.matvars['achieved_target']}", "blue"))
        self.jump_data = {"time": self.ref_time, "thrustDuration" : self.matvars['T_th'], "p0": p0,
                    "targetPos": self.targetPos,  "Fleg":self.Fleg,
                    "Fr_r": self.Fr_r0, "Fr_l": self.Fr_l0,  "Tf": self.matvars['Tf'] }

        # MPC vars (need to perform before normal optim to know Tf)
        self.mpc_N = int(0.4 * self.optim_params['N_dyn'])
        self.Fr_max_mpc = 250.

        self.optim_params_mpc = {}
        self.optim_params_mpc['int_method'] = 'rk4'
        self.optim_params_mpc['int_steps'] = 5.
        self.optim_params_mpc['contact_normal'] = matlab.double([1., 0., 0.]).reshape(3, 1)
        self.optim_params_mpc['b'] = self.anchor_distance_y
        self.optim_params_mpc['p_a1'] = matlab.double([0., 0., 0.]).reshape(3, 1)
        self.optim_params_mpc['p_a2'] = matlab.double([0., self.optim_params_mpc['b'], 0.]).reshape(3, 1)
        self.optim_params_mpc['g'] = 9.81
        self.optim_params_mpc['m'] = self.getRobotMass()
        self.optim_params_mpc['w1'] = 1.
        self.optim_params_mpc['w2'] = 0.000001
        self.optim_params_mpc['mpc_dt'] = matlab.double(self.matvars['Tf'] / (self.optim_params['N_dyn'] - 1))
        self.deltaFr_l =np.zeros((int(self.mpc_N)))
        self.deltaFr_r = np.zeros((int(self.mpc_N)))
        self.propeller_force = np.zeros((int(self.mpc_N)))

        if self.PLOT_MPC:
            self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)
        status_map = {
            1: "converged",
            -2: "not converged",
            2: "semidef.converg",
            0: "max number of function evaluations"
        }
        status = status_map.get(self.matvars['problem_solved'], "unknown status")
        print(f"problem converged?: {status}")

        if self.matvars['problem_solved'] != 1:
            # not convergence
            if self.matvars['problem_solved'] == -2:
                return False
            # but there can be cases (eg  semidef conv , fmax eval) for which there are also not violations
            violations = self.eval_constraints(self.matvars['c'], self.matvars['num_constr'], self.matvars['constr_tolerance'], verbose=False)
            # if there is at least one violation
            if violations:
                return False
            else:
                return True
        else:
            return True

    def eval_constraints(self, c, num_constr, constr_tolerance, debug=False, verbose=True):
        """
        Check constraint vector `c` against blocks described by `num_constr`.
        - c: sequence or 1D numpy array of constraint values
        - num_constr: object or dict with integer fields:
            wall_constraints,
            retraction_force_constraints,
            force_constraints,
            initial_final_constraints,
            via_point
        - constr_tolerance: scalar
        - debug: bool (print detailed per-constraint info)
        """
        # ensure numpy array (1D)
        c = np.array(c).flatten()
        # compute 0-based start indices for each block (Python indexing)
        w0 = 0
        r0 = w0 + int(num_constr['wall_constraints'])
        f0 = r0 + int(num_constr['retraction_force_constraints'])
        final0 = f0 + int(num_constr['force_constraints'])
        via0 = final0 + int(num_constr['initial_final_constraints'])
        violations = []
        # 1) wall constraints
        w_block = c[w0: w0 + int(num_constr['wall_constraints'])]
        if w_block.size > 0 and np.any(w_block > constr_tolerance):
            violations.append(f"1: {w_block > constr_tolerance}")
        # 2) retraction force constraints
        r_block = c[r0: r0 + int(num_constr['retraction_force_constraints'])]
        if r_block.size > 0 and np.any(r_block > constr_tolerance):
            violations.append(f"2: {r_block > constr_tolerance}")
        # 3) force constraints (unilateral, actuation, friction, ...)
        f_block = c[f0: f0 + int(num_constr['force_constraints'])]
        if f_block.size > 0 and np.any(f_block > constr_tolerance):
            violations.append(f"3: {f_block > constr_tolerance}")
        # 4) final point constraints (several subchecks)
        final_block = c[final0: final0 + int(num_constr['initial_final_constraints'])]
        if final_block.size > 0 and np.any(final_block > constr_tolerance):
            violations.append(f"4: {final_block > constr_tolerance}")
        # 5) via point constraints
        via_block = c[via0: via0 + int(num_constr['via_point'])]
        if via_block.size > 0 and np.any(via_block > constr_tolerance):
            violations.append(f"4: {final_block > constr_tolerance}")
        if verbose and not violations:
            print(colored(f"None", "red"))
        return violations

    def computeMPC(self, delta_t):
        # after the thrust we start MPC,  it will start from time 0.05 so the index will start from  2
        if self.getIndex(delta_t) != -1:
            self.mpc_index = self.getIndex(delta_t)
        else:  # whenever the MPC should not be updated anymore use delta_t to imncrement mpc_index_ffwd
            #print("delta_t MOD dtMpc", (delta_t % self.optim_params_mpc['mpc_dt']))
            if (delta_t % self.optim_params_mpc['mpc_dt']) < 0.001:  # increment mpc_index_ffwd every mpc_dt
                self.mpc_index_ffwd += 1
                if self.mpc_index_ffwd > (self.mpc_N-1): # reference is finished keep the last computed one
                    self.mpc_index_ffwd = self.mpc_N-1
                #debug
                #print("stop mpc, applying ffwd, mpc_index_ffwd: ", self.mpc_index_ffwd)
        # This is better for const dist cause it keeps optimizing till the end!!!
        if (self.mpc_index != self.mpc_index_old): # do optim only every dtMPC  not every dt
            # reduce MPC horizon gradually at the end
            if ((self.mpc_index + self.mpc_N) >=len(self.ref_time)):
                self.mpc_N -=1
            # eval ref
            ref_com = matlab.double(self.ref_com[:, self.mpc_index:self.mpc_index + self.mpc_N].tolist())
            Fr_l0 = matlab.double(self.Fr_l0[self.mpc_index:self.mpc_index + self.mpc_N].tolist())
            Fr_r0 = matlab.double(self.Fr_r0[self.mpc_index:self.mpc_index + self.mpc_N].tolist())
            actual_t = matlab.double(self.ref_time[self.mpc_index])
            actual_state = matlab.double([ self.psi, self.l_1, self.l_2, self.psid, self.l_1d, self.l_2d]).reshape(6,1)
            self.pause_physics_client()
            #perform optimization
            if p.PROPELLERS:
                x = mat_vector2python(self.eng.optimize_cpp_mpc_propellers_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, self.Fr_max_mpc, self.mpc_N, self.optim_params_mpc))
                # extract optim vars
                self.deltaFr_l = x[:self.mpc_N]
                self.deltaFr_r = x[self.mpc_N:2*self.mpc_N]
                self.propeller_force = x[2*self.mpc_N:3*self.mpc_N]
            else:
                x = mat_vector2python(self.eng.optimize_cpp_mpc_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0,self.Fr_max_mpc, self.mpc_N,self.optim_params_mpc))
                 # extract optim vars
                self.deltaFr_l = x[:self.mpc_N]
                self.deltaFr_r = x[self.mpc_N:]

            # store tracking error for RMSE computation
            tracking_error = self.ref_com[:, self.mpc_index] - (self.base_pos - p.anchor_pos)
            self.MPC_tracking_error.append(np.linalg.norm(tracking_error))
            #online plot MPC
            if self.PLOT_MPC:
                self.onlinePlotMPC(self.deltaFr_l, self.deltaFr_r)
            self.unpause_physics_client()

        self.mpc_index_old = self.mpc_index

        return self.deltaFr_l[self.mpc_index_ffwd],self.deltaFr_r[self.mpc_index_ffwd], self.propeller_force[self.mpc_index_ffwd]


    def onlinePlotMPC(self,deltaFr_l, deltaFr_r):
        # debug
        self.ax1.clear()
        self.ax2.clear()
        self.ax1.set_label("delta Frl")
        self.ax2.set_label("delta Frr")
        self.ax1.grid()
        self.ax2.grid()
        #MPC action (red)
        self.ax1.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N], deltaFr_l, "or-")
        self.ax2.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N], deltaFr_r, "or-")
        # full action (black)
        self.ax1.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
                      self.Fr_l0[self.mpc_index:self.mpc_index + self.mpc_N] + deltaFr_l, "ok-")
        self.ax2.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
                      self.Fr_r0[self.mpc_index:self.mpc_index + self.mpc_N] + deltaFr_r, "ok-")
        # plot Fr limits Does not work
        # self.ax1.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
        #               -self.Fr_max * np.ones((1, self.mpc_N)), "r-")
        # self.ax2.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
        #               -self.Fr_max * np.ones((1, self.mpc_N)), "r-")
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def computeJumpEnergyConsumption(self):
        # get index
        #a_bool = self.jump_data["time"] > self.jump_data["thrustDuration"]
        #lift_off_idx = min([i for (i, val) in enumerate(a_bool) if val]) - 1
        lift_off_idx = np.max(np.where((self.time_log - self.start_logging) <=self.jump_data["thrustDuration"]))
        impulse_work= 0.5 * self.optim_params['m'] *self.base_vel_log[:, lift_off_idx].dot(self.base_vel_log[:, lift_off_idx]) # ekin at liftoff
        # this integral is done on a rough discretization dt
        touch_down_idx = np.max(np.where( (self.time_log - self.start_logging)  < self.jump_data["Tf"]))
        hoist_work = 0.
        for i in range(touch_down_idx):
            hoist_work = hoist_work + (
                        abs(self.Fr_r_log[i] * self.l_2d_log[i]) + abs(self.Fr_l_log[i] * self.l_1d_log[i])) * conf.robot_params[p.robot_name]['dt']
        return impulse_work + hoist_work


    def print_message(self, message = "", decimate = 1000):
        if not hasattr(self, 'print_counter'):
            self.print_counter = 0
        if np.mod(self.print_counter, decimate) == 0:
            print(colored(message, "red"))
        self.print_counter += 1

    def checkCycleTermination(self):
        p.jumpNumber += 1
        #continue jumping
        if (p.jumpNumber < p.numberOfJumps):
            p.stateMachine = 'start_jump'
            # reset vars for multiple jumps
            self.touch_down_detected_prismleg = False
            self.touch_down_detected_l = False
            self.touch_down_detected_r = False
            self.optimal_control_traj_finished = False
            self.mpc_index = 0
            self.mpc_index_old = 0
            self.mpc_index_ffwd = 0  # updated only when we stop recomputing mpc

            #abrupt reset next state here, if needed
            if self.SAMPLE_FOR_VALUE_FUNCTION:
                self.resetRobot(self.initial_position[self.jumpNumber])
            self.startJump = self.time
            self.touch_down_detected_prismleg = False
            return False
        else:
            p.stateMachine = 'idle'
            print(colored("Terminating cycle","red"))
            #return False # stay there forever
            return True # exit loop

    def plotReferenceTraj(self,ref_com, color="white"):
        # plot intermediate positions (for iterates on rows so I need to transpose)
        for blob  in ref_com.T:
            self.ros_pub.add_marker(blob, color=color, radius=0.2, alpha = 0.5)



    def stateMachineLoop(self):

        terminateFlag = False
        if ( p.stateMachine == 'start_jump') and (p.time >= p.startJump):
            # first run optim and fill in jump variable
            p.pause_physics_client()
            # PAPER
            if p.PAPER and p.jumpNumber == 0:
                #first optim
                print(colored(f"Performing optimization for the whole trajectory only for paper figure", "red"))
                for i in range(len(p.desired_target)):
                    print(colored(f"Optimization {i}", "red"))
                    if i==0:
                        p.initOptim(p.base_pos - p.mat2Gazebo, p.desired_target[i])
                        p.total_ref_com = p.ref_com.copy()
                        p.targetPos_paper = [p.ref_com[:, -1]]
                    else:
                        # second optim:second jump we assume it starts from last position
                        p.initOptim(p.targetPos_paper[i-1], p.desired_target[i])
                        p.total_ref_com = np.concatenate((p.total_ref_com, p.ref_com), axis=1)
                        p.targetPos_paper.append(p.ref_com[:, -1])


            #normal usage
            print(colored(f"-------------------- Start trajectory optimization", "blue"))
            p.initOptim(p.base_pos - p.mat2Gazebo, p.desired_target[p.jumpNumber])
            p.unpause_physics_client()


            p.des_leg_orient = p.getImpulseAngle()

            #set the end of orienting
            p.end_orienting = p.startJump + p.orientTime
            p.end_thrusting = p.startJump + p.orientTime + p.jump_data["thrustDuration"]
            p.end_flying = p.startJump + p.orientTime + p.jump_data["Tf"]

            p.stateMachine = 'orienting_leg'  # this phase only waits is not doing anything
            if p.SAVE_BAG:
                p.recorder.start_recording_srv()

        if (p.stateMachine == 'orienting_leg'):
            # use propellers (review)
            if p.USE_PROPELLERS_FOR_LEG_REORIENT:
                p.q_des[p.hip_pitch_joint] = p.des_leg_orient[0]
                p.q_des[p.hip_roll_joint] = 0. #set leg straight
                # reorient base yaw to be p.des_leg_orient[1]
                rpy = p.math_utils.rot2eul(p.w_R_b)
                Mz = 30.*(p.des_leg_orient[0] - rpy[2])
                p.apply_propeller_moment(Mz)
            else:
                p.q_des[p.hip_pitch_joint] = p.des_leg_orient[0]
                p.q_des[p.hip_roll_joint] = p.des_leg_orient[1]

            if  (p.time >= p.end_orienting):
                print(colored(f"Stop orienting leg {p.time}", "blue"))
                print(colored(f"---------Starting jump  number {p.jumpNumber+1} to optimized target: {p.jump_data['targetPos']} from actual p0 : {p.base_pos - p.mat2Gazebo}","red"))
                print(colored(f"Start trusting", "blue"))
                p.tau_ffwd = np.zeros(p.robot.na)
                p.tau_ffwd[p.rope_index] = p.g[p.rope_index]  # compensate gravitu in the virtual joint to go exactly there
                p.pid.setPDjoint(p.anchor_passive_joints, 0., 0., 0.)
                p.pid.setPDjoint(p.base_passive_joints, 0., 0., 0.)
                p.pid.setPDjoint(p.leg_index, 0., 0., 0.)
                print(colored(f"ZERO LEG AND ROPE PD", "red"))
                p.stateMachine = 'thrusting'
                p.pid.setPDjoint(p.rope_index, 0., 0., 0.)
                p.w_Fleg = p.jump_data["Fleg"]

        if (p.stateMachine == 'thrusting'):
            # apply leg inpulse for thust duration
            p.tau_ffwd[p.leg_index] = -p.Jleg.T.dot(p.w_Fleg)
            # plot Fleg
            p.ros_pub.add_arrow(p.x_ee, p.w_Fleg / p.force_scale, "red", scale=2.5)

            # start also applying forces to ropes
            delta_t = p.time - p.end_orienting
            p.Fr_r = p.jump_data["Fr_r"][p.getIndex(delta_t)]
            p.Fr_l = p.jump_data["Fr_l"][p.getIndex(delta_t)]

            # plot rope forces
            p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=1.5)
            p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=1.5)
            p.tau_ffwd[p.rope_index[0]] = p.Fr_r
            p.tau_ffwd[p.rope_index[1]] = p.Fr_l

            if (p.time > p.end_thrusting):
                print(colored(f"Stop Trhusting  {p.time}", "blue"))
                print(colored(f"RESTORING LEG PD", "red"))
                # reenable  the PDs for base passive joints otherwise it keeps rotating like crazy
                p.pid.setPDjoint(p.base_passive_joints, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'] ,  0.)
                # reenable leg pd
                p.pid.setPDjoint(p.leg_index, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'],  0.)
                #reset the torque on the leg (stop applyng inpulse)
                p.tau_ffwd[p.leg_index] = np.zeros(len(p.leg_index))
                p.stateMachine = 'flying'


                if  p.landing:
                    p.stateMachine = 'flying_and_wait_for_touchdown'
                    #put leg straight for landing
                    p.q_des[p.leg_index] = np.array([-1.57, 0.0, 0.25])
                else:
                    # retract leg
                    p.q_des[p.leg_index[2]] = 0.25


                print(colored(f"Start "+ p.stateMachine+f" {p.time}", "blue"))

        if (p.stateMachine == 'flying'):
            # after the thrust we start MPC it will start from time 0.05 so the index should be 12
            # applying forces to ropes
            delta_t = p.time - p.end_orienting
            if p.MPC_control:
                deltaFr_l0, deltaFr_r0, prop_force = p.computeMPC(delta_t)
                if p.PROPELLERS:
                    p.apply_propeller_force(prop_force)

            else:
                deltaFr_l0 = 0.
                deltaFr_r0 = 0.

            p.Fr_l = p.jump_data["Fr_l"][p.getIndex(delta_t)]+ deltaFr_l0
            p.Fr_r = p.jump_data["Fr_r"][p.getIndex(delta_t)]+ deltaFr_r0

            #plot rope forces
            p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=2.5)
            p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=2.5)

            p.tau_ffwd[p.rope_index[0]] = p.Fr_r
            p.tau_ffwd[p.rope_index[1]] = p.Fr_l


            if (p.time >= p.end_flying):
                print(colored(f"Stop Flying  {p.time}", "blue"))
                # reset the qdes
                # we need to reset the rope PD because the Fr are finished and I would get the final value repeated  that is not the good thing to do
                p.resetRope()
                terminateFlag = p.checkCycleTermination()

        # this is the same as flying but with the lander
        if (p.stateMachine == 'flying_and_wait_for_touchdown'):
            # applying forces to ropes, when time is finished just rset rope length (only once!) and wait for tf
            delta_t = p.time - p.end_orienting
            if p.MPC_control:
                deltaFr_l0, deltaFr_r0, p.prop_force = p.computeMPC(delta_t)
                if p.PROPELLERS:
                    # old
                    if not self.USE_ORIENTATION_CONTROL:
                        p.apply_propeller_force(p.prop_force)
                    else:
                        prop_forceW = p.n_bar * p.prop_force
                        # compute thrust for orientation
                        p.prop_thrusts, w_wrench = p.orientControl.computeThrust(des_orient=p.base_rpy,
                                                                 act_orient=p.base_rpy,
                                                                 w_omega_b=p.omega_b,
                                                                 Ko=conf.robot_params[p.robot_name]['Ko'],
                                                                 Do=conf.robot_params[p.robot_name]['Do'],
                                                                 w_additional_force=prop_forceW)
                        p.apply_propeller_orient(w_wrench, p.prop_thrusts)
            else:
                deltaFr_l0 = 0.
                deltaFr_r0 = 0.

            if not p.optimal_control_traj_finished:
                if p.getIndex(delta_t) != -1:
                    p.Fr_l = p.jump_data["Fr_l"][p.getIndex(delta_t)] + deltaFr_l0
                    p.Fr_r = p.jump_data["Fr_r"][p.getIndex(delta_t)] + deltaFr_r0
                else:
                    # start again pid gains and reset qdes
                    # extend a bit the leg
                    p.q_des[p.leg_index] = np.array([-1.57, 0.0, 0.1])
                    p.resetRope()
                    p.optimal_control_traj_finished = True
                # check for early td and in case reset rope I comment otherwise is triggering to early
                if (p.time>=(p.end_thrusting + 0.5*p.jump_data["Tf"]))and p.detectTouchDown():
                    p.resetRope()
                    p.landing_error = p.printLandingInfo()
                    print(colored("Early TD detected, Starting landing", "blue"))
                    p.stateMachine = 'landing'
                    p.start_landing = p.time

            else: # you are checking for delayed TD you have already reset rope and restored PD
                if p.detectTouchDown():
                    p.landing_error = p.printLandingInfo()
                    print(colored("Delayed TD detected, Starting landing", "blue"))
                    p.stateMachine = 'landing'
                    p.start_landing = p.time

            # plot rope forces
            p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=3.5)
            p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=3.5)
            p.tau_ffwd[p.rope_index[0]] = p.Fr_r
            p.tau_ffwd[p.rope_index[1]] = p.Fr_l
            p.end_flying = p.startJump + p.orientTime + p.jump_data["Tf"]

        if (p.stateMachine == 'landing'):
                print(colored(f"Start landing {p.time}", "blue"))
                # this when you do not want to continue the cycle need to comment checkCycleTermination()
                #p.prop_force = (-25.)  # push against the wall
                #p.apply_propeller_force(p.prop_force)
                terminateFlag = p.checkCycleTermination()

        if (p.stateMachine == 'idle'):
            pass

        return terminateFlag

    def printLandingInfo(self):
        print(colored(f"PRINTING LANDING INFO", "red"))
        landing_location = self.base_pos - self.mat2Gazebo
        print(colored(f" real landing (in matlab convention) is: {landing_location}", "green"))
        print(colored(f" while from optim it should be  {self.targetPos}", "green"))

        print(colored(f" the landing error is  {np.linalg.norm(landing_location - self.targetPos)}", "green"))
        # jump_length = np.linalg.norm(p0[:2] - self.targetPos[:2])
        # MSE = np.square(np.array(p.MPC_tracking_error)).mean()
        # RMSE = math.sqrt(MSE)
        # print(colored(
        #     f" the relative landing error (norm per jump lenghth)  is {100 * np.linalg.norm(landing_location - self.targetPos) / jump_length}%",
        #     "blue"))
        print(colored(f" the energy consumption is  {p.computeJumpEnergyConsumption()}", "green"))
        print(colored(f" the leg impulse  is  {self.Fleg}", "green"))
        print(colored(f" the norm of the leg impulse  is  {np.linalg.norm(self.Fleg)}", "green"))

        if self.SAMPLE_FOR_VALUE_FUNCTION:
            jump_length = np.linalg.norm(self.desired_target[self.jumpNumber][1:] - self.ref_com[1:, 0])
            row = {'test':self.jumpNumber,
                    'p0_x':self.ref_com[0,0],
                    'p0_y':self.ref_com[1,0],
                    'p0_z':self.ref_com[2,0],
                    'pf_x': self.desired_target[self.jumpNumber][0],
                    'pf_y': self.desired_target[self.jumpNumber][1],
                    'pf_z': self.desired_target[self.jumpNumber][2],
                    'avg_tracking_cost': np.sum(self.MPC_tracking_error) / jump_length,
                    'tracking_cost':self.MPC_tracking_error}
            print(colored(f"Start: {self.ref_com[:, 0]}, Target: {self.desired_target[self.jumpNumber]}, Track.err: {np.sum(self.MPC_tracking_error)/jump_length}", "yellow"))
            df_row = pd.DataFrame([row])
            p.results = pd.concat([p.results, df_row], ignore_index=True)
            p.results.to_csv(p.result_csv_path, index=False)

        return self.targetPos - landing_location

    def readJsonFile(self, terrain="hemi"):
        json_path = "multiple_jumps_"+terrain+".json"
        with open(json_path, "r") as f:
            data = json.load(f)
        return data

def talker(p):
    p.start()
    additional_args = ['robot_name:='+p.robot_name,
                       'gui:=' + str(p.use_gui),
                       'spawn_2x:=' + str(conf.robot_params[p.robot_name]['spawn_2x']),
                       'spawn_2y:=' + str(conf.robot_params[p.robot_name]['spawn_2y']),
                       'spawn_2z:=' + str(conf.robot_params[p.robot_name]['spawn_2z']),
                       'wall_inclination:=' + str(conf.robot_params[p.robot_name]['wall_inclination']),
                       'double_propeller:=' + str(p.USE_PROPELLERS_FOR_LEG_REORIENT)
                       ]
    world_name = "climbingrobot2.world"
    launch_file = rospkg.RosPack().get_path('ros_impedance_controller') + '/launch/ros_impedance_controller_climbingrobot2.launch'
    p.startSimulator(world_name=world_name, additional_args=additional_args, launch_file=launch_file)

    p.loadModelAndPublishers()

    if p.SAMPLE_FOR_VALUE_FUNCTION:
        p.terrainManager = TerrainManager(grid_size=100, wall_depth=1, max_ridge_depth=0.5, seed="default", Lz=-20, Ly=5, generate_terrain=True, terrain_type='rock')
        p.mesh_x, p.mesh_y, p.mesh_z = p.terrainManager.get_mesh()
        #Read list of feasible jumps
        #get terrain consistent  targets and initial pos
        feas_jumps = pd.read_csv("feasible_jumps2.csv")
        p.desired_target = []
        p.initial_position = []
        p.test_indexes = []
        for _, row in feas_jumps.iterrows():
            x0 = p.terrainManager.wall_surface_eval(row["z0"], row["y0"], p.mesh_x, p.mesh_y, p.mesh_z) + 0.2  # shift the point to account for leg length!
            xf = p.terrainManager.wall_surface_eval(row["zf"], row["yf"], p.mesh_x, p.mesh_y, p.mesh_z) + 0.2  # shift the point to account for leg length!
            p.initial_position.append(np.array([x0, row["y0"], row["z0"]]))
            p.desired_target.append(np.array([xf, row["yf"], row["zf"]]))
            p.test_indexes.append(np.array([row["i0"], row["if"]]))
        p.numberOfJumps = len(p.desired_target)

        # prepare store of tracking results for feasible jumps
        p.result_csv_path = "value_function_dataset.csv"
        columns = ['test', 'i0', 'if', 'p0_x', 'p0_y', 'p0_z', 'pf_x', 'pf_y', 'pf_z', 'avg_tracking_cost', 'tracking_cost']
        p.results = pd.DataFrame(columns=columns)

        # Resume from existing CSV if exists
        if os.path.exists(p.result_csv_path) and os.path.getsize(p.result_csv_path) > 0:
            try:
                p.results = pd.read_csv(p.result_csv_path)
                print(colored(f"CSV {p.result_csv_path} exists... continuing from where you left", "blue"))
                if len(p.results) > 0:
                    #get the last test done (iloc allows to get the roe number)
                    p.jumpNumber = int(p.results.iloc[-1]["test"]) + 1
                else:
                    p.jumpNumber = 0
            except Exception as e:
                print(colored(f"Could not read CSV, starting fresh. Error: {e}", "red"))
                p.results = pd.DataFrame(columns=columns)
                p.jumpNumber = 0
        else:
            print(colored(f"CREATING NEW CSV TO STORE TESTS: {p.result_csv_path}", "blue"))
            p.results = pd.DataFrame(columns=columns)
            p.jumpNumber = 0
    else:
        #############hard coded
        # jump params
        # jump starting position
        # p0 = np.array([0.28, 2.5, -6.10104])  # there is singularity for px = 0!
        # p.numberOfJumps = 3
        # # jump landing position
        # p.desired_target = [np.array([0.28, 4, -3.7]),
        #                     np.array([0.28, 2.5, -6]),
        #                     np.array([0.28, 3.6, -11])]
        # p.terrainManager = TerrainManager( grid_size=100, wall_depth=1, max_ridge_depth=0.5, seed="default",Lz=-20, Ly=5, generate_terrain = True, terrain_type = 'rock')
        # ######################

        #############Json
        data = p.readJsonFile(terrain="rock") # hemi, gaussian
        p0 = np.array(data["target_points"][0])
        p.desired_target = [np.array(t) for t in data["target_points"][1:]]
        p.numberOfJumps = len(p.desired_target)
        p.terrainManager = TerrainManager(grid_size=data["terrain_info"]["grid_size"],wall_depth=data["terrain_info"]["wall_depth"], max_ridge_depth=data["terrain_info"]["max_ridge_depth"],
                                          seed=data["terrain_info"]["seed"], Lz=data["terrain_info"]["Lz"],Ly=data["terrain_info"]["Ly"], generate_terrain=True, terrain_type=data["terrain_info"]["terrain_type"] )
        p.mesh_x, p.mesh_y, p.mesh_z = p.terrainManager.get_mesh()
        p.jumpNumber = 0
        ##############


    p.startupProcedure()
    p.initVars()
    p.q_des = np.copy(p.q_des_q0)


    #loop frequency
    p.rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])
    p.updateKinematicsDynamics()

    # spawn mesh in gazebo (needs mat2Gazebo)
    if p.OBSTACLE_AVOIDANCE=='mesh':
        texture_path = rospkg.RosPack().get_path('climbingrobot_description') + '/media/materials/textures/rocks.jpg'
        spawnMesh(p.mesh_x, p.mesh_y, p.mesh_z, position=p.mat2Gazebo,store_location_mesh=os.environ['LOCOSIM_DIR']+'/robot_descriptions/climbingrobot_description/meshes/', texture_path=texture_path)


    p.orientTime = 1.0
    p.stateMachine = 'start_jump'


    # set the rope base joint variables to initialize in p0 position, the leg ones are defined in params.yaml
    if p.SAMPLE_FOR_VALUE_FUNCTION:
        p.startJump = 0.
        p.resetRobot(p.initial_position[p.jumpNumber])
    else:
        p.startJump = np.linalg.norm(p0) / 2
        p0_adj = p0.copy()
        p0_adj[0] = p.terrainManager.wall_surface_eval(p0[2], p0[1], p.mesh_x, p.mesh_y, p.mesh_z) + 0.2 # account for leg
        p.q_des[:12] = p.computeJointVariables(p0_adj)

    p.start_logging = p.startJump
    p.setSimSpeed(dt_sim=0.001, max_update_rate=300, iters=1500)


    while not ros.is_shutdown():
        # update the kinematics
        p.updateKinematicsDynamics()
        # jump state machine
        stop = p.stateMachineLoop()
        p.ros_pub.add_arrow(p.anchor_pos, (p.hoist_l_pos - p.anchor_pos), "green", scale=2.5)  # arope, already in gazebo
        p.ros_pub.add_arrow(p.anchor_pos2, (p.hoist_r_pos-p.anchor_pos2), "green", scale=2.5)  # arope, already in gazebo

        # plot contact force on retractable leg
        p.ros_pub.add_arrow(p.x_ee, p.contactForceW / p.force_scale, "blue", scale=2.5)

        #plot target position (whenever is available)
        p.ros_pub.add_marker(p.x_ee, radius=0.05)
        p.ros_pub.add_mesh(mesh_path=os.environ['LOCOSIM_DIR']+'/robot_descriptions/climbingrobot_description/meshes/runtime_mesh.obj', position=p.mat2Gazebo, color=None, alpha=1.0)
        if not p.PAPER and hasattr(p, "ref_com"):
            p.plotReferenceTraj(p.mat2Gazebo.reshape(3, 1) + p.ref_com, color="red")
            p.ros_pub.add_marker(p.mat2Gazebo + p.jump_data["targetPos"], color="red", radius=0.3, alpha=1.)
            p.ros_pub.add_marker(p.mat2Gazebo + p.targetPosIdeal, color="green", radius=0.5, alpha=0.5)
        if p.PAPER and hasattr(p, "total_ref_com"):
            p.plotReferenceTraj(p.mat2Gazebo.reshape(3, 1) + p.total_ref_com, color="white")
            for i in range(len(p.targetPos_paper)):
                p.ros_pub.add_marker(p.mat2Gazebo + p.targetPos_paper[i], color="green", radius=0.5, alpha=0.5)
        p.ros_pub.publishVisual(delete_markers=False)

        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),4)  # to avoid issues of dt 0.0009999
        if (p.time > p.start_logging):
            p.logData()
        # wait for synchronization of the control loop
        p.rate.sleep()

        if stop:
            break



def plot3D(name, figure_id, label, time_log, var, time_mat = None, var_mat = None):
    fig = plt.figure()
    fig.suptitle(name, fontsize=20)
    plt.subplot(3,1,1)
    plt.ylabel(label[0])
    plt.plot(time_log, var[0, :], linestyle='-', marker="o", markersize=0,  lw=5, color='blue')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[0, :], linestyle='-', marker="o", markersize=0,  lw=5, color='red')
    plt.grid(True)
    plt.legend(['act', 'ref'])

    plt.subplot(3,1,2)
    plt.ylabel(label[1])
    plt.plot(time_log, var[1, :], linestyle='-', marker="o", markersize=0,  lw=5, color='blue')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[1, :], linestyle='-', marker="o", markersize=0,  lw=5, color='red')
    plt.grid()
    plt.legend(['act', 'ref'])

    plt.subplot(3,1,3)
    plt.ylabel(label[2])
    plt.plot(time_log, var[2, :], linestyle='-', marker="o", markersize=0,  lw=5, color='blue')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[2, :], linestyle='-', marker="o", markersize=0,  lw=5, color='red')
    plt.grid()
    plt.legend(['act', 'ref'])

if __name__ == '__main__':
    p = ClimbingrobotController(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        pass

    # STOP RECORDER FIRST (if you do ctrl +c you give sigint to all the nodes) here you are not, hence you need to stop the recorder manually before
    if p.SAVE_BAG:
        p.recorder.stop_recording_srv()
    ros.signal_shutdown("killed")
    p.deregister_node()
    if p.landing: # for the landing test you should press Ctrl C to stop everything
        p.plotStuff()




        

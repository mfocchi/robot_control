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
from base_controllers.utils.common_functions import plotJoint, plotFrameLinear
from termcolor import colored
import os
import tf
from base_controllers.base_controller_fixed import BaseControllerFixed
from geometry_msgs.msg import Wrench, Point
from gazebo_msgs.msg import ContactsState
import scipy.io.matlab as mio
import rospkg
from scipy.linalg import block_diag
from base_controllers.utils.matlab_conversions import mat_vector2python, mat_matrix2python
import matlab.engine
import numpy.matlib
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
import pandas as pd
import sys
sys.path.insert(0,'./codegen')

import  base_controllers.params as conf
robotName = "climbingrobot2"

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

        self.OBSTACLE_AVOIDANCE = False
        self.obstacle_location = np.array([0, 2.5, -6])
        self.obstacle_size = np.array([1.5, 1.5, 0.866])

        self.force_scale = 60.
        self.mountain_thickness = 0.1 # TODO call the launch file passing this parameter
        self.r_leg = 0.3
        self.real_robot = conf.robot_params[robot_name]['real_robot']
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

    def loadModelAndPublishers(self, xacro_path=None):
        xacro_path = rospkg.RosPack().get_path('climbingrobot_description') + '/urdf/' + p.robot_name + '.xacro'
        additional_urdf_args = ' anchorX:=' + str(conf.robot_params[self.robot_name]['spawn_x'])
        additional_urdf_args += ' anchorY:=' + str(conf.robot_params[self.robot_name]['spawn_y'])
        additional_urdf_args += ' anchorZ:=' + str(conf.robot_params[self.robot_name]['spawn_z'])
        additional_urdf_args += ' anchor2X:=' + str(conf.robot_params[self.robot_name]['spawn_2x'])
        additional_urdf_args += ' anchor2Y:=' + str(conf.robot_params[self.robot_name]['spawn_2y'])
        additional_urdf_args += ' anchor2Z:=' + str(conf.robot_params[self.robot_name]['spawn_2z'])
        super().loadModelAndPublishers(xacro_path=xacro_path, additional_urdf_args=additional_urdf_args,  markers_time_to_live=conf.robot_params[p.robot_name]['dt'])

        self.broadcaster = tf.TransformBroadcaster()
        self.sub_contact= ros.Subscriber("/" + self.robot_name + "/foot_bumper", ContactsState,
                                             callback=self._receive_contact, queue_size=1, buff_size=2 ** 24,
                                             tcp_nodelay=True)
        # this is for the matlab optim
        self.eng = matlab.engine.start_matlab()
        self.eng.addpath('./codegen', nargout=0)
        if self.PROPELLERS:
            self.pub_prop_force = ros.Publisher("/base_force", Wrench, queue_size=1, tcp_nodelay=True)
        if self.SAVE_BAG:
            self.recorder = RosbagControlledRecorder(record_from_startup_=False)

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
        self.omega_b =  self.Jb[3:, :].dot(self.qd)

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

        self.rope_direction = (p.hoist_l_pos - p.anchor_pos) / np.linalg.norm(p.hoist_l_pos  - p.anchor_pos)
        self.rope_direction2 = (p.hoist_r_pos - p.anchor_pos2) / np.linalg.norm(p.hoist_r_pos - p.anchor_pos2)

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
        self.l_1 = self.q[p.rope_index[1]] - hoist_distance/2 + self.anchor_distance_y/2
        self.l_2 = self.q[p.rope_index[0]] - hoist_distance/2 + self.anchor_distance_y/2

        # use geometric intuition for psid
        n_par = (self.anchor_pos - self.anchor_pos2) / np.linalg.norm(self.anchor_pos - self.anchor_pos2)
        rope2_axis = (self.base_pos - self.anchor_pos2) / np.linalg.norm(self.base_pos - self.anchor_pos2)
        self.n_bar = np.cross(n_par, rope2_axis) / np.linalg.norm(np.cross(n_par, rope2_axis))

        #compute matlab state derivatives
        self.psid = (self.n_bar.dot(self.base_vel)) / np.linalg.norm(np.cross(n_par, self.base_pos - self.anchor_pos2))
        self.l_1d = self.qd[p.rope_index[1]]
        self.l_2d = self.qd[p.rope_index[0]]

        # the mountain is always wrt to world
        mountain_pos = np.array([-self.mountain_thickness/2, conf.robot_params[self.robot_name]['spawn_y'], 0.0])
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
        self.touch_down_detected_l = False
        self.touch_down_detected_r = False
        self.optimal_control_traj_finished = False
        self.MPC_tracking_error = []

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

        w_R_wall = self.math_utils.eul2Rot(np.array([0,-conf.robot_params[p.robot_name]['wall_inclination'],0]))
        self.wall_normal = w_R_wall[:,0].copy() #take X axis, I need to use copy otherwise matlab complains is not contiguous

        self.mpc_index = 0
        self.mpc_index_old = 0
        self.mpc_index_ffwd = 0 # updated only when we stop recomputing mpc

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
        time_gazebo = p.time_log - p.start_logging
        plotJoint('position', time_gazebo, p.q_log, p.q_des_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
        plot3D('basePos', 2,  ['X', 'Y', 'Z'], time_gazebo, actual_com, p.ref_time, p.ref_com)
        plot3D('matlab states', 3, ['psi', 'l1', 'l2'], time_gazebo, p.simp_model_state_log, p.ref_time, np.vstack((p.ref_psi, p.ref_l_1, p.ref_l_2)) )

        # plot rope forces
        # plt.figure()
        # plt.subplot(2, 1, 1)
        # plt.ylabel("Fr_l")
        # plt.plot(p.ref_time, p.Fr_l0, color='red')
        # plt.plot(time_gazebo, p.Fr_l_log, color='blue')
        # plt.grid()
        # plt.subplot(2, 1, 2)
        # plt.ylabel("Fr_r")
        # plt.plot(p.ref_time, p.Fr_r0, color='red')
        # plt.plot(time_gazebo, p.Fr_r_log, color='blue')
        # plt.grid()

        #save data
        filename = f'test_gazebo_MPC_{p.MPC_control}.mat'
        mio.savemat(filename, {'ref_time': p.ref_time, 'ref_com': p.ref_com,
                                'time_gazebo': time_gazebo, 'actual_com': actual_com,
                                'ref_psi':p.ref_psi,'ref_l_1':p.ref_l_1, 'ref_l_2':p.ref_l_2,
                                'psi': p.simp_model_state_log[0,:], 'l_1': p.simp_model_state_log[1,:], 'l_2': p.simp_model_state_log[2,:],
                                'psid': p.psid_log, 'l_1d': p.l_1d_log,'l_2d': p.l_2d_log,
                                'mu': p.mu , 'Fleg': p.Fleg,'Fr_max': p.Fr_max,
                                'Fr_l0': p.Fr_l0, 'Fr_r0': p.Fr_r0,
                                'Fr_l': p.Fr_l_log, 'Fr_r': p.Fr_r_log })

    def getIndex(self,t):
        try:
            # get index
            a_bool = self.jumps[self.jumpNumber]["time"] >= t
            idx = min([i for (i, val) in enumerate(a_bool) if val])-1
            if idx == -1:
                return 0
            else:
                return idx
        except:
            return  -1

    def getImpulseAngle(self):
        angle_hip_roll =  math.atan2(self.jumps[self.jumpNumber]["Fleg"][1],
                                self.jumps[self.jumpNumber]["Fleg"][0])
        angle_hip_pitch =  math.atan2(self.jumps[self.jumpNumber]["Fleg"][2], self.jumps[self.jumpNumber]["Fleg"][0])
        print(colored(f"Start orienting leg to (pitch, roll)  : {angle_hip_roll, angle_hip_roll}", "blue"))
        angle_hip_pitch +=-1.57
        return angle_hip_pitch, angle_hip_roll

    # compute the passive and rope joints reference from the matlab position referred to a world frame located in between anchors
    def computeJointVariables(self, p):
        # mountain_wire_pitch_l = math.atan2(p[0]-conf.robot_params[self.robot_name]['spawn_x'], -p[2])
        # mountain_wire_pitch_r = math.atan2(p[0]-conf.robot_params[self.robot_name]['spawn_2x'], -p[2])
        if conf.robot_params[self.robot_name]['wall_inclination']>0.: #TODO missing normal in matlab wall_constraint!
            p[0] = (-p[2]) * math.tan(conf.robot_params[self.robot_name]['wall_inclination'])  #spawn_x is for the anchor point which is shifted wrt the wall
            print(f"adjusting initial position to be consistent with wall: {p}")

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
        force_th = 10.
        if not self.touch_down_detected_l and (self.wall_normal.dot(self.contactForceW_l) > force_th):
            self.touch_down_detected_l = True
        if not self.touch_down_detected_r  and (self.wall_normal.dot(self.contactForceW_r) > force_th):
            self.touch_down_detected_r = True

        if self.touch_down_detected_l and self.touch_down_detected_r:
            print(colored("TouchDown Detected", "blue"))
            # sample com pos
            self.x_tilde0 =  self.wall_normal.reshape(1, 3) @ (self.com)# - self.x_p)
            return True
        else:
            return False

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
        print(colored(f"obstacle_avoidance: {self.optim_params['obstacle_avoidance']}", "red"))
        print(colored(f"obstacle_location: {self.optim_params['obstacle_location']}", "red"))
        print(colored(f"obstacle_size: {self.optim_params['obstacle_size']}", "red"))
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

    def initOptim(self, p0, pf):
        ##offline optim vars
        self.Fleg_max = 300.
        self.Fr_max = 90.  # had to increas because of slopes downward jumps it used tp be 90
        self.Fr_min = 0.  # had to increas because of slopes downward jumps it used tp be 0
        # down ward jumps
        #self.Fr_max = 190.  # had to increas because of slopes downward jumps it used tp be 90
        #self.Fr_min = 15.  # had to increas because of slopes downward jumps it used tp be 0
        self.mu = 0.8
        self.optim_params = {}
        self.optim_params['jump_clearance'] = 1.
        self.optim_params['m'] = self.getRobotMass()

        #if terrain is inclined we consider only the Y,Z component of the pf and we need to compute a target point consistent with the wall!
        if conf.robot_params[p.robot_name]['wall_inclination']>0.: #TODO missing normal in matlab wall_constraint!
            pf[0] = (-pf[2]) * math.tan(conf.robot_params[p.robot_name]['wall_inclination']) +  conf.robot_params[p.robot_name]['spawn_x'] #spawn_x is for the anchor point which is shifted wrt the wall
            print(f"adjusting landing target to be consistent with wall: {pf}")

        self.optim_params['obstacle_avoidance'] = self.OBSTACLE_AVOIDANCE
        self.optim_params['obstacle_location'] = matlab.double(self.obstacle_location).reshape(3, 1)
        self.optim_params['obstacle_size'] = matlab.double(self.obstacle_size).reshape(3, 1)
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

        try:
            self.matvars = self.eng.optimize_cpp_mex(matlab.double(p0.tolist()), matlab.double(pf.tolist()), self.Fleg_max, self.Fr_max,  self.Fr_min, self.mu, self.optim_params)
        except:
            print(colored("Regenerate matlab code issues in calling optimize_cpp_mex","red"))
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
        self.targetPosIdeal = self.ref_com[:, -1]

        print(colored(f"offline optimization accomplished, p0:{p0}, target(rough integr):{self.targetPos}", "blue"))
        print(colored(f"target to be compared with text_mex_x.py (fine integr. ) is:{self.matvars['achieved_target']}", "blue"))
        self.jumps = [{"time": self.ref_time, "thrustDuration" : self.matvars['T_th'], "p0": p0,
                    "targetPos": self.targetPos,  "Fleg":self.Fleg,
                    "Fr_r": self.Fr_r0, "Fr_l": self.Fr_l0,  "Tf": self.matvars['Tf'] }]

        # MPC vars (need to perform before normal optim to know Tf)
        self.mpc_N = int(0.4 * self.optim_params['N_dyn'])
        self.Fr_max_mpc = 100.

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
        #a_bool = self.jumps[self.jumpNumber]["time"] > self.jumps[self.jumpNumber]["thrustDuration"]
        #lift_off_idx = min([i for (i, val) in enumerate(a_bool) if val]) - 1
        lift_off_idx = np.max(np.where((self.time_log - self.start_logging) <=self.jumps[self.jumpNumber]["thrustDuration"]))
        impulse_work= 0.5 * self.optim_params['m'] *self.base_vel_log[:, lift_off_idx].dot(self.base_vel_log[:, lift_off_idx]) # ekin at liftoff
        # this integral is done on a rough discretization dt
        touch_down_idx = np.max(np.where( (self.time_log - self.start_logging)  < p.jumps[p.jumpNumber]["Tf"]))
        hoist_work = 0.
        for i in range(touch_down_idx):
            hoist_work = hoist_work + (
                        abs(self.Fr_r_log[i] * self.l_2d_log[i]) + abs(self.Fr_l_log[i] * self.l_1d_log[i])) * conf.robot_params[p.robot_name]['dt']
        return impulse_work + hoist_work

    def startRealRobot(self):
        #TODO
        pass

def talker(p):
    p.start()

    if p.real_robot:
        p.startRealRobot()
    else:
        additional_args = ['robot_name:='+p.robot_name,
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
    p.startupProcedure()
    p.initVars()
    p.q_des = np.copy(p.q_des_q0)

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])
    p.updateKinematicsDynamics()

    # jump params
    # jump starting position
    p0 = np.array([0.28,  2.5, -6.10104])  # there is singularity for px = 0!
    #jump landing position
    pf = np.array([0.28, 4, -4])

    print(colored(f"---------------Ideal Target landing: {pf}", "green"))
    p.startJump = 2.5
    p.orientTime = 1.0
    p.stateMachine = 'idle'
    p.jumpNumber  = 0
    p.numberOfJumps = 1
    p.start_logging = np.inf

    # set the rope base joint variables to initialize in p0 position, the leg ones are defined in params.yaml
    p.q_des[:12] = p.computeJointVariables(p0)
    p.setSimSpeed(dt_sim=0.001, max_update_rate=100, iters=1500)

    while not ros.is_shutdown():
        # update the kinematics
        p.updateKinematicsDynamics()
        # jump state machine
        if ( p.stateMachine == 'idle') and (p.time >= p.startJump):
            # first run optim and fill in jump variable
            p.pause_physics_client()
            p.initOptim(p.base_pos - p.mat2Gazebo, pf)
            p.unpause_physics_client()
            p.des_leg_orient = p.getImpulseAngle()

            #set the end of orienting
            p.end_orienting = p.startJump + p.orientTime
            p.end_thrusting = p.startJump + p.orientTime + p.jumps[p.jumpNumber]["thrustDuration"]
            p.start_logging = p.end_orienting
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
                print(colored(f"Stop orienting leg", "blue"))
                print("\033[34m" + "---------Starting jump  number ", p.jumpNumber, " to optimized target: ",
                      p.jumps[p.jumpNumber]["targetPos"], " from actual p0 : ", p.base_pos - p.mat2Gazebo)
                print(colored(f"Start trusting", "blue"))
                p.tau_ffwd = np.zeros(p.robot.na)
                p.tau_ffwd[p.rope_index] = p.g[p.rope_index]  # compensate gravitu in the virtual joint to go exactly there
                p.pid.setPDjoint(p.base_passive_joints, 0., 0., 0.)
                p.pid.setPDjoint(p.leg_index, 0., 0., 0.)
                print(colored(f"ZERO LEG AND ROPE PD", "red"))
                p.stateMachine = 'thrusting'
                p.pid.setPDjoint(p.rope_index, 0., 0., 0.)
                p.w_Fleg = p.jumps[p.jumpNumber]["Fleg"]

        if (p.stateMachine == 'thrusting'):
            # apply leg inpulse for thust duration
            p.tau_ffwd[p.leg_index] = -p.Jleg.T.dot(p.w_Fleg)
            # plot Fleg
            p.ros_pub.add_arrow(p.x_ee, p.w_Fleg / p.force_scale, "red", scale=2.5)

            # start also applying forces to ropes
            delta_t = p.time - p.end_orienting
            p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)]
            p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)]

            # plot rope forces
            p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=2.5)
            p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=2.5)
            p.tau_ffwd[p.rope_index[0]] = p.Fr_r
            p.tau_ffwd[p.rope_index[1]] = p.Fr_l

            if (p.time > p.end_thrusting):
                print(colored("Stop Trhusting", "blue"))
                print(colored(f"RESTORING LEG PD", "red"))
                # reenable  the PDs of default values for landing and reset the torque on the leg (stop applyng inpulse)
                p.pid.setPDjoint(p.base_passive_joints, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'] ,  0.)
                # reenable leg pd
                p.pid.setPDjoint(p.leg_index, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'],  0.)
                p.tau_ffwd[p.leg_index] = np.zeros(len(p.leg_index))
                p.stateMachine = 'flying'

                # retract leg and move langing elements
                p.q_des[p.leg_index[2]] = 0.25

                # retract leg for landing
                if  p.landing:
                    p.stateMachine = 'flying_and_wait_for_touchdown'
                print(colored("Start "+ p.stateMachine, "blue"))

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

            p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)]+ deltaFr_l0
            p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)]+ deltaFr_r0

            #plot rope forces
            p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=2.5)
            p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=2.5)

            p.tau_ffwd[p.rope_index[0]] = p.Fr_r
            p.tau_ffwd[p.rope_index[1]] = p.Fr_l
            end_flying = p.startJump + p.orientTime +  p.jumps[p.jumpNumber]["Tf"]

            if (p.time >= end_flying):
                print(colored("Stop Flying", "blue"))
                # reset the qdes
                # we need to reset the rope PD because the Fr are finished and I would get the final value repeated  that is not the good thing to do
                p.resetRope()
                energy = p.computeJumpEnergyConsumption()
                p.jumpNumber += 1
                if (p.jumpNumber < p.numberOfJumps):
                    p.stateMachine = 'idle'
                    # reset for multiple jumps
                    p.startJump = p.time
                else:
                    #p.pause_physics_client()
                    landing_location = p.base_pos-p.mat2Gazebo
                    print(colored(f" real landing (in matlab convention) is: {landing_location}", "blue"))
                    print(colored(f" while from optim it should be  {p.targetPos}", "blue"))

                    print(colored(f" the landing error is  {np.linalg.norm(landing_location - p.targetPos)}", "blue"))
                    jump_length = np.linalg.norm(p0[:2] - p.targetPos[:2])
                    MSE = np.square(np.array(p.MPC_tracking_error)).mean()
                    RMSE = math.sqrt(MSE)
                    print(colored(
                        f" the relative landing error (norm per jump lenghth)  is {100*np.linalg.norm(landing_location - p.targetPos) / jump_length}%",
                        "blue"))
                    print(colored(f" the energy consumption is  {energy}", "blue"))
                    print(colored(f" the rmse of MPC tracking error is  {RMSE}", "blue"))
                    print(colored(f" the leg impulse  is  {p.Fleg}", "blue"))
                    print(colored(f" the norm of the leg impulse  is  {np.linalg.norm(p.Fleg)}", "blue"))
                    p.plotStuff()
                    if p.SAVE_BAG:
                        p.recorder.stop_recording_srv()
                    break

        # this is the same as flying but with the lander
        if (p.stateMachine == 'flying_and_wait_for_touchdown'):
            # applying forces to ropes, when time is finished just rset rope length (only once!) and wait for tf
            delta_t = p.time - p.end_orienting
            if p.MPC_control:
                deltaFr_l0, deltaFr_r0, p.prop_force = p.computeMPC(delta_t)
                if p.PROPELLERS:
                    #if p.ADD_NOISE:
                        #prop_force += 0.1*np.sin(2*np.pi*3000/60)
                    p.apply_propeller_force(p.prop_force)
            else:
                deltaFr_l0 = 0.
                deltaFr_r0 = 0.

            if not p.optimal_control_traj_finished:
                if p.getIndex(delta_t) == -1:
                    # start again pid gains and reset qdes
                    p.resetRope()
                    p.optimal_control_traj_finished = True
                else:
                    p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)] +deltaFr_l0
                    p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)] + deltaFr_r0
                # check for early td and in case reset rope
                if p.detectTouchDown():
                    p.resetRope()
                    print(colored("Early TD detected, Start landing", "blue"))
                    p.stateMachine = 'landing'
                    p.start_landing = p.time
            else: # you are checking for delayed TD you have already reset rope and restored PD
                if p.detectTouchDown():
                    print(colored("Start landing", "blue"))
                    p.stateMachine = 'landing'
                    p.start_landing = p.time

            # plot rope forces
            p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=2.5)
            p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=2.5)
            p.tau_ffwd[p.rope_index[0]] = p.Fr_r
            p.tau_ffwd[p.rope_index[1]] = p.Fr_l
            end_flying = p.startJump + p.orientTime + p.jumps[p.jumpNumber]["Tf"]

        if (p.stateMachine == 'landing'):
                print(colored("Start landing", "blue"))
                p.prop_force = (-25.)  # push against the wall
                p.apply_propeller_force(p.prop_force)
                ####TODO
                pass

        # plot ropes as green arrows only when you not save bags because they are ugly
        if not p.SAVE_BAG:
            p.ros_pub.add_arrow(p.anchor_pos, (p.hoist_l_pos - p.anchor_pos), "green", scale=3.)  # arope, already in gazebo
            p.ros_pub.add_arrow(p.anchor_pos2, (p.hoist_r_pos-p.anchor_pos2), "green", scale=3.)  # arope, already in gazebo

        # plot contact force on retractable leg
        p.ros_pub.add_arrow(p.x_ee, p.contactForceW / p.force_scale, "blue", scale=2.5)

        #plot target position (whenever is available)
        try:
            p.ros_pub.add_marker(p.mat2Gazebo + p.jumps[p.jumpNumber]["targetPos"], color="red", radius=0.3, alpha=1.)
            p.ros_pub.add_marker(p.mat2Gazebo + p.targetPosIdeal, color="green", radius=0.5, alpha=0.5)
        except:
            pass
        p.ros_pub.add_marker(p.x_ee, radius=0.05)
        p.ros_pub.publishVisual(delete_markers=False)

        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),4)  # to avoid issues of dt 0.0009999
        if (p.time > p.start_logging):
            p.logData()
        # wait for synconization of the control loop
        rate.sleep()

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
        ros.signal_shutdown("killed")
        p.deregister_node()

    finally:
        ros.signal_shutdown("killed")
        p.deregister_node()
        if p.landing: # for the landing test you should press Ctrl C to stop everything
            p.plotStuff()
            if p.SAVE_BAG:
                p.recorder.stop_recording_srv()


        

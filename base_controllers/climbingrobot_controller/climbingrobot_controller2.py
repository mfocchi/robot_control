# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

from __future__ import print_function

import numpy as np
import rospy as ros
import scipy.linalg

from base_controllers.utils.math_tools import *
import pinocchio as pin
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
import matplotlib.pyplot as plt
from numpy import nan
from base_controllers.utils.common_functions import plotJoint, plotFrameLinear
from termcolor import colored
import os
from rospy import Time
import tf
from base_controllers.base_controller_fixed import BaseControllerFixed
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest
from std_srvs.srv    import Empty, EmptyRequest
import roslaunch
from geometry_msgs.msg import Wrench, Point
from gazebo_msgs.msg import ContactsState
import scipy.io.matlab as mio
import distro
import rospkg
from base_controllers.utils.custom_robot_wrapper import RobotWrapper
from scipy.linalg import block_diag

from base_controllers.utils.matlab_conversions import mat_vector2python, mat_matrix2python
import matlab.engine
import numpy.matlib
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
import pandas as pd

import sys
sys.path.insert(0,'./codegen')

import  base_controllers.params as conf
#robotName = "climbingrobot2landing"
robotName = "climbingrobot2"

class ClimbingrobotController(BaseControllerFixed):
    
    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)
        self.EXTERNAL_FORCE = False
        self.impedance_landing = True
        self.MPC_control = True
        self.PLOT_MPC = False
        self.type_of_disturbance = 'none' # 'none', 'impulse', 'const'
        self.MPC_uses_constraints = True
        self.PROPELLERS = True
        self.MULTIPLE_JUMPS = False # use this for paper to generate targets in an ellipsoid around p0,
        self.SAVE_BAG = False # does not show rope vectors
        self.ADD_NOISE = False #creates multiple jumps, in case of MULTOPLE JUMPS adds noise to velocity in case of disturbance adds noise to disturbance
        self.OBSTACLE_AVOIDANCE = False
        self.obstacle_location = np.array([-0.5, 2.5, -6])

        self.rope_index = np.array([2, 8]) #'wire_base_prismatic_r', 'wire_base_prismatic_l',
        self.leg_index = np.array([12, 13, 14])
        self.hip_pitch_joint = 12
        self.hip_roll_joint = 13
        self.base_passive_joints = np.array([3,4,5, 9,10,11])
        self.anchor_passive_joints = np.array([0,1, 6,7])
        self.impulse_start_count = 0 # start disturbance at different point of the flight phase

        if robot_name == 'climbingrobot2landing':
            self.landing = True
            self.force_scale = 150.
        else:
            self.landing = False
            self.force_scale = 60.
        self.landing_joints = np.array([15, 17])
        self.mountain_thickness = 0.1 # TODO call the launch file passing this parameter
        self.r_leg = 0.3
        print("Initialized climbingrobot controller---------------------------------------------------------------")

    def apply_propeller_force(self, ext_force):
        # create force per to ropes plane
        self.prop_forceW  = self.n_bar * ext_force
        self.ros_pub.add_arrow(self.base_pos, self.prop_forceW/p.force_scale , "blue", scale=3.5)
        wrench = Wrench()
        wrench.force.x = self.prop_forceW [0]
        wrench.force.y = self.prop_forceW [1]
        wrench.force.z = self.prop_forceW [2]
        wrench.torque.x = 0.
        wrench.torque.y = 0.
        wrench.torque.z = 0.
        self.pub_prop_force.publish(wrench)

    def applyWrench(self, Fx=0, Fy=0, Fz=0, Mx=0, My=0, Mz=0,  time_interval=0, start_time=None):
        if start_time is None:
            start_time = ros.Time.now()
        wrench = Wrench()
        wrench.force.x = Fx
        wrench.force.y = Fy
        wrench.force.z = Fz
        wrench.torque.x = Mx
        wrench.torque.y = My
        wrench.torque.z = Mz
        reference_frame = "world" # you can apply forces only in this frame because this service is buggy, it will ignore any other frame
        reference_point = Point(x = 0., y = 0., z = 0.)
        self.apply_body_wrench(body_name=self.robot_name+"::base_link", reference_frame=reference_frame, reference_point=reference_point, wrench=wrench, start_time=start_time,  duration=ros.Duration(time_interval))

    def loadModelAndPublishers(self, xacro_path=None):
        xacro_path = rospkg.RosPack().get_path('climbingrobot_description') + '/urdf/' + p.robot_name + '.xacro'
        additional_urdf_args = ' anchorX:=' + str(conf.robot_params[self.robot_name]['spawn_x'])
        additional_urdf_args += ' anchorY:=' + str(conf.robot_params[self.robot_name]['spawn_y'])
        additional_urdf_args += ' anchorZ:=' + str(conf.robot_params[self.robot_name]['spawn_z'])
        additional_urdf_args += ' anchor2X:=' + str(conf.robot_params[self.robot_name]['spawn_2x'])
        additional_urdf_args += ' anchor2Y:=' + str(conf.robot_params[self.robot_name]['spawn_2y'])
        additional_urdf_args += ' anchor2Z:=' + str(conf.robot_params[self.robot_name]['spawn_2z'])
        super().loadModelAndPublishers(xacro_path=xacro_path, additional_urdf_args=additional_urdf_args)

        self.broadcaster = tf.TransformBroadcaster()
        self.sub_contact= ros.Subscriber("/" + self.robot_name + "/foot_bumper", ContactsState,
                                             callback=self._receive_contact, queue_size=1, buff_size=2 ** 24,
                                             tcp_nodelay=True)
        if p.landing:
            self.sub_contact= ros.Subscriber("/" + self.robot_name + "/foot_landing_l_bumper", ContactsState,
                                                 callback=self._receive_contact_landing_l, queue_size=1, buff_size=2 ** 24,
                                                 tcp_nodelay=True)
            self.sub_contact= ros.Subscriber("/" + self.robot_name + "/foot_landing_r_bumper", ContactsState,
                                                 callback=self._receive_contact_landing_r, queue_size=1, buff_size=2 ** 24,
                                                 tcp_nodelay=True)
        # this is for the matlab optim
        self.eng = matlab.engine.start_matlab()
        self.eng.addpath('./codegen', nargout=0)
        if self.PROPELLERS:
            self.pub_prop_force = ros.Publisher("/base_force", Wrench, queue_size=1, tcp_nodelay=True)
        if self.SAVE_BAG:
            self.recorder = RosbagControlledRecorder(False)
        if self.ADD_NOISE:
            # remove any previous instance
            try:
                os.system('rm *.csv')
            except:
                pass
            print(colored('CREATING NEW CSV TO STORE NOISE TESTS', 'blue'))
            columns = ['test_nr', 'ideal_target', 'optim_target', 'landing_location', 'landing_error', 'relative_error', 'energy', 'rmse']
            if p.type_of_disturbance != 'none':
                columns.append('base_dist')
            self.df = pd.DataFrame(columns=columns)



    def getRobotMass(self):
        robot_link_masses = []
        #get link masses supported by joints
        for idx in self.robot.model.inertias:
            robot_link_masses.append(idx.mass)
        # the robot is supported after this joint
        total_robot_mass = sum(robot_link_masses[self.robot.model.getJointId('wire_base_yaw_l'):])
        return total_robot_mass

    def generateDisturbanceOnHemiSphere(self, min, max):
        # sample_spherical(npoints, ndim=3):
        direction = np.random.randn(3)
        direction /= np.linalg.norm(direction, axis=0)
        # hemisphere
        if direction[2] > 0:
            direction[2] *= -1
        #sample magnitude
        amp = min + max*np.random.uniform(low=0, high=1,size=1)    
        return amp*direction

    def generateWindDisturbance(self,n_test, amp):
        directions = np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]] )
        return amp*directions[n_test,:]

    def updateKinematicsDynamics(self):

        if self.ADD_NOISE:# add white gaussian noise
            # it is better to scale the 5% i.e, the variance after sampling with a higher variance value
            self.state_der_noise = np.array([0.01, 0.2, 0.2]) * numpy.random.normal(0., 1)
            #plt.hist(samples, num_bins)
            #plt.show()
        else:
            self.state_der_noise = np.zeros(3)

        # q is continuously updated
        self.robot.computeAllTerms(self.q, self.qd )
        # joint space inertia matrix
        self.M = self.robot.mass(self.q )
        # bias terms
        self.h = self.robot.nle(self.q  , self.qd )
        #gravity terms
        self.g = self.robot.gravity(self.q )
        #compute ee position  in the world frame
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        # this is expressed in a workdframe with the origin attached to the base frame origin
        self.anchor_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId('anchor')).translation
        self.anchor_pos2 = self.robot.framePlacement(self.q, self.robot.model.getFrameId('anchor_2')).translation
        self.anchor_distance_y = (self.anchor_pos2 - self.anchor_pos)[1]
        self.base_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).translation

        self.w_R_b = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).rotation
        self.x_ee =  self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation
        self.base_rpy = self.math_utils.rot2eul(self.w_R_b)

        self.Jb = self.robot.frameJacobian(self.q, self.robot.model.getFrameId('base_link'), True,  pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        self.omega_b =  self.Jb[3:, :].dot(self.qd)
        self.base_vel = self.Jb[:3, :].dot(self.qd)

        self.hoist_l_pos = self.base_pos +  self.w_R_b.dot(np.array([0.0, -0.05, 0.05]))
        # print("sanitycheck", self.hoist_l_pos - self.robot.framePlacement(self.q, self.robot.model.getFrameId('pre-base3')).translation)
        self.hoist_r_pos = self.base_pos + self.w_R_b.dot(np.array([0.0, 0.05, 0.05]))
        # print("sanitycheck", self.hoist_r_pos - self.robot.framePlacement(self.q, self.robot.model.getFrameId('fake_link')).translation)

        self.rope_direction = (p.hoist_l_pos - p.anchor_pos) / np.linalg.norm(p.hoist_l_pos  - p.anchor_pos)
        self.rope_direction2 = (p.hoist_r_pos - p.anchor_pos2) / np.linalg.norm(p.hoist_r_pos - p.anchor_pos2)

        # compute jacobian of the end effector in the world frame (take only the linear part and the actuated joints part)
        self.J = self.robot.frameJacobian(self.q , self.robot.model.getFrameId(frame_name), True, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
        self.Jleg = self.J[:, self.leg_index]
        self.dJdq = self.robot.frameClassicAcceleration(self.q , self.qd , None,  self.robot.model.getFrameId(frame_name), False).linear

        w_R_wire = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wire')).rotation
        w_R_wire2 = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wire_2')).rotation

        self.mat2Gazebo = self.anchor_pos
        self.base_pos_mat = self.base_pos - self.mat2Gazebo
        self.psi = math.atan2(self.base_pos_mat[0], -self.base_pos_mat[2])
        # use geometric intuition for psid
        n_par = (self.anchor_pos - self.anchor_pos2) / np.linalg.norm(self.anchor_pos - self.anchor_pos2)
        rope2_axis = (self.base_pos - self.anchor_pos2) / np.linalg.norm(self.base_pos - self.anchor_pos2)
        self.n_bar = np.cross(n_par, rope2_axis) / np.linalg.norm(np.cross(n_par, rope2_axis))
        self.psid = (self.n_bar.dot(self.base_vel)) / np.linalg.norm(
            np.cross(n_par, self.base_pos - self.anchor_pos2)) + self.state_der_noise[0]

        # WF matlab to WF Gazebo offset
        hoist_distance = np.linalg.norm(self.hoist_l_pos - self.hoist_r_pos)
        # to get the matlab state from the gazebo prismatic joints we need to consider that the gazebo joints is in zero config
        # when the rope is 2.5 m half of anchor distance (startup at the point in the middle of the anchors)
        self.l_1 = self.q[p.rope_index[1]] - hoist_distance/2 + self.anchor_distance_y/2
        #print("sanitycheck", self.l_1 - np.linalg.norm((self.hoist_l_pos - self.anchor_pos)))
        self.l_1d = self.qd[p.rope_index[1]] + self.state_der_noise[1]
        self.l_2 = self.q[p.rope_index[0]] - hoist_distance/2 + self.anchor_distance_y/2
        #print("sanitycheck", self.l_2 - np.linalg.norm((self.hoist_r_pos - self.anchor_pos2)))
        self.l_2d = self.qd[p.rope_index[0]]+ self.state_der_noise[2]



        # compute com variables
        robotComB = pin.centerOfMass(self.robot.model, self.robot.data, self.q)

        # from ground truth
        self.com = self.robot.robotComW(self.q)

        # the mountain is always wrt to world
        mountain_pos = np.array([- self.mountain_thickness/2, conf.robot_params[self.robot_name]['spawn_y'], 0.0])
        self.broadcaster.sendTransform(mountain_pos, (0.0, 0.0, 0.0, 1.0), ros.Time.now(), '/wall', '/world')
        if self.OBSTACLE_AVOIDANCE:
            self.broadcaster.sendTransform(mountain_pos, (0.0, 0.0, 0.0, 1.0), ros.Time.now(), '/pillar', '/world')

        if p.landing:
            self.x_landing_l = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wheel_l')).translation
            self.x_landing_r = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wheel_r')).translation
            # kinematics of middle point
            self.x_p = 0.5 * (self.x_landing_l + self.x_landing_r)
            self.J_landing_l= self.robot.frameJacobian(self.q, self.robot.model.getFrameId('wheel_l'), True,  pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]
            self.J_landing_r = self.robot.frameJacobian(self.q, self.robot.model.getFrameId('wheel_r'), True,  pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]

    def _receive_contact(self, msg):
        self.contactForceW = np.zeros(3)
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.contactForceW = self.robot.framePlacement(self.q,  self.robot.model.getFrameId("lower_link")).rotation.dot(grf)

    def _receive_contact_landing_l(self, msg):
        self.contactForceW_l = np.zeros(3)
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.contactForceW_l = self.robot.framePlacement(self.q, self.robot.model.getFrameId("wheel_l")).rotation.dot(grf)

    def _receive_contact_landing_r(self, msg):
        self.contactForceW_r = np.zeros(3)
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.contactForceW_r = self.robot.framePlacement(self.q, self.robot.model.getFrameId("wheel_r")).rotation.dot(grf)


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

        self.wall_normal = np.array([1.,0.,0.])

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

    # deprecated: this function cannot be used with closed kin chain
    # def resetBase(self, p0 = None):
    #     # create the message
    #     req_reset_joints = SetModelConfigurationRequest()
    #     req_reset_joints.model_name = self.robot_name
    #     req_reset_joints.urdf_param_name = 'robot_description'
    #     req_reset_joints.joint_names = self.joint_names
    #     req_reset_joints.joint_positions = conf.robot_params[self.robot_name]['q_0'].tolist()
    #
    #     # send request and get response (in this case none)
    #     self.reset_joints(req_reset_joints)
    #
    #     print(colored(f"---------Resetting Base", "blue"))

    # Deprecated
    # def spawnMountain(self):
    #     package = 'gazebo_ros'
    #     executable = 'spawn_model'
    #     name = 'spawn_climb_wall'
    #     namespace = '/'
    #     args = '-urdf -param climb_wall -model mountain -x '+ str(conf.robot_params[self.robot_name]['spawn_x'] - self.mountain_thickness/2)+ ' -y '+ str(conf.robot_params[self.robot_name]['spawn_y'])
    #     node = roslaunch.core.Node(package, executable, name, namespace,args=args,output="screen")
    #     self.launch = roslaunch.scriptapi.ROSLaunch()
    #     self.launch.start()
    #     process = self.launch.launch(node)

    def startupProcedure(self):
        #set PD gains
        super().startupProcedure()

    def plotStuff(self):
        #plot rope forces
        # plt.figure()
        # plt.subplot(2, 1, 1)
        # plt.ylabel("Fr_l")
        # plt.plot(p.ref_time, p.Fr_l0, color='red')
        #
        # plt.grid()
        # plt.subplot(2, 1, 2)
        # plt.ylabel("Fr_r")
        # plt.plot(p.ref_time, p.Fr_r0, color='red')
        # plt.grid()

        # from operator import itemgetter
        # # plot rope joints
        # subset = p.rope_index
        # plotJoint('position', p.time_log, p.q_log[subset, :], p.q_des_log[subset, :],
        #           joint_names=itemgetter(*subset)(conf.robot_params[p.robot_name]['joint_names']))

        if p.numberOfJumps < 2: # do plots only for one jump
            print("PLOTTING")
            # plotFrameLinear('com position', 1, p.time_log, None, p.com_log)
            # plotFrameLinear('contact force', 2, p.time_log, None, p.contactForceW_log)
            actual_com= p.base_pos_log - p.anchor_pos.reshape(3, 1) # is in anchor frame which is WF in matlab
            time_gazebo = p.time_log - p.start_logging
            #plotJoint('position', time_gazebo, p.q_log, p.q_des_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
            if not p.MULTIPLE_JUMPS:
                plot3D('basePos', 2,  ['X', 'Y', 'Z'], time_gazebo, actual_com, p.ref_time, p.ref_com)
            plot3D('states_test_'+str(p.n_test), 3, ['psi', 'l1', 'l2'], time_gazebo, p.simp_model_state_log, p.ref_time, np.vstack((p.ref_psi, p.ref_l_1, p.ref_l_2)) )
            if p.MPC_control:
                filename = f'test_gazebo_MPC_{p.MPC_control}_constraints_{p.MPC_uses_constraints}_dist_{p.type_of_disturbance}.mat'
            else:
                filename = f'test_gazebo_MPC_{p.MPC_control}_dist_{p.type_of_disturbance}.mat'
            #IRIM
            #filename = 'test_irim_gazebo.mat'
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
        angle_hip_pitch = -1.57 + math.atan2(self.jumps[self.jumpNumber]["Fleg"][2], self.jumps[self.jumpNumber]["Fleg"][0])
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

    def computeOrientationControl(self, des_roll, des_pitch):
        # compute desired orientation
        w_R_des = self.math_utils.eul2Rot(np.array([des_roll, 0., des_pitch]))
        # compute rotation matrix from actual orientation of ee to the desired
        b_R_des = self.w_R_b.T.dot(w_R_des)
        # compute the angle-axis representation of the associated orientation error
        # compute the angle:
        delta_theta = math.atan2(np.sqrt(pow(b_R_des[2,1]-b_R_des[1,2], 2) +  pow(b_R_des[0,2]-b_R_des[2,0], 2) + pow(b_R_des[1,0]-b_R_des[0,1], 2)),
                                 b_R_des[0,0]+ b_R_des[1,1]+ b_R_des[2,2]-1 )
        # compute the axis (deal with singularity)
        if delta_theta == 0.0:
            e_error_o = np.zeros(3)
        else:
            r_hat = 1/(2*np.sin(delta_theta))*np.array([b_R_des[2,1]-b_R_des[1,2], b_R_des[0,2]-b_R_des[2,0], b_R_des[1,0]-b_R_des[0,1]])
            # compute the orientation error
            e_error_o = delta_theta * r_hat
        # the error is expressed in the end-effector frame
        # we need to map it in the world frame to compute the moment because the jacobian is in the WF
        w_error_o = self.w_R_b.dot(e_error_o)


        # compute the virtual moment (angular part of the wrench) to realize the orientation task
        W_Gamma_des =  np.multiply(conf.robot_params[p.robot_name]['Ko'], w_error_o) + np.multiply(conf.robot_params[p.robot_name]['Do'],-self.omega_b)
        # selction matrix to remove the pitch
        S = np.array([[1,0,0], [0,0,1]])
        # map to BF and remove the pitch
        B_Gamma_desRY = S.dot(self.w_R_b.dot(W_Gamma_des))
        # build jacobian (3x1)
        J_p = np.hstack((  (self.math_utils.skew(p.hoist_l_pos - p.base_pos).dot(self.rope_direction)).reshape(3,1), (self.math_utils.skew(p.hoist_r_pos - p.base_pos).dot(self.rope_direction2)).reshape(3,1) ))
        # map it to BF and remove pitch
        B_J_pRY = S.dot(self.w_R_b.dot(J_p))


        f_r_fbk = np.linalg.inv(B_J_pRY).dot(B_Gamma_desRY)
        # print("left rope axis", self.rope_direction)
        # print("right rope axis", self.rope_direction2)
        # print("skew left", p.hoist_l_pos - p.base_pos)
        # print("skew right", p.hoist_r_pos - p.base_pos)


        return 0,0# f_r_fbk[0], f_r_fbk[1]

    def detectTouchDown(self):
        force_th = 10.
        if not self.touch_down_detected_l and (self.wall_normal.dot(self.contactForceW_l) > force_th):
            self.touch_down_detected_l = True
        if not self.touch_down_detected_r  and (self.wall_normal.dot(self.contactForceW_r) > force_th):
            self.touch_down_detected_r = True

        if self.touch_down_detected_l and self.touch_down_detected_r:
            print(colored("TouchDown Detected", "blue"))
            #TODO uncomment
            #self.pid.setPDjoint(p.landing_joints, 0., 0, 0.)

            # sample com pos
            self.x_tilde0 =  self.wall_normal.reshape(1, 3) @ (self.com)# - self.x_p)
            return True
        else:
            return False

    def computeLandingControl(self):
        #self.ros_pub.add_marker(self.x_p, radius=0.05, color = "green")
        #compute relative position wrt base and xp and project on landing leg plane (supposed to be aligned with wall normal)
        x_tilde = self.wall_normal.dot(self.com)# - self.x_p)
        xd_tilde = self.wall_normal.dot(self.base_vel)

        # compute impedance law for com
        K_l =  10.
        D_l = 2*math.sqrt(10. * self.getRobotMass())
        f_com =  K_l * (self.x_tilde0 - x_tilde) - D_l * xd_tilde
        if f_com <0:
            f_com = 0.
        f_com_vec =  self.wall_normal*f_com
        self.ros_pub.add_arrow( self.base_pos ,f_com_vec/self.force_scale , "red", scale=3.5)

        # map into feet landing forces
        A = np.zeros((6, 6))
        #sum linear forces
        A[:3, :3] = np.eye(3)
        A[:3, 3:] = np.eye(3)
        A[3:, :3] = self.math_utils.skew(self.x_landing_l - self.com)
        A[3:, 3:] = self.math_utils.skew(self.x_landing_r - self.com)

        # keep recomputing gravity comp TODO Pinocchio does not compute gravity comp because he does not know about the kin loop
        # Todo implement it
        #self.tau_ffwd[p.rope_index] = self.g[p.rope_index]
        # for now we use the actual forces from the pd
        self.Fr_r_actual  = 2*self.g[p.rope_index[1]] #self.tau[p.rope_index[0]] TODO fix this
        self.Fr_l_actual  = self.g[p.rope_index[1]] #self.tau[p.rope_index[1]]

        #print("fcom:", f_com_vec)
        b = np.zeros(6)
        b[:3] = - self.Fr_r_actual -self.Fr_l_actual - self.getRobotMass() * self.robot.model.gravity.vector[:3] + f_com_vec
        b[3:] = -np.cross(self.hoist_l_pos - self.com, self.rope_direction * self.Fr_l_actual) - np.cross(self.hoist_r_pos - self.com, self.rope_direction2 * self.Fr_r_actual)

        F_l = np.linalg.pinv(A).dot(b)  # Fl_l, Fl_r
        #self.ros_pub.add_arrow(self.x_landing_l, F_l[:3] / self.force_scale, "red", scale=3.5)
        #self.ros_pub.add_arrow(self.x_landing_r, F_l[3:] / self.force_scale, "red", scale=3.5)
        #debug
        # self.ros_pub.add_arrow(self.hoist_l_pos,  self.rope_direction *self.Fr_l_actual / self.force_scale, color = "black", scale=3.5)
        # self.ros_pub.add_arrow(self.hoist_r_pos,self.rope_direction2*self.Fr_r_actual / self.force_scale, "blue", scale=3.5)
        # self.ros_pub.add_arrow(self.com, self.getRobotMass() * self.robot.model.gravity.vector[:3] / self.force_scale, "red", scale=3.5)

        # build jacobian extracting columbs from  geom landing jacobians
        Jl = block_diag(self.J_landing_l[:, 15].reshape(3,1), self.J_landing_r[:, 17].reshape(3,1))

        #debug
        # feas_space_proj = np.linalg.pinv(Jl.T).dot(Jl.T)
        # F_l_feas = feas_space_proj.dot(F_l)
        # self.ros_pub.add_arrow(self.x_landing_l, F_l_feas[:3] / self.force_scale, "green", scale=3.5)
        # self.ros_pub.add_arrow(self.x_landing_r, F_l_feas[3:] / self.force_scale, "green", scale=3.5)
        # TODO uncomment
        #tau = -Jl.T.dot(F_l)
        tau = np.zeros(2)
        return tau

    def resetRope(self):
        print(colored(f"RESTORING ROPE PD", "red"))
        # enable PD for rope and reset the PD reference to the new estension
        # sample the new elongation
        self.q_des[p.rope_index[0]] = np.copy(p.q[p.rope_index[0]])
        self.q_des[p.rope_index[1]] = np.copy(p.q[p.rope_index[1]])
        #print("resetting rope joints qdes : ", self.q_des[p.rope_index])
        # stop applying rope forces and restore PD gains on rope joints
        # TODO if you have lateral velocity the rope that has no univlaterality will create
        #  TODO a Fr >0, better not use PD and apply gravity comp self.g[p.rope_index] (actually not computed for the right branch from pinocchio
        # because of kin loop
        self.Fr_r = 0.
        self.Fr_l = 0.
        self.tau_ffwd[p.rope_index] = np.zeros(2)
        self.pid.setPDjoint(p.rope_index, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'], 0.)

    def initOptim(self, p0, pf):
        ##offline optim vars
        self.Fleg_max = 300.
        self.Fr_max = 90.
        self.mu = 0.8


        self.optim_params = {}
        self.optim_params['jump_clearance'] = 1.

        if self.OBSTACLE_AVOIDANCE:
            # I hard code it otherwise does not converge cause it is very sensitive
            p0 = np.array([0.5, 0.5, -6])

        if self.landing:
            self.optim_params['m'] = 15.07 # I need to hardcode it otherwise it does not converge
            # I hardcode this because wall inclination is non 0 so we start from 0.5
            p0 = np.array([0.5, 2.5, -6])
            pf = np.array([0.5, 4, -4])
            self.Fleg_max = 600.
            self.Fr_max = 300.
        else:
            self.optim_params['m'] = self.getRobotMass()
        self.optim_params['obstacle_avoidance'] = self.OBSTACLE_AVOIDANCE
        self.optim_params['obstacle_location'] = matlab.double(self.obstacle_location).reshape(3, 1)
        self.optim_params['num_params'] = 4.
        self.optim_params['int_method'] = 'rk4'
        self.optim_params['N_dyn'] = 30.
        self.optim_params['FRICTION_CONE'] = 1.
        self.optim_params['int_steps'] = 5.
        self.optim_params['contact_normal'] = matlab.double([1., 0., 0.]).reshape(3, 1)
        self.optim_params['b'] = self.anchor_distance_y
        self.optim_params['p_a1'] = matlab.double([0., 0., 0.]).reshape(3, 1)
        self.optim_params['p_a2'] = matlab.double([0., self.optim_params['b'], 0.]).reshape(3, 1)
        self.optim_params['g'] = 9.81
        self.optim_params['w1'] = 1. # smooth
        if not p.MULTIPLE_JUMPS:
            self.optim_params['w2'] = 0. # hoist work
        else:
            self.optim_params['w2'] = 100.  # hoist work use this for multiple jumps for energetic comparison (test are for 0 or 100)
        self.optim_params['w3'] = 0.
        self.optim_params['w4'] = 0.
        self.optim_params['w5'] = 0.
        self.optim_params['w6'] = 0.
        self.optim_params['T_th'] = 0.05


        # for unit test use
        #p0 = np.array([0.5, 2.5, -6])  # there is singularity for px = 0!
        self.matvars = self.eng.optimize_cpp_mex(matlab.double(p0.tolist()), matlab.double(pf.tolist()), self.Fleg_max, self.Fr_max, self.mu, self.optim_params)
        # the result should be achieved_target [[0.5762191796248742], [4.004735278193368], [-3.984719850311409]]
        # Tf 1.2888886080707584
        #print(self.matvars["achieved_target"])
        # print(self.matvars["Tf"])

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
        self.targetPos = self.ref_com[:,-1]

        # matlab validation test (set qkeee = -0.5)
        # self.matvars = mio.loadmat('validation.mat', squeeze_me=True, struct_as_record=False)
        # self.ref_com  =self.matvars['p']
        # self.ref_psi = self.matvars['psi']
        # self.ref_l_1 = self.matvars['l1']
        # self.ref_l_2 = self.matvars['l2']
        # self.ref_time = self.matvars['time']
        # self.Fr_l0 = self.matvars['Fr_l']
        # self.Fr_r0 = self.matvars['Fr_r']
        # self.Fleg = self.matvars['Fleg']

        print(colored(f"offline optimization accomplished, p0:{p0}, target:{self.targetPos}", "blue"))

        # paper IRIM uncomment this
        # self.MPC_control = False
        # self.matvars = mio.loadmat('test_irim.mat', squeeze_me=True, struct_as_record=False)
        # self.ref_com = self.matvars['p']
        # self.ref_psi = np.zeros((len(self.matvars['l1'])))
        # self.ref_l_1 = self.matvars['l1']
        # self.ref_l_2 =  self.matvars['l2']
        # self.ref_time =  self.matvars['time']
        # self.Fr_l0 = self.matvars['Fr_l']
        # self.Fr_r0 = self.matvars['Fr_r']
        # self.Fleg = self.matvars['Fleg']
        # self.targetPos = self.matvars['achieved_target']


        # print(self.Fr_l0)
        # print(self.Fr_r0)

        self.jumps = [{"time": self.ref_time, "thrustDuration" : self.matvars['T_th'], "p0": p0,
                    "targetPos": self.targetPos,  "Fleg":self.Fleg,
                    "Fr_r": self.Fr_r0, "Fr_l": self.Fr_l0,  "Tf": self.matvars['Tf'] }]

        # MPC vars (need to perform optim to know Tf)
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

        if self.landing:
            self.Fr_max_mpc = 150. # this is not used in the mpc it creates issues
        if self.PLOT_MPC:
            self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)
            #self.ax = self.fig.add_subplot(111)

    def computeMPC(self, delta_t):
        # after the thrust we start MPC,  it will start from time 0.05 so the index will start from  2
        if self.getIndex(delta_t) != -1:
            self.mpc_index = self.getIndex(delta_t)

        # # This stops the MPC when self.mpc_index + self.mpc_N = N
        # if ((self.mpc_index + self.mpc_N) < len(self.ref_time)): # do not do anything for the last part
        #     if (self.mpc_index != self.mpc_index_old): # do optim only every dtMPC  not every dt
        #         #debug
        #         # print(self.mpc_index)
        #         # print(delta_t)
        #
        #         # eval ref
        #         ref_com = matlab.double(self.ref_com[:, self.mpc_index:self.mpc_index + self.mpc_N].tolist())
        #         Fr_l0 = matlab.double(self.Fr_l0[self.mpc_index:self.mpc_index + self.mpc_N].tolist())
        #         Fr_r0 = matlab.double(self.Fr_r0[self.mpc_index:self.mpc_index + self.mpc_N].tolist())
        #         actual_t = matlab.double(self.ref_time[self.mpc_index])
        #
        #         # print(ref_com)
        #         # print(Fr_l0)
        #         # print(Fr_r0)
        #         # print(actual_t)
        #         # unit test  normal (no landing)  mpc_index = 1 horizon 0.4 N
        #         #actual_state = matlab.double([0.0937, 6.6182, 6.5577, 0.1738, 1.1335, 1.3561]).reshape(6, 1)
        #         # x= [-63.17725056815982,-18.593042192877622,20.415257331471878,37.003764502121626,34.24067171177194,22.039578469101244,9.443858579323292,0.712971781109941,-3.7611337837956627,-4.9814348273387,-4.734640420584856,-4.402709304195949,-81.26537242370831,-48.586231490584375,-17.39599228319515,-0.3487135265221699,3.9620273273582467,1.8924880643624182,-1.0812319792261356,-2.676715350748216,-3.1404020861190594,-3.200333076663731,-3.3249973760318268,-3.5154072845296485]]
        #         actual_state = matlab.double([ self.psi, self.l_1, self.l_2, self.psid, self.l_1d, self.l_2d]).reshape(6,1)
        #         #print(actual_state)
        #
        #         self.pause_physics_client()
        #         #perform optimization
        #         if self.MPC_uses_constraints:
        #             x = mat_vector2python(self.eng.optimize_cpp_mpc_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, self.Fr_max_mpc, self.mpc_N, self.optim_params_mpc))
        #         else:
        #             x = mat_vector2python(self.eng.optimize_cpp_mpc_no_constraints_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, self.Fr_max_mpc, self.mpc_N, self.optim_params_mpc))
        #
        #         # extract optim vars
        #         self.deltaFr_l = x[:self.mpc_N]
        #         self.deltaFr_r = x[self.mpc_N:]
        #         # online plot MPC
        #         p.onlinePlotMPC(self.deltaFr_l, self.deltaFr_r)
        #         self.unpause_physics_client()

        else:  # whenever the MPC should not be updated anymore use delta_t to imncrement mpc_index_ffwd
            #print("delta_t MOD dtMpc", (delta_t % self.optim_params_mpc['mpc_dt']))
            if (delta_t % self.optim_params_mpc['mpc_dt']) < 0.001:  # increment mpc_index_ffwd every mpc_dt
                self.mpc_index_ffwd += 1
                if self.mpc_index_ffwd > (self.mpc_N-1): # reference is finished keep the last computed one
                    self.mpc_index_ffwd = self.mpc_N-1
                #debug
                #print("stop mpc, applying ffwd, mpc_index_ffwd: ", self.mpc_index_ffwd)

        # This is better for const dist cause it keeps optimizing till the end
        if (self.mpc_index != self.mpc_index_old): # do optim only every dtMPC  not every dt
            # reduce MPC horizon gradually at the end
            if ((self.mpc_index + self.mpc_N) >=len(self.ref_time)):
                self.mpc_N -=1
            # eval ref
            ref_com = matlab.double(self.ref_com[:, self.mpc_index:self.mpc_index + self.mpc_N].tolist())
            Fr_l0 = matlab.double(self.Fr_l0[self.mpc_index:self.mpc_index + self.mpc_N].tolist())
            Fr_r0 = matlab.double(self.Fr_r0[self.mpc_index:self.mpc_index + self.mpc_N].tolist())
            actual_t = matlab.double(self.ref_time[self.mpc_index])


            # unit test  normal (no landing)  mpc_index = 1 horizon 0.4 N
            #actual_state = matlab.double([0.0937, 6.6182, 6.5577, 0.1738, 1.1335, 1.3561]).reshape(6, 1)
            # x= [-63.17725056815982,-18.593042192877622,20.415257331471878,37.003764502121626,34.24067171177194,22.039578469101244,9.443858579323292,0.712971781109941,-3.7611337837956627,-4.9814348273387,-4.734640420584856,-4.402709304195949,-81.26537242370831,-48.586231490584375,-17.39599228319515,-0.3487135265221699,3.9620273273582467,1.8924880643624182,-1.0812319792261356,-2.676715350748216,-3.1404020861190594,-3.200333076663731,-3.3249973760318268,-3.5154072845296485]]
            actual_state = matlab.double([ self.psi, self.l_1, self.l_2, self.psid, self.l_1d, self.l_2d]).reshape(6,1)
            #print(actual_state)

            self.pause_physics_client()
            #perform optimization
            if p.PROPELLERS:
                x = mat_vector2python(self.eng.optimize_cpp_mpc_propellers_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, self.Fr_max_mpc, self.mpc_N, self.optim_params_mpc))
                # extract optim vars
                self.deltaFr_l = x[:self.mpc_N]
                self.deltaFr_r = x[self.mpc_N:2*self.mpc_N]
                self.propeller_force = x[2*self.mpc_N:3*self.mpc_N]
            else:
                if self.MPC_uses_constraints:
                    x = mat_vector2python(self.eng.optimize_cpp_mpc_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0,self.Fr_max_mpc, self.mpc_N,self.optim_params_mpc))
                else:
                    x = mat_vector2python(self.eng.optimize_cpp_mpc_no_constraints_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, self.Fr_max_mpc, self.mpc_N, self.optim_params_mpc))
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
        #MPC action
        self.ax1.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N], deltaFr_l, "or-")
        self.ax2.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N], deltaFr_r, "or-")
        # full action
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

    def generateTargetPoints(self, p0):
        # generate points in an ellipse of axis a b = 2.5 around p0
        alpha = np.deg2rad(45)
        theta = np.linspace(alpha, alpha + 2 * np.pi, 9)
        # main axes ellipse
        a =  (self.anchor_distance_y-1.5) /2 / np.cos(alpha)
        b = 2
        y = a * np.cos(theta[:-1])
        z = b * np.sin(theta[:-1])

        Rx = np.array([[np.cos(-alpha), -np.sin(-alpha)], [np.sin(-alpha), np.cos(-alpha)]])
        # sets 0 coord equal to p0
        cw = numpy.matlib.repmat(p0.reshape(3,1).copy(), 1, 8)
        cw[1:, :] += Rx.dot(np.vstack((y, z)))
        return cw
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

def talker(p):
    p.start()
    additional_args = ['spawn_2x:=' + str(conf.robot_params[p.robot_name]['spawn_2x']),
                       'spawn_2y:=' + str(conf.robot_params[p.robot_name]['spawn_2y']),
                       'spawn_2z:=' + str(conf.robot_params[p.robot_name]['spawn_2z']),
                       'obstacle:='+str(p.OBSTACLE_AVOIDANCE),
                       'obstacle_x:=' + str(p.obstacle_location[0]),
                       'obstacle_y:=' + str(p.obstacle_location[1]),
                       'obstacle_z:=' + str(p.obstacle_location[2])
                       ]
    if p.landing:
        additional_args.append('wall_inclination:='+ str(conf.robot_params[p.robot_name]['wall_inclination']))
    if p.SAVE_BAG:
        additional_args.append('rviz_conf:='+rospkg.RosPack().get_path('climbingrobot_description') + '/rviz/conf_paper_tro.rviz')

    p.startSimulator(world_name="climbingrobot2.world",additional_args=additional_args)
    p.loadModelAndPublishers()

    p.startupProcedure()
    p.initVars()
    p.q_des = np.copy(p.q_des_q0)

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])
    p.updateKinematicsDynamics()



    # 1----very simple jump with apply wrench to detach from wall and kinematics
    #jumpN = 0
    # while not ros.is_shutdown():
    #     p.updateKinematicsDynamics()
    #     #multiple jump test
    #     if (jumpN == 0) and (p.time >3.): #change target
    #         p.pid.setPDjoint(p.anchor_passive_joints, 0., 0., 0.)
    #         p0 = np.array([0.0, 3, -8])
    #         print(colored(f"Jumpping to {p0}", "red"))
    #         p.applyWrench(100, 0, 0., time_interval=0.02)
    #         p.q_des[:12] = p.computeJointVariables(p0)
    #         print(p.q_des[:12])
    #         jumpN += 1
    #     if (jumpN == 1) and (p.time > 5.5):  # change target
    #         p0 = np.array([0.0, 4.5, -8])
    #         print(colored(f"Jumpping to {p0}", "red"))
    #         p.applyWrench(100, 0, 0., time_interval=0.02)
    #         p.q_des[:12] = p.computeJointVariables(p0)
    #         print(p.q_des[:12])
    #         jumpN += 1
    #     if (jumpN == 2) and (p.time > 10):  # change target
    #         p0 = np.array([0.0, 2, -6])
    #         print(colored(f"Jumpping to {p0}", "red"))
    #         p.applyWrench(100, 0, 0., time_interval=0.02)
    #         p.q_des[:12] = p.computeJointVariables(p0)
    #         print(p.q_des[:12])
    #         jumpN += 1
    #     # compensate gravity only for robpe joint
    #     p.tau_ffwd[p.rope_index] = p.g[p.rope_index]
    #     p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
    #     p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3)  # to avoid issues of dt 0.0009999
    #     p.logData()
    #     p.ros_pub.add_arrow(p.anchor_pos, (p.base_pos - p.anchor_pos), "green", scale=3.)  # arope, already in gazebo
    #     p.ros_pub.add_arrow(p.anchor_pos2, (p.base_pos - p.anchor_pos2), "green", scale=3.)  # arope, already in gazebo
    #     p.ros_pub.add_arrow(p.x_ee, p.contactForceW / 5., "blue", scale=4.5)
    #     p.ros_pub.add_marker(p.mat2Gazebo + p0, color="red", radius=0.2)
    #     p.ros_pub.publishVisual()
    #     rate.sleep()

    # # 2 ---validation test matlab  (do a jump with constant fr and fleg_x, to test also the )
    # jump_state = 'start'
    # # the jump will start after 2 seconds where the robot will have settled down to the init config
    # p.startJump = 2.
    # p.numberOfJumps = 1
    # p0 = np.array([0.28, 2.5, -10.10104])  # there is singularity for px = 0!
    # # set the rope base joint variables to initialize in p0 position, the leg ones are defined in params.yaml
    # p.q_des[:12] = p.computeJointVariables(p0)
    # while not ros.is_shutdown():
    #     p.updateKinematicsDynamics()
    #     if (jump_state == 'start') and (p.time >p.startJump): #set the  impulse
    #         print(colored(f"Start Matlab Validation Test with constant Fr", "red"))
    #         p.pid.setPDjoint(p.anchor_passive_joints, 0., 0., 0.)
    #         p.pid.setPDjoint(p.rope_index, 0., 0., 0.)
    #         if p.landing:
    #             Fr_l0 = -90
    #             Fr_r0 = -90
    #             Fleg = 1000 #with landing you need higher impulse
    #         else:
    #             Fr_l0 = -30
    #             Fr_r0 = -40
    #             Fleg = 200
    #
    #         p.w_Fleg = np.array([Fleg, 0., 0.])
    #         if p.EXTERNAL_FORCE:
    #             p.applyWrench(Fleg[0], Fleg[1], Fleg[2], time_interval=0.05)
    #         jump_state = 'thrusting'
    #
    #     if (jump_state == 'thrusting'):
    #         if not p.EXTERNAL_FORCE and p.time<(p.startJump + 0.05):
    #             p.tau_ffwd[p.leg_index] = -p.Jleg.T.dot(p.w_Fleg)
    #             p.ros_pub.add_arrow(p.x_ee, p.w_Fleg / 20., "red", scale=4.5)
    #         else:
    #             # apply disturbance to see self stabiliyizing effect
    #             p.applyWrench(Mz=400, time_interval=0.1)
    #             p.tau_ffwd[p.leg_index] = np.zeros(3)
    #             if  p.landing:
    #                 # retract knee joint and extend landing joints
    #                 p.tau_ffwd[p.landing_joints] = np.zeros(2)
    #                 #retract leg and move langing elements
    #                 p.q_des[p.leg_index[2]] = 0.3
    #                 #p.q_des[p.landing_joints] = np.array([-0.8, 0.8]) # we comment this because to match the matlab sim we should stop when the base_x reaches spawnx
    #             jump_state = 'flying'
    #
    #     if (jump_state == 'flying'):# use forces
    #         # compute orientation controller
    #         p.Fr_l_fbk, p.Fr_r_fbk = p.computeOrientationControl(0.,0.)
    #         p.Fr_l = Fr_l0 + p.Fr_l_fbk
    #         p.Fr_r = Fr_r0 + p.Fr_r_fbk
    #         p.tau_ffwd[p.rope_index[0]] = p.Fr_r
    #         p.tau_ffwd[p.rope_index[1]] = p.Fr_l
    #         if (p.base_pos[0] < conf.robot_params[p.robot_name]['spawn_x']):
    #             print(colored(f"target in matlab is: {p.base_pos-p.mat2Gazebo}"))
    #             break
    #
    #     # plot stuff
    #     p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / 50., "red", scale=4.5)
    #     p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / 50., "red", scale=4.5)
    #
    #
    #     p.ros_pub.add_arrow(p.anchor_pos, (p.hoist_l_pos - p.anchor_pos), "green", scale=3.)  # arope, already in gazebo
    #     p.ros_pub.add_arrow(p.anchor_pos2, (p.hoist_r_pos-p.anchor_pos2), "green", scale=3.)  # arope, already in gazebo
    #     if p.landing:
    #         p.ros_pub.add_arrow(p.x_landing_l, p.contactForceW_l / 20., "blue", scale=4.5)
    #         p.ros_pub.add_arrow(p.x_landing_r, p.contactForceW_r / 20., "blue", scale=4.5)
    #     p.ros_pub.add_arrow(p.x_ee, p.contactForceW / 20., "blue", scale=4.5)
    #     p.ros_pub.add_marker(p.mat2Gazebo + p0, color="red", radius=0.2)
    #
    #     p.ros_pub.publishVisual()
    #     p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
    #     p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),
    #                       3)  # to avoid issues of dt 0.0009999
    #     if (p.time > p.startJump):
    #         p.logData()
    #     rate.sleep()
    #
    # plotFrameLinear('position', p.time_log, Pose_log=p.base_rpy_log, title='Roll Pitch Yaw')
    # p.deregister_node()

    #3 --- whole pipeline with variable Fr_l Fr_r and the Fleg x,y,z
    # single jump
    # p0 is defined wrt anchor1 pos in matlab convention
    # jump params
    p0 = np.array([0.28,  2.5, -6.10104])  # there is singularity for px = 0!

    if p.MULTIPLE_JUMPS:
        landingW = p.generateTargetPoints(p0)
        if p.ADD_NOISE:
            landingW = np.repeat(landingW, repeats=50, axis=1)
            #print(landingW)
    else:
        if p.OBSTACLE_AVOIDANCE:
            # old one vertical
            #landingW = np.array([0.28, 4, -10]).reshape(3,1)
            # new one horizontal
            landingW = np.array([0.5, 4.5, -6]).reshape(3, 1)
            p0 = np.array([0.5, 0.5, -6])
        else:
            landingW = np.array([0.28, 4, -4]).reshape(3, 1)

        if p.ADD_NOISE:
            if p.type_of_disturbance == 'impulse':
                number_of_tests = 100
            else:
                number_of_tests = 6
            landingW = np.matlib.repmat(landingW, 1, number_of_tests)

    for p.n_test in range(landingW.shape[1]):
        pf = landingW[:,p.n_test]
        print(colored(f"---------------Ideal Reference landing test # {p.n_test}: {pf}", "green"))

        # jump parameters
        if p.MULTIPLE_JUMPS:
            p.startJump = 2.5 # wait more for longer jumps to initialize
        else:
            p.startJump = 2.5
        p.orientTime = 1.0
        p.stateMachine = 'idle'
        p.jumpNumber  = 0
        p.numberOfJumps = 1
        p.start_logging = np.inf

        # set the rope base joint variables to initialize in p0 position, the leg ones are defined in params.yaml
        p.q_des[:12] = p.computeJointVariables(p0)

        #p.setSimSpeed(dt_sim=0.001, max_update_rate=300, iters=1500)

        while not ros.is_shutdown():

            # update the kinematics
            p.updateKinematicsDynamics()

            #multiple jumps state machine
            if ( p.stateMachine == 'idle') and (p.time >= p.startJump) and (p.jumpNumber<p.numberOfJumps):
                # first run optim and fill in jump variable
                p.pause_physics_client()
                p.initOptim(p.base_pos - p.mat2Gazebo, pf)
                p.unpause_physics_client()
                print(colored(f"Start orienting leg to (pitch, roll)  : {p.getImpulseAngle()}", "blue"))
                p.q_des[p.hip_pitch_joint], p.q_des[p.hip_roll_joint] = p.getImpulseAngle()

                #set the end of orienting
                p.end_orienting = p.startJump + p.orientTime
                p.end_thrusting = p.startJump + p.orientTime + p.jumps[p.jumpNumber]["thrustDuration"]
                p.start_logging = p.end_orienting
                p.stateMachine = 'orienting_leg'  # this phase only waits is not doing anything
                if p.SAVE_BAG:
                    p.recorder.start_recording_srv()


            if (p.stateMachine == 'orienting_leg') and (p.time >= p.end_orienting):
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
                p.ros_pub.add_arrow(p.x_ee, p.w_Fleg / p.force_scale, "red", scale=4.5)

                # start also applying forces to ropes
                delta_t = p.time - p.end_orienting
                p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)]
                p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)]

                # plot rope forces
                p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=4.5)
                p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=4.5)
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
                    # apply diusturbance to show max leg reorientation
                    #p.applyWrench(Mz=100, time_interval=0.2)

                    # manage lander retracting leg
                    if  p.landing:
                        # extend landing joints
                        p.tau_ffwd[p.landing_joints] = np.zeros(2)
                        if p.impedance_landing:
                            p.q_des[p.landing_joints] = np.array([-0.6, 0.6])
                            p.stateMachine = 'flying_and_reorient_lander'
                    print(colored("Start "+ p.stateMachine, "blue"))

                    #add impulsive disturbance
                    p.delayed_start = 0.
                    if p.type_of_disturbance == 'impulse':
                        p.dist_duration = 0.1
                        p.base_dist = np.array([50., -50., 30.])

                        if p.ADD_NOISE:
                            if (p.n_test % 10) == 0:
                                p.impulse_start_count+=1
                                print(colored(f'APPLYING IMPULSE AT {p.impulse_start_count*10}% of the flying phase\n', 'red'))
                                p.delayed_start = p.impulse_start_count * (p.jumps[p.jumpNumber]["Tf"] - p.jumps[p.jumpNumber]["thrustDuration"])/10
                            p.base_dist = p.generateDisturbanceOnHemiSphere(25, 25)
                            print(colored(f"generated disturbance direction {p.base_dist}","red"))
                            

                    #add constant disturbance
                    if p.type_of_disturbance == 'const':
                        p.dist_duration = p.jumps[p.jumpNumber]["Tf"] - p.jumps[p.jumpNumber]["thrustDuration"]
                        p.base_dist = np.array([7., -7., 0.])
                        if p.ADD_NOISE:
                            p.base_dist = p.generateWindDisturbance(p.n_test,7)

                    if p.type_of_disturbance != 'none':
                        p.applyWrench(p.base_dist[0],p.base_dist[1],p.base_dist[2], time_interval=p.dist_duration, start_time=ros.Time.now()+ros.Duration(p.delayed_start))

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

                #if p.type_of_disturbance != 'none':
                    # if ((delta_t-p.delayed_start)>=0) and ((delta_t-p.delayed_start) < p.dist_duration):
                    #     p.ros_pub.add_arrow(p.base_pos,  p.base_dist / 10., "green", scale=16.5)

                p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)]+ deltaFr_l0
                p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)]+ deltaFr_r0

                #plot rope forces
                p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=4.5)
                p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=4.5)

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
                        print(colored(f" the error is  {np.linalg.norm(landing_location - p.targetPos)}", "blue"))
                        jump_length = np.linalg.norm(p0[:2] - p.targetPos[:2])
                        MSE = np.square(np.array(p.MPC_tracking_error)).mean()
                        RMSE = math.sqrt(MSE)
                        print(colored(
                            f" the relative error is {np.linalg.norm(landing_location - p.targetPos) / jump_length}",
                            "blue"))
                        print(colored(f" the energy consumption is  {energy}", "blue"))
                        print(colored(f" the rmse of tracking error is  {RMSE}", "blue"))

                        if p.ADD_NOISE:
                            dict = {'test_nr': p.n_test, 'ideal_target': landingW[:,p.n_test], 'optim_target': p.targetPos,'landing_location':landing_location, 'landing_error': np.linalg.norm(landing_location - p.targetPos),
                                    'relative_error': np.linalg.norm(landing_location - p.targetPos) / jump_length,'energy':energy, 'rmse': RMSE}
                            if p.type_of_disturbance != 'none':
                                dict['base_dist'] = p.base_dist
                            df_dict = pd.DataFrame([dict])
                            p.df = pd.concat([p.df, df_dict], ignore_index=True)
                            #print(p.df.head())
                            filename = f'noise_multiple_{p.MULTIPLE_JUMPS}_dist_{p.type_of_disturbance}.csv'
                            p.df.to_csv(filename, index=None)
                        # fundamental: save everything before initVars! cause it will be deleted
                        else:
                            p.plotStuff()
                        if p.SAVE_BAG:
                            p.recorder.stop_recording_srv()
                        #reset for the next jump
                        if p.MULTIPLE_JUMPS or p.ADD_NOISE:
                            p.startupProcedure()
                            p.initVars()
                            p.q_des = np.copy(p.q_des_q0)

                        break
            # this is the same as flying but with the lander
            if (p.stateMachine == 'flying_and_reorient_lander'):
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

                if p.type_of_disturbance != 'none':
                    if ((delta_t - p.delayed_start) >= 0) and ((delta_t - p.delayed_start) < p.dist_duration):
                        p.ros_pub.add_arrow(p.base_pos, p.base_dist / 10., "blue", scale=4.5)

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
                else: # you are checking for delayed TD you have already reset rope and restored PD
                    if p.detectTouchDown():
                        print(colored("Start landing", "blue"))
                        p.stateMachine = 'landing'

                # plot rope forces
                p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=4.5)
                p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=4.5)
                p.tau_ffwd[p.rope_index[0]] = p.Fr_r
                p.tau_ffwd[p.rope_index[1]] = p.Fr_l
                end_flying = p.startJump + p.orientTime + p.jumps[p.jumpNumber]["Tf"]

                # reorient legs to land parallel to the wall
                rpy = p.math_utils.rot2eul(p.w_R_b)
                p.q_des[p.landing_joints] = np.array([-0.8, 0.8]) - np.array([rpy[2], rpy[2]])


            if (p.stateMachine == 'landing'):
                p.tau_ffwd[p.landing_joints] = p.computeLandingControl()

            # plot ropes as green arrows
            if not p.SAVE_BAG:
                p.ros_pub.add_arrow(p.anchor_pos, (p.hoist_l_pos - p.anchor_pos), "green", scale=3.)  # arope, already in gazebo
                p.ros_pub.add_arrow(p.anchor_pos2, (p.hoist_r_pos-p.anchor_pos2), "green", scale=3.)  # arope, already in gazebo
            # plot contact forces on landing legs
            if p.landing:
                p.ros_pub.add_arrow(p.x_landing_l, p.contactForceW_l / p.force_scale, "blue", scale=4.5)
                p.ros_pub.add_arrow(p.x_landing_r, p.contactForceW_r / p.force_scale, "blue", scale=4.5)
            # plot contact force on retractable leg
            p.ros_pub.add_arrow(p.x_ee, p.contactForceW / p.force_scale, "blue", scale=4.5)

            #plot target position (whenever is available)
            try:
                p.ros_pub.add_marker(p.mat2Gazebo + p.jumps[p.jumpNumber]["targetPos"], color="red", radius=0.3)
            except:
                pass
            p.ros_pub.add_marker(p.x_ee, radius=0.05)
            p.ros_pub.publishVisual()

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


        

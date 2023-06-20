# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

from __future__ import print_function

import numpy as np
import rospy as ros
import scipy.linalg

from utils.math_tools import *
import pinocchio as pin
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
import matplotlib.pyplot as plt
from numpy import nan
from utils.common_functions import plotJoint, plotFrameLinear
from termcolor import colored
import os
from rospy import Time
import tf
from base_controller_fixed import BaseControllerFixed
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

import  params as conf
robotName = "climbingrobot2landing"
#robotName = "climbingrobot2"

class ClimbingrobotController(BaseControllerFixed):
    
    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)
        self.EXTERNAL_FORCE = False
        self.impedance_landing = True

        self.rope_index = np.array([2, 8]) #'wire_base_prismatic_r', 'wire_base_prismatic_l',
        self.leg_index = np.array([12, 13, 14])
        self.hip_pitch_joint = 12
        self.hip_roll_joint = 13
        self.base_passive_joints = np.array([3,4,5, 9,10,11])
        self.anchor_passive_joints = np.array([0,1, 6,7])

        if robot_name == 'climbingrobot2landing':
            self.landing = True
            self.force_scale = 100.
        else:
            self.landing = False
            self.force_scale = 20.
        self.landing_joints = np.array([15, 17])
        self.mountain_thickness = 0.1 # TODO call the launch file passing this parameter
        self.r_leg = 0.3
        print("Initialized climbingrobot controller---------------------------------------------------------------")

    def applyWrench(self, Fx=0, Fy=0, Fz=0, Mx=0, My=0, Mz=0,  time_interval=0):
        wrench = Wrench()
        wrench.force.x = Fx
        wrench.force.y = Fy
        wrench.force.z = Fz
        wrench.torque.x = Mx
        wrench.torque.y = My
        wrench.torque.z = Mz
        reference_frame = "world" # you can apply forces only in this frame because this service is buggy, it will ignore any other frame
        reference_point = Point(x = 0., y = 0., z = 0.)
        self.apply_body_wrench(body_name=self.robot_name+"::base_link", reference_frame=reference_frame, reference_point=reference_point, wrench=wrench, start_time=ros.Time(),  duration=ros.Duration(time_interval))

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
        #compute ee position  in the world frame  
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        # this is expressed in a workdframe with the origin attached to the base frame origin
        self.anchor_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId('anchor')).translation
        self.anchor_pos2 = self.robot.framePlacement(self.q, self.robot.model.getFrameId('anchor_2')).translation
        self.base_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).translation

        self.w_R_b = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).rotation
        self.x_ee =  self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation
        self.base_rpy = self.math_utils.rot2eul(self.w_R_b)

        self.Jb = self.robot.frameJacobian(self.q, self.robot.model.getFrameId('base_link'), True,  pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        self.omega_b =  self.Jb[3:, :].dot(self.qd)
        self.base_vel = self.Jb[:3, :].dot(self.qd)

        self.hoist_l_pos = self.base_pos +  self.w_R_b.dot(np.array([0.0, -0.05, 0.05]))
        self.hoist_r_pos = self.base_pos + self.w_R_b.dot(np.array([0.0, 0.05, 0.05]))
        self.rope_direction = (p.hoist_l_pos - p.anchor_pos) / np.linalg.norm(p.hoist_l_pos  - p.anchor_pos)
        self.rope_direction2 = (p.hoist_r_pos - p.anchor_pos2) / np.linalg.norm(p.hoist_r_pos - p.anchor_pos2)

        # compute jacobian of the end effector in the world frame (take only the linear part and the actuated joints part)
        self.J = self.robot.frameJacobian(self.q , self.robot.model.getFrameId(frame_name), True, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
        self.Jleg = self.J[:, self.leg_index]
        self.dJdq = self.robot.frameClassicAcceleration(self.q , self.qd , None,  self.robot.model.getFrameId(frame_name), False).linear

        w_R_wire = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wire')).rotation
        w_R_wire2 = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wire_2')).rotation


        self.l = self.q[p.rope_index[0]]
        self.ldot = self.qd[p.rope_index[0]]
        self.l_2 = self.q[p.rope_index[1]]
        self.ldot_2 = self.qd[p.rope_index[1]]

        # WF matlab to WF Gazebo offset
        self.mat2Gazebo = self.anchor_pos

        # compute com variables
        robotComB = pin.centerOfMass(self.robot.model, self.robot.data, self.q)

        # from ground truth
        self.com = self.robot.robotComW(self.q)

        # the mountain is always wrt to world
        mountain_pos = np.array([- self.mountain_thickness/2, conf.robot_params[self.robot_name]['spawn_y'], 0.0])
        self.broadcaster.sendTransform(mountain_pos, (0.0, 0.0, 0.0, 1.0), ros.Time.now(), '/wall', '/world')
        # self.broadcaster.sendTransform(mountain_pos, (0.0, 0.0, 0.0, 1.0), ros.Time.now(), '/pillar', '/world')
        # self.broadcaster.sendTransform(mountain_pos, (0.0, 0.0, 0.0, 1.0), ros.Time.now(), '/pillar2', '/world')
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
        self.touch_down_detected_l = False
        self.touch_down_detected_r = False
        self.optimal_control_traj_finished = False

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

        self.wall_normal = np.array([1.,0.,0.])


    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                # self.simp_model_state_log[:, self.log_counter] = np.array([self.theta, self.phi, self.l])
                # self.ldot_log[self.log_counter] = self.ldot
                self.base_pos_log[:, self.log_counter] = self.base_pos
                self.base_rpy_log[:, self.log_counter] = self.base_rpy
                self.Fr_l_log[self.log_counter] = self.Fr_l
                self.Fr_r_log[self.log_counter] = self.Fr_r
                self.Fr_l_fbk_log[self.log_counter] = self.Fr_l_fbk
                self.Fr_r_fbk_log[self.log_counter] = self.Fr_r_fbk

                #self.time_jump_log[self.log_counter] = self.time - self.end_thrusting
                pass

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
        if p.numberOfJumps < 2: # do plots only for one jump
            print("PLOTTING")
            # plotFrameLinear('com position', 1, p.time_log, None, p.com_log)
            # plotFrameLinear('contact force', 2, p.time_log, None, p.contactForceW_log)
            traj_gazebo= p.base_pos_log - p.anchor_pos.reshape(3, 1) # is in anchor frame
            time_gazebo = p.time_log - p.start_logging
            #plotJoint('position', 0, time_gazebo, p.q_log, p.q_des_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
            #plotJoint('torque', 1, time_gazebo, None, None, None, None, None,None, p.tau_ffwd_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
            plot3D('basePos', 2,  ['X', 'Y', 'Z'], time_gazebo, traj_gazebo , p.matvars['solution'].time, p.matvars['solution'].p )
            #plot3D('states', 3, ['theta', 'phi', 'l'], time_gazebo, p.simp_model_state_log)
            mio.savemat('test_gazebo2.mat', {'solution': p.matvars['solution'], 'mu': p.matvars['mu'],
                                            'Fleg':p.matvars['solution'].Fleg, 'Fr_max':p.matvars['Fr_max'], 'p0':p.matvars['p0'],'pf': p.matvars['pf'],
                                            'time_gazebo': time_gazebo, 'traj_gazebo': traj_gazebo})


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


    # compute the passive and rope joints reference from the matlab position referred to a world frame located in between anchor
    def computeJointVariables(self, p):
        # mountain_wire_pitch_l = math.atan2(p[0]-conf.robot_params[self.robot_name]['spawn_x'], -p[2])
        # mountain_wire_pitch_r = math.atan2(p[0]-conf.robot_params[self.robot_name]['spawn_2x'], -p[2])
        mountain_wire_pitch_l = math.atan2(p[0] , -p[2])
        mountain_wire_pitch_r = math.atan2(p[0] , -p[2])

        anchor_distance_y = (self.anchor_pos2 - self.anchor_pos)[1]
        mountain_wire_roll_l = -math.atan2(-p[2], p[1])
        mountain_wire_roll_r = math.atan2(-p[2], anchor_distance_y-p[1])

        wire_base_prismatic_l = np.linalg.norm(p) -anchor_distance_y*0.5
        wire_base_prismatic_r = math.sqrt(p[0]*p[0] +(anchor_distance_y - p[1])*(anchor_distance_y - p[1]) + p[2] * p[2])-anchor_distance_y*0.5

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
        # TODO apply gravity comp self.g[p.rope_index]
        self.Fr_r = 0.
        self.Fr_l = 0.
        self.tau_ffwd[p.rope_index] = np.zeros(2)
        self.pid.setPDjoint(p.rope_index, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'], 0.)

def talker(p):
    p.start()
    additional_args = ['spawn_2x:=' + str(conf.robot_params[p.robot_name]['spawn_2x']),
                       'spawn_2y:=' + str(conf.robot_params[p.robot_name]['spawn_2y']),
                       'spawn_2z:=' + str(conf.robot_params[p.robot_name]['spawn_2z'])]
    if p.landing:
        additional_args.append('wall_inclination:='+ str(conf.robot_params[p.robot_name]['wall_inclination']))
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

    # 2 ---validation test matlab  (do a jump with constant fr and fleg_x)
    # jump_state = 'start'
    # # the jump will start after 2 seconds where the robot will have settled down to the init config
    # p.startJump = 2.5
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
    #             #p.applyWrench(Mz=200, time_interval=0.1)
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
    # p.deregister_node()
    #
    #

    #3 --- whole pipeline with variable Fr_l Fr_r and the Fleg x,y,z
    # single jump
    if p.landing:
        p.matvars = mio.loadmat('test_matlab2landingClearance.mat', squeeze_me=True,struct_as_record=False)
    else:
        p.matvars = mio.loadmat('test_matlab2.mat', squeeze_me=True, struct_as_record=False)
    p.jumps = [{"time": p.matvars['solution'].time, "thrustDuration" : p.matvars['solution'].T_th, "p0": p.matvars['p0'],
                "targetPos": p.matvars['solution'].achieved_target,  "Fleg": p.matvars['solution'].Fleg,
                "Fr_r": p.matvars['solution'].Fr_r, "Fr_l": p.matvars['solution'].Fr_l,  "Tf": p.matvars['solution'].Tf }]

    # jump parameters
    p.startJump = 2.5
    p.orientTime = 0.05

    p.stateMachine = 'idle'
    p.jumpNumber  = 0
    p.numberOfJumps = len(p.jumps)
    p.start_logging = np.inf

    # p0 is defined wrt anchor1 pos in matlab convention
    p0 = p.jumps[p.jumpNumber]["p0"] # np.array([0.5, 2.5, -6])
    # set the rope base joint variables to initialize in p0 position, the leg ones are defined in params.yaml
    p.q_des[:12] = p.computeJointVariables(p0)

    print(colored(f"Start orienting leg to (pitch, roll)  : {p.getImpulseAngle()}", "blue"))
    p.q_des[p.hip_pitch_joint], p.q_des[p.hip_roll_joint]  = p.getImpulseAngle()

    #p.setSimSpeed(dt_sim=0.001, max_update_rate=300, iters=1500)

    while not ros.is_shutdown():

        # update the kinematics
        p.updateKinematicsDynamics()

        #multiple jumps state machine
        if ( p.stateMachine == 'idle') and (p.time >= p.startJump) and (p.jumpNumber<p.numberOfJumps):
            #set the end of orienting
            p.end_orienting = p.startJump + p.orientTime
            p.end_thrusting = p.startJump + p.orientTime + p.jumps[p.jumpNumber]["thrustDuration"]
            p.start_logging = p.end_orienting
            p.stateMachine = 'orienting_leg'  # this phase olny waits is not doing anything

        if (p.stateMachine == 'orienting_leg') and (p.time >= p.end_orienting):
            print("\033[34m" + "---------Starting jump  number ", p.jumpNumber, " to target: ",
                  p.jumps[p.jumpNumber]["targetPos"], " from p0 : ", p.base_pos - p.mat2Gazebo)

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
            # plot forces
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

                # manage lander retracting leg
                if  p.landing:
                    # retract knee joint and extend landing joints
                    p.tau_ffwd[p.landing_joints] = np.zeros(2)
                    #retract leg and move langing elements
                    p.q_des[p.leg_index[2]] = 0.25
                    if p.impedance_landing:
                        p.q_des[p.landing_joints] = np.array([-0.6, 0.6])
                        p.stateMachine = 'flying_and_reorient_lander'
                print(colored("Start "+ p.stateMachine, "blue"))

        if (p.stateMachine == 'flying'):
            # compute orientation controller TODO

            # applying forces to ropes
            delta_t = p.time - p.end_orienting
            p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)]
            p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)]
            #plot forces
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
                matlab_target = p.jumps[p.jumpNumber]["targetPos"]
                p.jumpNumber += 1
                if (p.jumpNumber < p.numberOfJumps):
                    p.stateMachine = 'idle'
                    # reset for multiple jumps
                    p.startJump = p.time
                else:
                    #p.pause_physics_client()
                    print(colored(f"target achieved (in matlab convention) is: {p.base_pos-p.mat2Gazebo}", "blue"))
                    print(colored(f" while it should be  {matlab_target}", "blue"))
                    break

        if (p.stateMachine == 'flying_and_reorient_lander'):
            # applying forces to ropes, when time is finished just rset rope length (only once!) and wait for tf
            delta_t = p.time - p.end_orienting
            if not p.optimal_control_traj_finished:
                if p.getIndex(delta_t) == -1:
                    # start again pid gains and reset qdes
                    p.resetRope()
                    p.optimal_control_traj_finished = True
                else:
                    p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)]
                    p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)]
                # check for early td and in case reset rope
                if p.detectTouchDown():
                    p.resetRope()
                    print(colored("Early TD detected, Start landing", "blue"))
                    p.stateMachine = 'landing'
            else: # you are checking for delayed TD you have already reset rope
                if p.detectTouchDown():
                    print(colored("Start landing", "blue"))
                    p.stateMachine = 'landing'

            # plot forces
            p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=4.5)
            p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=4.5)
            p.tau_ffwd[p.rope_index[0]] = p.Fr_r
            p.tau_ffwd[p.rope_index[1]] = p.Fr_l
            end_flying = p.startJump + p.orientTime + p.jumps[p.jumpNumber]["Tf"]

            # reorient legs to land parallel
            rpy = p.math_utils.rot2eul(p.w_R_b)
            p.q_des[p.landing_joints] = np.array([-0.8, 0.8]) - np.array([rpy[2], rpy[2]])


        if (p.stateMachine == 'landing'):
            p.tau_ffwd[p.landing_joints] = p.computeLandingControl()

        # plot ropes
        p.ros_pub.add_arrow(p.anchor_pos, (p.hoist_l_pos - p.anchor_pos), "green", scale=3.)  # arope, already in gazebo
        p.ros_pub.add_arrow(p.anchor_pos2, (p.hoist_r_pos-p.anchor_pos2), "green", scale=3.)  # arope, already in gazebo

        # plot contact forces on landing legs
        if p.landing:
            p.ros_pub.add_arrow(p.x_landing_l, p.contactForceW_l / p.force_scale, "blue", scale=4.5)
            p.ros_pub.add_arrow(p.x_landing_r, p.contactForceW_r / p.force_scale, "blue", scale=4.5)
        # plot contact force on retractable leg
        p.ros_pub.add_arrow(p.x_ee, p.contactForceW / p.force_scale, "blue", scale=4.5)
        #plot target position
        p.ros_pub.add_marker(p.mat2Gazebo + p.jumps[p.jumpNumber]["targetPos"], color="red", radius=0.3)
        p.ros_pub.add_marker(p.x_ee, radius=0.05)
        p.ros_pub.publishVisual()

        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),3)  # to avoid issues of dt 0.0009999
        if (p.time > p.start_logging):
            p.logData()
        # wait for synconization of the control loop
        rate.sleep()


def plot3D(name, figure_id, label, time_log, var, time_mat = None, var_mat = None):
    fig = plt.figure(figure_id)
    fig.suptitle(name, fontsize=20)

    plt.subplot(3,1,1)
    plt.ylabel(label[0])
    plt.plot(time_log, var[0, :], linestyle='-', marker="o", markersize=0,  lw=5, color='red')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[0, :], linestyle='-', marker="o", markersize=0,  lw=5, color='blue')
    plt.grid()
    plt.legend(['sim', 'matlab'])

    plt.subplot(3,1,2)
    plt.ylabel(label[1])
    plt.plot(time_log, var[1, :], linestyle='-', marker="o", markersize=0,  lw=5, color='red')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[1, :], linestyle='-', marker="o", markersize=0,  lw=5, color='blue')
    plt.grid()
    plt.legend(['sim', 'matlab'])

    plt.subplot(3,1,3)
    plt.ylabel(label[2])
    plt.plot(time_log, var[2, :], linestyle='-', marker="o", markersize=0,  lw=5, color='red')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[2, :], linestyle='-', marker="o", markersize=0,  lw=5, color='blue')
    plt.grid()
    plt.legend(['sim', 'matlab'])

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

        #plotFrameLinear('position', p.time_log, Pose_log=p.base_rpy_log, title='Roll Pitch Yaw')
        # plot rope forces
        # plt.figure()
        # plt.subplot(2, 1, 1)
        # plt.ylabel("Fr_l")
        # plt.plot(p.time_log, p.Fr_l_log, color='red')
        # plt.plot(p.time_log,p.Fr_l_fbk_log, color='blue')
        # plt.grid()
        # plt.subplot(2, 1, 2)
        # plt.ylabel("Fr_r")
        # plt.plot(p.time_log, p.Fr_r_log,color='red')
        # plt.plot(p.time_log,p.Fr_r_fbk_log, color='blue')
        # plt.grid()

        from operator import itemgetter
        #plot rope joints
        subset = p.rope_index
        plotJoint('position', p.time_log, p.q_log[subset,:], p.q_des_log[subset,:],
                  joint_names=itemgetter(*subset)(conf.robot_params[p.robot_name]['joint_names']))


        # plotJoint('torque', p.time_log, tau_log=p.tau_log, tau_ffwd_log = p.tau_ffwd_log,
        #           joint_names=conf.robot_params[p.robot_name]['joint_names'])
        if conf.plotting:
            p.plotStuff()


        
